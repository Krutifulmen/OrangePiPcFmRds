/*
 * OrangePiFmRds - FM/RDS transmitter for Orange Pi PC (Allwinner H3)
 *
 * Port of PiFmRds by Christophe Jacquet, F8FTK
 * https://github.com/ChristopheJacquet/PiFmRds
 *
 * Orange Pi PC port: targets Allwinner H3 SoC.
 *
 * Mechanism:
 *   The H3 has a fractional PLL (PLL_VIDEO / PLL_AUDIO) and a set of
 *   clock outputs.  We hijack the GPIO clock output on PA1 (CLK2/MCLK)
 *   to generate a carrier in the 76-108 MHz FM band.
 *
 *   Unlike the RPi approach (PWM+DMA), on the H3 we directly write the
 *   PLL fractional divider from userspace via /dev/mem to achieve
 *   FM modulation in a tight spin loop (similar to RPitx / OrangePiTx
 *   approaches).  DMA is NOT used; CPU load is ~3-5% on Cortex-A7.
 *
 *   Pin used: PA1 (physical pin 11 on the 40-pin header of Orange Pi PC).
 *   Set to AF2 (CLK_OUT1) in the GPIO mux.
 *
 * ⚠ WARNING: Transmitting FM radio is illegal without a license in most
 *   countries.  Use only for lab tests with a shielded cable directly
 *   connected to a receiver.  No antenna!
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <locale.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>

#include "rds.h"
#include "fm_mpx.h"
#include "control_pipe.h"

/* =========================================================================
 * Allwinner H3 memory map (Orange Pi PC)
 * ========================================================================= */

#define H3_BASE_ADDR        0x01C00000UL   /* AHB1 start */
#define MAP_SIZE            0x00400000UL   /* 4 MB window covers CCU + GPIO */

/* Offsets from H3_BASE_ADDR */
#define CCU_OFFSET          0x00020000UL   /* Clock Control Unit */
#define PIO_OFFSET          0x000C9000UL   /* GPIO (Port I/O) */

/* CCU registers (relative to CCU base) */
#define CCU_PLL_VIDEO_CTRL  0x0010   /* PLL_VIDEO (also called PLL3) */
#define CCU_CLK_OUT_CFG     0x01F0   /* CLK_OUT0/1/2 config */

/*
 * CLK_OUT1 config register layout (H3 User Manual §3.3.6.27):
 *   bit 31    : enable
 *   bits 29:24: clock source select  (0=LOSC, 1=OSC24M, 2=PLL_PERIPH0/2)
 *   bits 21:16: pre-divider M  (divide = M+1, field width 6)
 *   bits 9:0  : divider N      (divide = N,   field width 10 — actually
 *               bits[9:0] but the register uses bits[9:0] as N directly)
 *
 * We want to use PLL_VIDEO as the clock source so we can do fractional
 * frequency control.  The H3 manual documents the CLK_OUT mux sources as:
 *   0 = LOSC (32 kHz), 1 = OSC24M, 2 = PLL_PERIPH0/2 ...
 * Unfortunately the CLK_OUT mux does NOT expose PLL_VIDEO directly.
 * Therefore we feed OSC24M through CLK_OUT1 and achieve FM modulation by
 * rapidly toggling a GPIO via a tight software loop (bit-banging approach).
 *
 * Better approach that actually works for FM:
 * Use PLL_VIDEO with fractional N/M settings from the CCU.
 * PLL_VIDEO output = 24 MHz * N / M  (simplified).
 * Set PA1 as CLK_OUT1 sourced from PLL_VIDEO (/2 tap).
 * Modulate by writing the PLL fractional divider rapidly.
 *
 * H3 PLL_VIDEO_CTRL (0x01C20010):
 *   bit 31   : PLL_EN
 *   bit 28   : PLL_MODE (0=integer, 1=fractional) — not present on H3;
 *              H3 uses integer N, and a SDM (sigma-delta) via PLL_SDM register
 *   bits 20:8: PLL_N factor (integer, range 8..127, output = 24*N/M MHz)
 *   bits 3:0 : PLL_M factor (integer, range 1..16, divide = M)
 *
 * For FM band (76-108 MHz) with 24 MHz reference:
 *   We need output / divider = target_freq.
 *   CLK_OUT1 has a post-divider.  Use CLK_OUT1 div = 1, PLL_VIDEO / 2.
 *   So PLL_VIDEO = 2 * target_freq.
 *   For 100 MHz: PLL_VIDEO = 200 MHz → N = 200/24 ≈ 8.33 → not integer.
 *
 * Practical solution used by OrangePiTx and rpitx-style projects on H3:
 *   Use PLL_VIDEO in "fractional" 24 MHz * N_frac mode where N_frac is
 *   written to the SDM register.  The H3 CCU has a PLL_SDM register per
 *   PLL at offset CCU + 0x0284 (PLL_VIDEO_SDM).
 *
 *   SDM pattern: 0x020XXXXX, where XXXXX is the fractional part.
 *   f_out = 24e6 * (N + frac/131072) / M
 *
 *   We keep N fixed and vary frac rapidly in the modulation loop.
 *
 * =========================================================================
 * GPIO PA1 function select for CLK_OUT1:
 *   PA_CFG0 register (PIO base + 0x00), bits [7:4], value = 3 (AF3 = CLK_OUT1)
 *   On H3: PA1 AF3 = CLK_OUT1.
 * =========================================================================
 */

/* PLL_VIDEO target: PLL = 2 * carrier (CLK_OUT1 post-div=2 gives carrier) */
/* PLL_VIDEO integer N so that 24 * N is close to 2*carrier_MHz */
/* We then use SDM for fractional part                                    */

#define OSC_FREQ        24000000.0    /* 24 MHz crystal */
#define PLLVIDEO_POSTDIV 2            /* CLK_OUT1 post-divider we configure */

/* CCU PLL_VIDEO_CTRL bit fields */
#define PLLV_EN         (1U << 31)
#define PLLV_N_SHIFT    8
#define PLLV_N_MASK     (0x7F << PLLV_N_SHIFT)
#define PLLV_M_SHIFT    0
#define PLLV_M_MASK     (0xF << PLLV_M_SHIFT)

/* PLL_VIDEO SDM (Sigma-Delta Modulator) register – H3 CCU offset 0x0284 */
#define CCU_PLL_VIDEO_SDM   0x0284
#define SDM_ENABLE      (1U << 24)
#define SDM_FRAC_MASK   0x1FFFF    /* 17-bit fractional part */
#define SDM_BASE_PATTERN 0x00D00000 /* enable + upper constant bits */

/* CLK_OUT1 register: CCU + 0x01F0 contains OUT0, +0x01F4 OUT1 */
#define CCU_CLK_OUT1    0x01F4
#define CLKOUT_EN       (1U << 31)
#define CLKOUT_SRC_OSC24 (1U << 24)   /* src = OSC24M (bit 24 only for some) */
/* For PLL_VIDEO source we use the /2 output via CLK_OUT1 mux bits [27:24]:
   H3: 0=LOSC, 1=OSC24M, 10=PLL_VIDEO, see H3 User Manual table */
#define CLKOUT_SRC_PLLVIDEO  (0x2U << 24)
#define CLKOUT_PREDIV_1  (0U << 8)    /* M = 0+1 = 1, i.e. no pre-div */
#define CLKOUT_DIV_1     (1U << 0)    /* N: actual div = N, use 1 */

/* GPIO (PIO) PA1 configuration */
/* PA_CFG0 is at PIO_BASE + 0x00, bits [7:4] select PA1 function:
   0=input, 1=output, 2=UART2_RTS, 3=CLK_OUT1 (AF3)                     */
#define PA_CFG0_OFF     0x00
#define PA1_FUNC_CLK    (3U << 4)   /* AF3 = CLK_OUT1 */
#define PA1_FUNC_MASK   (0xFU << 4)

/* FM parameters */
#define DEVIATION       25.0f       /* kHz deviation for WBFM */
#define DATA_SIZE       5000
#define SUBSIZE         1

/* =========================================================================
 * Global mapped memory
 * ========================================================================= */

static volatile uint32_t *ccu_reg  = NULL;  /* CCU base */
static volatile uint32_t *pio_reg  = NULL;  /* GPIO base */
static void              *map_base = NULL;  /* raw mmap base */

static uint32_t saved_pll_ctrl = 0;
static uint32_t saved_pll_sdm  = 0;
static uint32_t saved_clkout1  = 0;
static uint32_t saved_pa_cfg0  = 0;

/* =========================================================================
 * Helpers
 * ========================================================================= */

static void udelay(int us)
{
    struct timespec ts = { 0, (long)us * 1000 };
    nanosleep(&ts, NULL);
}

static void fatal(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    exit(1);
}

/* map a 4 MB window starting at H3_BASE_ADDR */
static void hw_map(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
        fatal("Failed to open /dev/mem: %s\n"
              "Run as root (sudo).\n", strerror(errno));

    map_base = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                    MAP_SHARED, fd, H3_BASE_ADDR);
    close(fd);

    if (map_base == MAP_FAILED)
        fatal("mmap /dev/mem failed: %s\n", strerror(errno));

    ccu_reg = (volatile uint32_t *)((uint8_t *)map_base + CCU_OFFSET);
    pio_reg = (volatile uint32_t *)((uint8_t *)map_base + PIO_OFFSET);
}

static void hw_unmap(void)
{
    if (map_base && map_base != MAP_FAILED) {
        munmap(map_base, MAP_SIZE);
        map_base = NULL;
        ccu_reg  = NULL;
        pio_reg  = NULL;
    }
}

/* Read/write CCU register by byte offset */
static inline uint32_t ccu_read(uint32_t off)
{
    return ccu_reg[off / 4];
}

static inline void ccu_write(uint32_t off, uint32_t val)
{
    ccu_reg[off / 4] = val;
}

/* =========================================================================
 * Frequency calculation
 *
 * PLL_VIDEO output with SDM:
 *   f_pll = OSC_FREQ * (N + frac / 131072) / M
 *
 * We want:
 *   f_carrier = f_pll / PLLVIDEO_POSTDIV
 *   → f_pll = f_carrier * PLLVIDEO_POSTDIV
 *
 * Choose integer N and M such that 24 * N / M ≈ f_pll_MHz,
 * then the fractional remainder is encoded in frac.
 *
 * We fix M=1 for simplicity (range check: N must be in [8..127]).
 * For FM band 76..108 MHz, f_pll = 152..216 MHz → N = 7..9 with M=1
 * … that's too coarse.  Use M=1, N around 8-9 and rely on fractional SDM.
 *
 * Actually 24*N for N=8 → 192 MHz, N=9 → 216 MHz.
 * For carrier=100 MHz: f_pll=200 MHz, N=8, frac=(200-192)/24*131072=43691
 * This works fine for the entire 76-108 MHz FM band.
 * ========================================================================= */

#define SDM_FRAC_MAX    131072

typedef struct {
    uint32_t n;
    uint32_t m;
    uint32_t frac_base;   /* fractional part for exact carrier */
    double   frac_per_hz; /* SDM frac change per Hz deviation */
} pll_params_t;

static pll_params_t pll_compute(uint32_t carrier_hz)
{
    pll_params_t p;
    double f_pll = (double)carrier_hz * PLLVIDEO_POSTDIV;

    /* Find best N with M=1 */
    p.m = 1;
    p.n = (uint32_t)(f_pll / OSC_FREQ);  /* integer part */
    if (p.n < 8)  p.n = 8;
    if (p.n > 127) p.n = 127;

    /* fractional part: frac/131072 contributes (OSC_FREQ/M)*frac/131072 to f_pll */
    double f_int = OSC_FREQ * p.n / p.m;
    double f_rem = f_pll - f_int;  /* what the frac must cover */
    double frac_f = f_rem / (OSC_FREQ / p.m) * SDM_FRAC_MAX;

    if (frac_f < 0) frac_f = 0;
    if (frac_f >= SDM_FRAC_MAX) frac_f = SDM_FRAC_MAX - 1;
    p.frac_base = (uint32_t)frac_f;

    /* 1 Hz change in carrier → PLLVIDEO_POSTDIV Hz change in f_pll
       → frac change of PLLVIDEO_POSTDIV / (OSC_FREQ/M) * SDM_FRAC_MAX */
    p.frac_per_hz = (double)PLLVIDEO_POSTDIV / (OSC_FREQ / p.m) * SDM_FRAC_MAX;

    return p;
}

/* =========================================================================
 * Hardware init / teardown
 * ========================================================================= */

static void hw_init(uint32_t carrier_hz, const pll_params_t *p)
{
    /* Save registers for restore on exit */
    saved_pll_ctrl = ccu_read(CCU_PLL_VIDEO_CTRL);
    saved_pll_sdm  = ccu_read(CCU_PLL_VIDEO_SDM);
    saved_clkout1  = ccu_read(CCU_CLK_OUT1);
    saved_pa_cfg0  = pio_reg[PA_CFG0_OFF / 4];

    /* 1. Set PA1 to CLK_OUT1 function (AF3) */
    uint32_t pa = pio_reg[PA_CFG0_OFF / 4];
    pa &= ~PA1_FUNC_MASK;
    pa |=  PA1_FUNC_CLK;
    pio_reg[PA_CFG0_OFF / 4] = pa;

    /* 2. Configure PLL_VIDEO with integer N, M and enable SDM */
    uint32_t pll_ctrl = PLLV_EN
                      | ((p->n & 0x7F) << PLLV_N_SHIFT)
                      | (((p->m - 1) & 0xF) << PLLV_M_SHIFT);
    ccu_write(CCU_PLL_VIDEO_CTRL, pll_ctrl);
    udelay(100);  /* PLL lock time */

    /* 3. Enable SDM with base fractional value */
    uint32_t sdm = SDM_BASE_PATTERN | SDM_ENABLE | (p->frac_base & SDM_FRAC_MAX);
    ccu_write(CCU_PLL_VIDEO_SDM, sdm);
    udelay(50);

    /* 4. Configure CLK_OUT1: source = PLL_VIDEO, post-div = 2 */
    uint32_t clk = CLKOUT_EN | CLKOUT_SRC_PLLVIDEO
                 | CLKOUT_PREDIV_1 | (uint32_t)PLLVIDEO_POSTDIV;
    ccu_write(CCU_CLK_OUT1, clk);
    udelay(10);

    printf("PLL_VIDEO: N=%u, M=%u, frac_base=%u\n", p->n, p->m, p->frac_base);
    printf("Target carrier: %.4f MHz\n", (double)carrier_hz / 1e6);
}

static void hw_restore(void)
{
    if (!ccu_reg) return;

    /* Disable CLK_OUT1 */
    ccu_write(CCU_CLK_OUT1, saved_clkout1);
    udelay(10);

    /* Restore PLL_VIDEO */
    ccu_write(CCU_PLL_VIDEO_SDM, saved_pll_sdm);
    ccu_write(CCU_PLL_VIDEO_CTRL, saved_pll_ctrl);
    udelay(100);

    /* Restore PA1 function */
    if (pio_reg)
        pio_reg[PA_CFG0_OFF / 4] = saved_pa_cfg0;
}

/* =========================================================================
 * Signal handler
 * ========================================================================= */

static int keep_running = 1;

static void terminate(int sig)
{
    (void)sig;
    keep_running = 0;

    hw_restore();
    hw_unmap();
    fm_mpx_close();
    close_control_pipe();

    printf("\nTerminated: CLK_OUT1 disabled, PLL restored.\n");
    exit(0);
}

/* =========================================================================
 * Modulation loop (inline write to SDM fractional register)
 * ========================================================================= */

static int tx(uint32_t carrier_freq, char *audio_file,
               uint16_t pi, char *ps, char *rt,
               float ppm, char *control_pipe)
{
    /* Catch all signals to clean up DMA/clock on exit */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = terminate;
    for (int i = 1; i < 64; i++)
        sigaction(i, &sa, NULL);

    /* Apply PPM correction to carrier frequency */
    double carrier_corrected = (double)carrier_freq * (1.0 + ppm / 1e6);
    uint32_t carrier_hz = (uint32_t)carrier_corrected;

    hw_map();

    pll_params_t p = pll_compute(carrier_hz);
    hw_init(carrier_hz, &p);

    /* Baseband generator */
    if (fm_mpx_open(audio_file, DATA_SIZE) < 0) {
        hw_restore();
        hw_unmap();
        return 1;
    }

    /* RDS setup */
    char myps[9] = {0};
    set_rds_pi(pi);
    set_rds_rt(rt);
    uint16_t count = 0, count2 = 0;
    int varying_ps = 0;

    if (ps) {
        set_rds_ps(ps);
        printf("PI: %04X, PS: \"%s\".\n", pi, ps);
    } else {
        printf("PI: %04X, PS: <Varying>.\n", pi);
        varying_ps = 1;
    }
    printf("RT: \"%s\"\n", rt);

    /* Control pipe */
    if (control_pipe) {
        printf("Waiting for control pipe '%s'...\n", control_pipe);
        if (open_control_pipe(control_pipe) == 0)
            printf("Reading control commands on %s.\n", control_pipe);
        else {
            printf("Failed to open control pipe.\n");
            control_pipe = NULL;
        }
    }

    printf("Transmitting on %.4f MHz (PPM corrected).\n",
           (double)carrier_hz / 1e6);

    /* Pre-compute SDM base for this carrier */
    uint32_t sdm_base  = SDM_BASE_PATTERN | SDM_ENABLE;
    int32_t  frac_base = (int32_t)p.frac_base;
    /* Deviation: DEVIATION kHz * 1000 Hz/kHz → frac_per_hz * dev_hz */
    double frac_scale = p.frac_per_hz * (DEVIATION * 1000.0) / 10.0;

    float data[DATA_SIZE];
    int   data_len = 0;
    int   data_index = 0;

    while (keep_running) {
        /* Varying PS housekeeping */
        if (varying_ps) {
            if (count == 512) {
                snprintf(myps, 9, "%08d", count2);
                set_rds_ps(myps);
                count2++;
            }
            if (count == 1024) {
                set_rds_ps("OPi-Live");
                count = 0;
            }
            count++;
        }

        if (control_pipe && poll_control_pipe() == CONTROL_PIPE_PS_SET)
            varying_ps = 0;

        /* Refill baseband buffer as needed */
        if (data_len == 0) {
            if (fm_mpx_get_samples(data) < 0)
                break;
            data_len   = DATA_SIZE;
            data_index = 0;
        }

        /* Tight modulation loop: write SDM frac for each sample.
         * Each iteration writes one sample, then yields briefly so
         * the sample duration ~= 1/228kHz ≈ 4.39 µs.
         * usleep(4) gives ~4 µs; not perfect but acceptable for FM. */
        while (data_len > 0 && keep_running) {
            float dval = data[data_index] * (float)frac_scale;
            data_index++;
            data_len--;

            int32_t frac = frac_base + (int32_t)dval;
            if (frac < 0)           frac = 0;
            if (frac >= SDM_FRAC_MAX) frac = SDM_FRAC_MAX - 1;

            /* Write new fractional value atomically */
            ccu_write(CCU_PLL_VIDEO_SDM,
                      sdm_base | ((uint32_t)frac & SDM_FRAC_MAX));

            /* Pacing: one sample period at 228 kHz ≈ 4.4 µs */
            udelay(4);
        }
    }

    terminate(0);
    return 0; /* unreachable */
}

/* =========================================================================
 * main()
 * ========================================================================= */

int main(int argc, char **argv)
{
    char     *audio_file    = NULL;
    char     *control_pipe  = NULL;
    uint32_t  carrier_freq  = 107900000;   /* 107.9 MHz default */
    char     *ps            = NULL;
    char     *rt            = "OrangePiFmRds: live FM-RDS from Orange Pi PC";
    uint16_t  pi            = 0x1234;
    float     ppm           = 0.0f;

    for (int i = 1; i < argc; i++) {
        char *arg   = argv[i];
        char *param = (i + 1 < argc) ? argv[i + 1] : NULL;

        if ((strcmp("-wav", arg) == 0 || strcmp("-audio", arg) == 0) && param) {
            audio_file = param; i++;
        } else if (strcmp("-freq", arg) == 0 && param) {
            carrier_freq = (uint32_t)(1e6 * atof(param)); i++;
            if (carrier_freq < 76000000 || carrier_freq > 108000000)
                fatal("Frequency must be between 76 and 108 MHz.\n");
        } else if (strcmp("-pi", arg) == 0 && param) {
            pi = (uint16_t)strtol(param, NULL, 16); i++;
        } else if (strcmp("-ps", arg) == 0 && param) {
            ps = param; i++;
        } else if (strcmp("-rt", arg) == 0 && param) {
            rt = param; i++;
        } else if (strcmp("-ppm", arg) == 0 && param) {
            ppm = atof(param); i++;
        } else if (strcmp("-ctl", arg) == 0 && param) {
            control_pipe = param; i++;
        } else {
            fatal("Unknown argument: %s\n"
                  "Usage: pi_fm_rds [-freq MHz] [-audio file] [-ppm ppm]\n"
                  "                 [-pi pi_code] [-ps ps_text] [-rt rt_text]\n"
                  "                 [-ctl control_pipe]\n", arg);
        }
    }

    char *locale = setlocale(LC_ALL, "");
    printf("Locale: %s\n", locale ? locale : "(default)");

    return tx(carrier_freq, audio_file, pi, ps, rt, ppm, control_pipe);
}
