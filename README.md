# OrangePiFmRds — FM/RDS transmitter for Orange Pi PC (Allwinner H3)

Порт [PiFmRds](https://github.com/ChristopheJacquet/PiFmRds) на **Orange Pi PC**
с SoC **Allwinner H3** (Cortex-A7, quad-core).

---

## Как это работает (технические отличия от RPi)

| | Raspberry Pi | Orange Pi PC (H3) |
|---|---|---|
| SoC | Broadcom BCM2835/2837 | Allwinner H3 |
| Механизм генерации несущей | PWM + DMA → GPCLK (GPIO4) | CCU PLL_VIDEO + SDM → CLK_OUT1 (PA1) |
| Управление памятью | VideoCore mailbox (`/dev/vcio`) | прямой доступ `/dev/mem` |
| Пин несущей | GPIO4 (физический пин 7) | PA1 (физический пин 11) |
| Модуляция | DMA записывает делитель GPCLK | Цикл записи SDM-регистра PLL_VIDEO |
| Нагрузка CPU | ~1.6% (DMA делает всё) | ~3–5% (программный цикл) |

### Механизм модуляции

Orange Pi PC использует модуль **CCU (Clock Control Unit)** Allwinner H3.
Несущая генерируется через **PLL_VIDEO** с включённым **SDM (Sigma-Delta Modulator)**,
который позволяет дробное управление частотой.

```
PLL_VIDEO = 24 MHz * (N + frac/131072) / M
CLK_OUT1  = PLL_VIDEO / 2    → выход на PA1
```

FM-модуляция выполняется записью поля `frac` регистра `PLL_VIDEO_SDM`
(CCU + 0x0284) в плотном цикле с периодом ~4.4 мкс (228 кГц).

---

## Аппаратные требования

- **Orange Pi PC** (Allwinner H3)
- Кабель/провод от **PA1 (физический пин 11)** к приёмнику
  (⚠ никогда не подключайте антенну!)
- Любая ОС на ядре Linux ≥ 4.9 с поддержкой `/dev/mem`

### Пин PA1 на 40-пиновом разъёме Orange Pi PC

```
         3.3V  [ 1] [ 2]  5V
    PA12/SDA1  [ 3] [ 4]  5V
    PA11/SCL1  [ 5] [ 6]  GND
       PA6/PWM [ 7] [ 8]  PA13/UART2_TX
          GND  [ 9] [10]  PA14/UART2_RX
 ★ PA1/CLK_OUT1[11] [12]  PD14
         PA0   [13] [14]  GND
         ...
```

**Пин 11 = PA1 = CLK_OUT1** — именно здесь появится FM-несущая.

---

## Установка зависимостей

```bash
# Armbian / Ubuntu / Debian на Orange Pi PC
sudo apt update
sudo apt install -y build-essential libsndfile1-dev
```

---

## Сборка

```bash
cd src
make
```

---

## Запуск

```bash
# Требуется root для доступа к /dev/mem
sudo ./pi_fm_rds -freq 107.9 -audio sound.wav -ps "OPI" -rt "Hello from Orange Pi" -pi 1234

# Без аудио (только несущая + RDS)
sudo ./pi_fm_rds -freq 100.0 -ps "TEST"

# С PPM-коррекцией генератора
sudo ./pi_fm_rds -freq 107.9 -audio music.wav -ppm 120
```

### Параметры

| Параметр | Описание |
|----------|----------|
| `-freq <MHz>` | Частота несущей (76–108 МГц), по умолчанию 107.9 |
| `-audio <file>` | WAV-файл для передачи (44100 Гц, моно или стерео) |
| `-ps <text>` | Имя станции RDS (до 8 символов) |
| `-rt <text>` | Радиотекст RDS (до 64 символов) |
| `-pi <hex>` | PI-код RDS (например `1234`) |
| `-ppm <value>` | Коррекция кварцевого генератора в ppm |
| `-ctl <pipe>` | Путь к control pipe для динамической смены PS/RT |

---

## Управление через pipe

```bash
# Открыть управляющий поток
mkfifo /tmp/rds_ctl

# Запустить с ним
sudo ./pi_fm_rds -freq 107.9 -ctl /tmp/rds_ctl &

# Сменить PS на лету
echo "PS=NewName" > /tmp/rds_ctl
echo "RT=New radio text here" > /tmp/rds_ctl
```

---

## ⚠ Важные предупреждения

1. **Законодательство**: FM-передача без лицензии незаконна в большинстве стран.
   Используйте только для лабораторных тестов с экранированным кабелем.

2. **Без антенны**: Никогда не подключайте антенну. Даже без антенны сигнал
   может распространяться на несколько метров.

3. **root-доступ**: Программа требует прямого доступа к `/dev/mem`.
   Запускайте через `sudo`.

4. **Гармоники**: GPIO генерирует прямоугольный сигнал с нечётными гармониками
   (3-я, 5-я, 7-я...). Без ФНЧ-фильтра это нарушает диапазоны GSM, APCO и др.

---

## Отличия от оригинального кода RPi

Все изменения сосредоточены в `src/pi_fm_rds.c`:

- **Убран** `mailbox.c` / `mailbox.h` (VideoCore mailbox — специфика RPi/Broadcom)
- **Заменены** адреса периферии BCM → Allwinner H3 (CCU + PIO)
- **Убраны** PWM / DMA регистры; вместо них — прямая запись SDM-регистра PLL_VIDEO
- **Изменена** функция `pll_compute()`: вычисляет N, M, frac для `PLL_VIDEO + SDM`
- **Функция модуляции**: вместо DMA-кольца используется `while` + `udelay(4)`
- **Пин**: GPIO4 (RPi) → PA1 / CLK_OUT1 (Orange Pi PC)
- Все файлы `rds.c`, `fm_mpx.c`, `control_pipe.c`, `waveforms.c` — **без изменений**

---

## Производительность

На Allwinner H3 @ 1.2 GHz:
- CPU load: ~3–5%
- Джиттер таймера `udelay(4)`: ±1–2 мкс (приемлемо для FM)
- Качество модуляции: сопоставимо с RPi при использовании 44100 Гц WAV

---

## Лицензия

GPL-2.0-or-later (наследует лицензию оригинального PiFmRds).
