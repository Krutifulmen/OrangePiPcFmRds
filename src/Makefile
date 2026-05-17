CC      = gcc
CFLAGS  = -Wall -std=gnu99 -O2 -ffast-math -DORANGEPI_H3
LDFLAGS = -lsndfile -lm

# On the Orange Pi itself (armv7l / aarch64) add ARM tuning flags
UNAME := $(shell uname -m)
ifeq ($(UNAME), armv7l)
    CFLAGS += -march=armv7-a -mtune=cortex-a7 -mfloat-abi=hard -mfpu=neon-vfpv4
else ifeq ($(UNAME), aarch64)
    CFLAGS += -march=armv8-a -mtune=cortex-a53
endif

TARGET  = pi_fm_rds

OBJS = pi_fm_rds.o rds.o rds_strings.o waveforms.o fm_mpx.o \
       control_pipe.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Dependencies
pi_fm_rds.o:    pi_fm_rds.c rds.h fm_mpx.h control_pipe.h
rds.o:          rds.c waveforms.h rds_strings.h
rds_strings.o:  rds_strings.c rds_strings.h
fm_mpx.o:       fm_mpx.c fm_mpx.h rds.h
control_pipe.o: control_pipe.c control_pipe.h rds.h
waveforms.o:    waveforms.c waveforms.h

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
