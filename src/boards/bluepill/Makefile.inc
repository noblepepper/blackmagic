CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -mcpu=cortex-m3 -mthumb \
	-DSTM32F1  -I../libopencm3/include \
	-I boards/common/stm32 -Iinclude \
	-I boards/bluepill
LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs -lopencm3_stm32f1 \
	-Wl,-T,boards/bluepill/bluepill.ld -nostartfiles -lc\
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

VPATH += boards/common/stm32

SRC +=          \
	platform.c \
	serialno.c	\
	timing.c	\
	timing_stm32.c	\
	platform_common.c \

all:	usb-in-out.bin

host_clean:
	-$(Q)$(RM) usb-in-out.bin
