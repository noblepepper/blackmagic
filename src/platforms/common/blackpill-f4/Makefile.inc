CROSS_COMPILE ?= arm-none-eabi-
BMP_BOOTLOADER ?=
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS +=                          	\
	-Istm32/include                 \
	-Iinclude                 	\
	-Itarget			\
	-mcpu=cortex-m4                 \
	-mthumb                         \
	-mfloat-abi=hard                \
	-mfpu=fpv4-sp-d16               \
	-DSTM32F4                       \
	-I../libopencm3/include         \
	-Iplatforms/common/stm32        \
	-Iplatforms/$(BOARD) \
	-Iplatforms/common/blackpill-f4

LDFLAGS +=                      \
	-lopencm3_stm32f4       \
	-Wl,-T,$(LINKER_SCRIPT) \
	-nostartfiles           \
	-lc                     \
	-lnosys                 \
	-Wl,-Map=mapfile        \
	-mthumb                 \
	-mcpu=cortex-m4         \
	-Wl,-gc-sections        \
	-mfloat-abi=hard        \
	-mfpu=fpv4-sp-d16       \
	-L../libopencm3/lib

CFLAGS += -DDFU_SERIAL_LENGTH=13

VPATH +=                          \
	platforms/common/stm32        \
	platforms/common/blackpill-f4

SRC +=               \
	blackpill-f4.c   \
	serialno.c       \

all:	usb-in-out.bin 

host_clean:
	-$(Q)$(RM) usb-in-out.bin
