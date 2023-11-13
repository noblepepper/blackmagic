BOARD ?= blackpill-f411ce
BOARD_DIR = boards/$(BOARD)
VPATH += $(BOARD_DIR) target

# CFLAGS += -Wall -Wextra -Werror -Wreturn-type \
CFLAGS += -Wall -Wextra -Wreturn-type \
	-Wno-char-subscripts \
	-std=c11 -g3 -MD -I./target \
	-I. -Iinclude -I$(BOARD_DIR)

SRC =                     \
	main.c            \
	usbwrap.c         \
	setup.c           \


$(info BOARD is $(BOARD))
$(info BOARD_DIR is $(BOARD_DIR))
include $(BOARD_DIR)/Makefile.inc

OPT_FLAGS ?= -Os
CFLAGS += $(OPT_FLAGS)
LDFLAGS += $(OPT_FLAGS)
# Output memory usage information
LDFLAGS += -Wl,--print-memory-usage

ifndef TARGET
TARGET = usb-in-out.elf
endif

include boards/common/Makefile.inc

OBJ = $(patsubst %.S,%.o,$(patsubst %.c,%.o,$(SRC)))

$(TARGET): include/version.h $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o:	%.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.o:	%.S
	@echo "  AS      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.bin:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $^ $@

%.hex:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O ihex $^ $@

.PHONY:	clean host_clean all_boards clang-format FORCE

clean:	host_clean
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) *.o *.d *.elf *.bin *~ $(TARGET) $(HOSTFILES)
	-$(Q)$(RM) boards/*/*.o boards/*/*.d mapfile ftd2xx.dll

clang-format:
	$(Q)clang-format -i *.c */*.c */*/*.c *.h */*.h */*/*.h

