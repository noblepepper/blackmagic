BOARD ?= blackpill-f411ce
PLATFORM_DIR = platforms/$(BOARD)
VPATH += $(PLATFORM_DIR) target

# CFLAGS += -Wall -Wextra -Werror -Wreturn-type \
CFLAGS += -Wall -Wextra -Wreturn-type \
	-Wno-char-subscripts \
	-std=c11 -g3 -MD -I./target \
	-I. -Iinclude -I$(PLATFORM_DIR)

SRC =              \
	main.c         \
	usbwrap.c         \

include $(PLATFORM_DIR)/Makefile.inc

OPT_FLAGS ?= -Os
CFLAGS += $(OPT_FLAGS)
LDFLAGS += $(OPT_FLAGS)

ifndef TARGET
TARGET = usb-in-out.elf
endif

include platforms/common/Makefile.inc

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

.PHONY:	clean host_clean all_platforms clang-format FORCE

clean:	host_clean
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) *.o *.d *.elf *~ $(TARGET) $(HOSTFILES)
	-$(Q)$(RM) platforms/*/*.o platforms/*/*.d mapfile ftd2xx.dll

clang-format:
	$(Q)clang-format -i *.c */*.c */*/*.c *.h */*.h */*/*.h

