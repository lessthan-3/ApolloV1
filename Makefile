
# Simple AVR Makefile for Linux (avr-gcc)
# Targets: all (default) -> .hex, .elf, .eep

################################################################################
# Configurable variables (override on make command line, e.g. `make MCU=atmega328p`)
MCU ?= atmega168pb
F_CPU ?= 8000000UL
TARGET ?= ApolloV1

# Toolchain
CC := avr-gcc
OBJCOPY := avr-objcopy
OBJDUMP := avr-objdump
SIZE := avr-size

# Source files
SRCS := main.c lcd.c
OBJS := $(SRCS:.c=.o)

# Compiler and linker flags
CFLAGS := -std=gnu11 -Wall -Wextra -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I.
# Default to size-optimized build; use `make CFLAGS="$(CFLAGS) -g -Og"` for debug
CFLAGS += -Os
LDFLAGS := -Wl,--gc-sections -mmcu=$(MCU)
LDLIBS := -lm

.PHONY: all clean size disasm

all: $(TARGET).hex

################################################################################
# Build rules

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)
	@echo "Built: $@"
	@$(SIZE) --format=avr --mcu=$(MCU) $@ || true

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	@echo "Built: $@"

# Create .eep file if the ELF contains an .eeprom section
$(TARGET).eep: $(TARGET).elf
	-$(OBJCOPY) -O ihex --only-section=.eeprom $< $@ || true
	@if [ -f $@ ] && [ ! -s $@ ]; then rm -f $@; fi

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

disasm: $(TARGET).elf
	$(OBJDUMP) -S $< > $(TARGET).lss
	@echo "Disassembly: $(TARGET).lss"

size: $(TARGET).elf
	$(SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf

################################################################################
# Helpers

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).lss $(TARGET).eep

################################################################################
# Usage notes (for convenience)
# Build (default): make
# Clean: make clean
# Override MCU or F_CPU: make MCU=atmega328p F_CPU=16000000UL
################################################################################
