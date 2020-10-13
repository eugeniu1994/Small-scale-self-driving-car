###############################################################################
# Makefile for the project WT_Move
###############################################################################

## General Flags
PROJECT = WT_Move
MCU = atmega644pa
TARGET = WT_Move.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -Os -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=WT_Move.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = WT_Move.o WildThumperLib_Main.o WT_I2Cmaster.o WT_UART.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) WT_Move.hex WT_Move.eep WT_Move.lss size

## Compile
WT_Move.o: ../WT_Move.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

WildThumperLib_Main.o: ../WildThumperLib_Main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

WT_I2Cmaster.o: ../WT_I2Cmaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

WT_UART.o: ../WT_UART.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) WT_Move.elf dep/* WT_Move.hex WT_Move.eep WT_Move.lss WT_Move.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

