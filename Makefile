# Makefile for the template/example projects in the STM32F0 StdPeriph Driver
# package.
#
# It uses GCC and OpenOCD (or stlink).
#
# Like the readme.txt file in each example directory says (in STM32 StdPeriph
# Driver library), copy the example contents over to the project template
# directory and build.
#
# Tested with Mentor Sourcery ARM GCC toolchain.

# Basename for the resulting .elf/.bin/.hex file
RESULT ?= main

# Path to the STM32F0xx_StdPeriph_Lib_V1.0.0/ directory
TOPDIR = STM32F0xx_StdPeriph_Lib_V1.0.0

SOURCES = \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/TrueSTUDIO/Project/syscalls.c \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/main.c \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.c \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/system_stm32f0xx.c \
	  $(TOPDIR)/Libraries/CMSIS/Device/ST/STM32F0xx/Source/Templates/TrueSTUDIO/startup_stm32f0xx.s \
	  $(wildcard $(TOPDIR)/Libraries/STM32F0xx_StdPeriph_Driver/src/*.c) \
	  $(TOPDIR)/Utilities/STM32_EVAL/STM320518_EVAL/stm320518_eval.c \
	  $(TOPDIR)/Utilities/STM32_EVAL/STM320518_EVAL/stm320518_eval_lcd.c

HEADERS = \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/main.h \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_conf.h \
	  $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/stm32f0xx_it.h \
	  $(wildcard $(TOPDIR)/Libraries/CMSIS/Include/*.h) \
	  $(wildcard $(TOPDIR)/Libraries/CMSIS/Device/ST/STM32F0xx/Include/*.h) \
	  $(wildcard $(TOPDIR)/Libraries/STM32F0xx_StdPeriph_Driver/inc/*.h)

LINKER_SCRIPT = $(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates/TrueSTUDIO/Project/stm32_flash.ld

INCLUDES += -I$(TOPDIR)/Libraries/CMSIS/Device/ST/STM32F0xx/Include \
	    -I$(TOPDIR)/Libraries/CMSIS/Include \
	    -I$(TOPDIR)/Libraries/STM32F0xx_StdPeriph_Driver/inc \
	    -I$(TOPDIR)/Utilities/STM32_EVAL/STM320518_EVAL \
	    -I$(TOPDIR)/Utilities/STM32_EVAL/Common \
	    -I$(TOPDIR)/Project/STM32F0xx_StdPeriph_Templates

CFLAGS += -DUSE_STDPERIPH_DRIVER -fno-common -Wall -Os -g3 -mcpu=cortex-m0 -mthumb
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS += $(INCLUDES)
CFLAGS += -Ibaselibc/include/

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size


# So that the "build depends on the makefile" trick works no matter the name of
# the makefile
THIS_MAKEFILE := $(lastword $(MAKEFILE_LIST))

all: build size

build: $(RESULT).elf $(RESULT).bin $(RESULT).hex $(RESULT).lst

$(RESULT).elf: $(SOURCES) $(HEADERS) $(LINKER_SCRIPT) $(THIS_MAKEFILE)
	$(CC) -Wl,-M=$(RESULT).map -Wl,-T$(LINKER_SCRIPT) $(CFLAGS) $(SOURCES) baselibc.a -o $@

OBJECTS = $(SOURCES:.c=.o) $(SOURCES:.s=.o)

#$(RESULT).elf: $(OBJECTS) $(HEADERS) $(LINKER_SCRIPT) $(THIS_MAKEFILE)
	#$(CC) -Wl,-M=$(RESULT).map -Wl,-T$(LINKER_SCRIPT) $(CFLAGS) $(OBJECTS) baselibc.a -o $@

%.o: %.c $(HEADERS) $(THIS_MAKEFILE)
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s $(HEADERS) $(THIS_MAKEFILE)
	$(CC) $(CFLAGS) -c $< -o $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.lst: %.elf
	$(OBJDUMP) -x -S $(RESULT).elf > $@

size: $(RESULT).elf
	$(SIZE) $(RESULT).elf

install: build
	st-flash write $(RESULT).bin 0x08000000
## Or with openocd (>= v0.6.0)
#openocd -f board/stm32f0discovery.cfg -c "init; reset halt; flash write_image erase $(RESULT).bin 0x08000000; reset run; shutdown"

clean:
	rm -f $(RESULT).elf
	rm -f $(RESULT).bin
	rm -f $(RESULT).map
	rm -f $(RESULT).hex
	rm -f $(RESULT).lst

.PHONY: all build size clean install
