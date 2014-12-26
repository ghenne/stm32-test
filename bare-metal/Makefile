CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
CFLAGS += -ffunction-sections -fdata-sections -fno-common -Os -g3 -mcpu=cortex-m0 -mthumb

all: main.elf

main.elf: main.c Makefile stm32.ld
	$(CC) -I. -c $(CFLAGS) main.c
	$(LD) -M=main.map -T stm32.ld -o main.elf main.o
	$(OBJCOPY) -O binary main.elf main.bin

size: main.elf
	$(SIZE) $<

install: main.elf
	st-flash write main.bin 0x08000000

clean:
	rm -f main.elf main.bin main.o main.map
