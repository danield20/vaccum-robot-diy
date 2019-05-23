PORT ?= /dev/ttyUSB0

all: vaccum.hex

program: upload

vaccum.elf: vaccum.c
	avr-g++ -mmcu=atmega324p -DF_CPU=16000000 -Wall -Os -o $@ $^

vaccum.hex: vaccum.elf
	avr-objcopy -j .text -j .data -O ihex vaccum.elf vaccum.hex
	avr-size vaccum.elf

clean:
	rm -rf vaccum.elf vaccum.hex

.PHONY: all clean program upload

