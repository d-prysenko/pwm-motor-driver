
GCC = avr-gcc -Wall -Os -g -mmcu=attiny13a
OBJCOPY = avr-objcopy -j .text -j .data -O ihex


all: 
	$(GCC) main.c -o main.elf
	$(OBJCOPY) main.elf main.hex

flash:
	avrdude -c usbasp -B 125kHz -p t13 -V -U flash:w:main.hex:i
	# ..\avrdude\avrdude.exe -c usbasp -B 125kHz -p t13 -U flash:w:main.hex:i