all: main.c
	avr-gcc -mmcu=attiny88 -DF_CPU=16000000 -Wall -funsigned-char -Os -o main.elf -I. \
		main.c usbdrv/usbdrv.c usbdrv/usbdrvasm.S usbdrv/oddebug.c
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

flash: all
	C:\Users\User\AppData\Local\Arduino15\packages\ATTinyCore\tools\micronucleus\2.5-azd1b/micronucleus --no-ansi --run --timeout 60 main.hex

dude: all
	avrdude -C "C:\Users\User\Desktop\avr dude\avrdude.conf" \
		-v -e -p attiny88 -c stk500v1 -P COM3 -b 19200 -U flash:w:main.hex

clean:
	-rm main.elf main.hex
