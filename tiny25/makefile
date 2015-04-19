BAUD=19200
SRC=project
AVRTYPE=attiny25
AVRFREQ=1000000 # 1MHz for accurate baudrate timing
PROGTYPE=usbasp

cflags=-g -DF_CPU=$(AVRFREQ) -Wall -Os -Werror -Wextra

memoryTypes=calibration eeprom efuse flash fuse hfuse lfuse lock signature application apptable boot prodsig usersig

all: object elf hex
run: program

object:
	avr-gcc $(cflags) -mmcu=$(AVRTYPE) -Wa,-ahlmns=$(SRC).lst -c -o $(SRC).o $(SRC).cpp 
	avr-gcc $(cflags) -mmcu=$(AVRTYPE) -Wa,-ahlmns=$(SRC).lst -c -o $(SRC).o $(SRC).cpp 

elf: object
	avr-gcc $(cflags) -mmcu=$(AVRTYPE) -o $(SRC).elf $(SRC).o
	chmod a-x $(SRC).elf 2>&1

hex: elf
	avr-objcopy -j .text -j .data -O ihex $(SRC).elf $(SRC).flash.hex

#avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex $(SRC).elf $(SRC).eeprom.hex
#avr-objcopy -j .fuse -O ihex $(SRC).elf $(SRC).fuses.hex --change-section-lma .fuse=0
#srec_cat $(SRC).fuses.hex -Intel -crop 0x00 0x01 -offset  0x00 -O $(SRC).lfuse.hex -Intel
#srec_cat $(SRC).fuses.hex -Intel -crop 0x01 0x02 -offset -0x01 -O $(SRC).hfuse.hex -Intel
#srec_cat $(SRC).fuses.hex -Intel -crop 0x02 0x03 -offset -0x02 -O $(SRC).efuse.hex -Intel

help:
	@echo 'backup       Read all known memory types from controller and write it into a file. Available memory types: $(memoryTypes)'
	@echo 'clean        Delete automatically created files.'
	@echo 'disassemble  Compile source code, then disassemble object file to mnemonics.'
	@echo 'dumpelf      Dump the contents of the .elf file. Useful for information purposes only.'
	@echo 'edit     Edit the .cpp source file.'
	@echo 'eeprom       Extract EEPROM data from .elf file and program the device with it.'
	@echo 'elf      Create $(SRC).elf'
	@echo 'flash        Program $(SRC).hex to controller flash memory.'
	@echo 'fuses        Extract FUSES data from .elf file and program the device with it.'
	@echo 'help     Show this text.'
	@echo 'hex      Create all hex files for flash, eeprom and fuses.'
	@echo 'object       Create $(SRC).o'
	@echo 'program      Do all programming to controller.'

clean: 
	rm -f $(SRC).elf $(SRC).eeprom.hex $(SRC).fuses.hex $(SRC).lfuse.hex $(SRC).hfuse.hex $(SRC).efuse.hex $(SRC).flash.hex $(SRC).o
	date


disassemble: elf
	avr-objdump -s -j .fuse $(SRC).elf
	avr-objdump -C -d $(SRC).elf 2>&1

eeprom: hex
	#avrdude -p$(AVRTYPE) -c$(PROGTYPE) -b$(BAUD) -v -U eeprom:w:$(SRC).eeprom.hex
	date

fuses: hex
	sudo avrdude -p$(AVRTYPE) -c$(PROGTYPE) -b$(BAUD) -v -U lfuse:w:$(SRC).lfuse.hex
	date

dumpelf: elf
	avr-objdump -s -h $(SRC).elf

program: flash eeprom fuses

flash: hex
	sudo avrdude -p$(AVRTYPE) -c$(PROGTYPE) -b$(BAUD) -v -U flash:w:$(SRC).flash.hex
	date

backup:
	@for memory in $(memoryTypes); do \
	sudo avrdude -p $(AVRTYPE) -c$(PROGTYPE) -b$(BAUD) -v -U $$memory:r:./$(AVRTYPE).$$memory.hex:i; \
	done

debsetup:
	sudo apt-get install avrdude srecord binutils-avr gcc-avr avr-libc gdb-avr
