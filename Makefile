#
#   Makefile
#

# TARGET=atmega2560    HFUSE=0xc9  LFUSE=0xef

APPNAME = main
MMCU = atmega328
F_CPU = 20000000

# Atmega328 0x380 * 2 = 0x7000 
BOOTADDR = 0x7000

PROGRAMMER = usbasp
AVROPTIONS =
LFUSE = 0xF7
HFUSE = 0xD8
#HFUSE = 0xD9
EFUSE = 0x07

# Source files
CXXSOURCES =
CSOURCES = main.c
ASMSOURCES =

# Includes
INCLUDES += -I.
LIBDIRS += -L.
LIBS += -lm

# Optimization s (size), 0 (off) , 1, 2, 3
OPTIMIZE = s

#########################################################################################################

COMMON = -mmcu=$(MMCU) -DF_CPU=$(F_CPU)UL -O$(OPTIMIZE)     \
	-g -fpack-struct -fshort-enums -funsigned-bitfields \
	-Wl,-u,vfprintf -lprintf_flt                        \
	-funsigned-char -Wall -Wa,-ahlms=$(APPNAME).lst

ifdef BOOTADDR
COMMON += -DBOOT_START=$(BOOTADDR)
endif

CFLAGS = $(COMMON) -std=gnu99 -Wstrict-prototypes $(INCLUDES)
CXXFLAGS = $(COMMON) -std=gnu++98 -fno-exceptions $(INCLUDES)
ASMFLAGS = $(INCLUDES) -mmcu=$(MMCU) -x assembler-with-cpp
LDFLAGS = -Wl,-Map,$(APPNAME).map -mmcu=$(MMCU) $(LIBS)

CC = avr-gcc
CXX = avr-g++
AR = avr-ar
LD = avr-ld
OBJCOPY = avr-objcopy
SIZE = avr-size
OBJDUMP = avr-objdump
STRIP = avr-strip
AVRDUDE = avrdude
TARGET = $(APPNAME).elf
TARGETS = $(TARGET) $(TARGET:%.elf=%.lss) $(TARGET:%.elf=%.hex) $(TARGET:%.elf=%.eep)

#CINPUTS   = $(wildcard *.c)
#CXXINPUTS = $(wildcard *.cpp)
#ASMINPUTS = $(wildcard *.S)
#OBJECTS   = $(CINPUTS:%.c=%.o) $(CXXINPUTS:%.cpp=%.o) $(ASMINPUTS:%.S=%.o)
OBJECTS    = $(CXXSOURCES:%.cpp=%.o) $(CSOURCES:%.c=%.o) $(ASMSOURCES:%.S=%.o)

all: $(TARGETS) size

%.elf: $(OBJECTS)
ifdef BOOTADDR
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBDIRS) $(LIBS) -Wl,-section-start=.text=$(BOOTADDR) -o $(TARGET)
else
	$(CC) $(LDFLAGS) $(OBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)
endif

%.a: $(OBJECTS)
	$(AR) -c -r $@ $? $(LIBS)

%.hex: $(TARGET)
	$(OBJCOPY) -O ihex -j .text -j .data  $< $@

%.eep: $(TARGET)
	-$(OBJCOPY) -O ihex -j .text -j .data $< $@ || exit 0

%.lss: $(TARGET)
	$(OBJDUMP) -h -S $< > $@

size: $(TARGET)
	$(SIZE) --mcu=$(MMCU) -C main.elf

disasm:	$(TARGET)
	$(OBJDUMP) -d main.elf

clean:
	rm -f main.hex main.lst main.obj main.cof main.list main.map main.eep main.lss main.elf *.o main.s *~

help:
	@echo "Usage: make                same as make"
	@echo "       make help           this help"
	@echo "       make main.hex       create main.hex"
	@echo "       make clean          remove redundant data"
	@echo "       make disasm         disasm main"
	@echo "       make flash          upload main.hex into flash"
	@echo "       make fuses          program fuses"
	@echo "       make avrdude        test avrdude"
	@echo "Current values:"
	@echo "       APPNAME=${TARGET}"
	@echo "       TARGET=${MMCU}"
	@echo "       LFUSE=${LFUSE}"
	@echo "       HFUSE=${HFUSE}"
	@echo "       CLOCK=${F_CPU}"


flash: $(TARGETS)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MMCU) $(AVROPTIONS) -U flash:w:main.hex

fuses:
	$(AVRDUDE) -B 100 -c $(PROGRAMMER) -p $(MMCU) $(AVROPTIONS) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

avrdude:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MMCU) $(AVROPTIONS) -v
