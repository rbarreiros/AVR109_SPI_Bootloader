
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr109.h>
//#include <intrinsics.h>

#define VERSION_H '1'
#define VERSION_L '0'

// Util types
typedef uint8_t byte;
typedef uint8_t bool;
#define true  1
#define false 0

#define PROG_START   0x0000
#define BOOT_TIMEOUT 4000

#ifndef BOOT_START
#error "Makefile needs BOOTADDR Defined"
#endif

#define HIGH(x) ( (uint8_t) (x >> 8) )
#define LOW(x)  ( (uint8_t) x )

// Pin Definitions

#define SPI_DDR  DDRB
#define SPI_PORT PORTB
#define SPI_PIN  PINB
#define SPI_SS   PB2  // Input
#define SPI_MOSI PB3  // Input
#define SPI_MISO PB4  // Output
#define SPI_SCK  PB5  // Input

// Leds

#define LED_DDR    DDRD
#define LED_PORT   PORTD
#define LED_GREEN  PD5
#define LED_YELLOW PD6
#define LED_RED    PD7

// SPI Commands

#define EXEC_PROG 0xAC

#define READ_LOW_FLASH   0x20
#define READ_HIGH_FLASH  0x28
#define READ_DEVICEID    0x30
#define WRITE_LOW_FLASH  0x40
#define WRITE_HIGH_FLASH 0x48
#define TRANSFER_FLASH   0x4C
#define READ_EEPROM      0xA0
#define WRITE_EEPROM     0xC0

// Global

static volatile unsigned long gMilliseconds;
bool     gInProgramming = false;
byte     gBuffer[SPM_PAGESIZE];
uint16_t gFlashAddress = 0;
uint16_t gEepromAddress = 0;

// -------------------- SPI functions

void SetupSPI(void)
{
  // SS, MOSI, SCK Inputs
  SPI_DDR &= ~(1 << SPI_SS) & ~(1 << SPI_MOSI) & ~(1 << SPI_SCK);

  // MISO Output
  SPI_DDR |= (1 << SPI_MISO);

  // Enable SPI
  SPCR = (1 << SPE);
}

byte SPI_Receive(void)
{
  while(!(SPSR & (1<<SPIF)))
  {
    if(!gInProgramming)
      if(gMilliseconds > BOOT_TIMEOUT) return 0x00;
  }

  return SPDR;
}

void SPI_Send(byte data)
{
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}

// -------------------- Bootloader functions

void ExitBootloader(void)
{
  LED_PORT &= ~(1 << LED_GREEN) & ~(1 << LED_YELLOW) & ~(1 << LED_RED);

  // Disable Interrupts
  cli();
  TIMSK2 = 0;
  SPCR = 0;

  // Do not forget the move IVT back to regular flash IVT
  MCUCR = (1 << IVCE);
  MCUCR = 0;

  LED_PORT |= (1 << LED_GREEN) | (1 << LED_YELLOW);
  LED_PORT &= ~(1 << LED_RED);
  _delay_ms(500);
  LED_PORT |= (1 << LED_RED);

  boot_rww_enable();
  (*((void(*)(void))PROG_START))();
}

void SetupLeds(void)
{
  // All outputs and works as sink
  LED_DDR |= (1 << LED_GREEN) | (1 << LED_YELLOW) | (1 << LED_RED);
  LED_PORT |= (1 << LED_GREEN) | (1 << LED_YELLOW) | (1 << LED_RED);
}

void SetupTimer(void)
{
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS20);
  TIMSK2 = (1 << OCIE2A);
  OCR2A = ((F_CPU / 128) / 1000);
}

int main(void)
{
  byte data;
  uint16_t buffSize;

  // Disable Interrupts and move IVT
  cli();
  MCUCR = (1 << IVCE);
  MCUCR = (1 << IVSEL);

  // Watchdog
  //MCUSR = 0;
  wdt_disable();

  // Setup Board LEDS
  SetupLeds();

  // Setup the 1ms Timer
  SetupTimer();
  
  // Setup SPI
  SetupSPI();

  // Turn Yellow ON Signal wait for data
  LED_PORT &= ~(1 << LED_YELLOW);
  _delay_ms(500);
  LED_PORT |= (1 << LED_YELLOW);
  LED_PORT &= ~(1 << LED_GREEN);

  // Enable Interrupts
  sei();
  //int tmp = 0;

  while(1)
  {
    _delay_us(1);
    if(!gInProgramming)
      if(gMilliseconds > BOOT_TIMEOUT) break;

    data = SPI_Receive();
    switch(data)
    {
    case AVR_ENTERPROGRAMMING:  // Enter programming, suspend boot timeout
      gInProgramming = true;
      SPI_Send('\r');
      break;
    case AVR_LEAVEPROGRAMMING:  // Leave programming return to application
    case AVR_EXITBOOTLOADER:
      gInProgramming = false;
      ExitBootloader();
      break;

    case AVR_AUTOINCREMENTADDR: // Yes we support auto increment
      SPI_Send('Y');
      break;
    case AVR_SETADDRESS:        // Set start address
      gEepromAddress = (SPI_Receive() << 8) | SPI_Receive();
      gFlashAddress = gEepromAddress << 1;
      SPI_Send('\r');
      break;
    case AVR_CHIPERASE: // Erase Flash
      for(uint16_t addr = 0; addr < BOOT_START; addr += SPM_PAGESIZE)
      {
	boot_page_erase_safe(addr);
      }
      SPI_Send('\r');
      break;

    case AVR_READSIGNATURE: // Send this avr sig
      SPI_Send(SIGNATURE_0);
      SPI_Send(SIGNATURE_1);
      SPI_Send(SIGNATURE_2);
      break;
      
    case AVR_SELECTDEVICETYPE: // Ignored, only 1 device
      SPI_Receive();
      SPI_Send('\r');
      break;

    case AVR_SUPPORTEDDEVICES: // ATMega328 doesn't have an avr109/avr901 devcode
      SPI_Send(0x00);
      SPI_Send(0x00);
      break;

    case AVR_SOFTIDENTIFIER: // Send exactly 7 chars
      SPI_Send('D');
      SPI_Send('M');
      SPI_Send('X');
      SPI_Send(' ');
      SPI_Send('B');
      SPI_Send(' ');
      SPI_Send('v');
      break;

    case AVR_PROGTYPE:  // Programmer Type - Serial
      SPI_Send('S');
      break;

    case AVR_SOFTVERSION: // Send version
      SPI_Send(VERSION_H);
      SPI_Send(VERSION_L);
      break;

    case AVR_CHECKBLOCKSUPP: // Yes, support block size and block size
      SPI_Send('Y');
      SPI_Send(HIGH(SPM_PAGESIZE));
      SPI_Send(LOW(SPM_PAGESIZE));
      break;
      
    case AVR_STARTBLOCKREAD: // Read block
      buffSize = (SPI_Receive() << 8) | SPI_Receive();
      char memType = SPI_Receive();
      switch(memType)
      {
      case 'F':
	uint16_t tmp = 0;
	for(uint16_t i = 0; i < buffSize; i += 2)
	{
	  tmp = pgm_read_word(gFlashAddress);
	  SPI_Send(LOW(tmp));
	  SPI_Send(HIGH(tmp));
	  gFlashAddress += 2;
	}
	break;
      case 'E':
	uint8_t tmp = 0;
	for(uint8_t i = 0; i < buffSize; i++)
	{
	  tmp = eeprom_read_byte((uint8_t *)gEepromAddress);
	  SPI_Send(tmp);
	  gEepromAddress++;
	}
	break;
      default:
	SPI_Send('?');
	break;
      };
      break;

    case AVR_STARTBLOCKLOAD: // Write block
      buffSize = (SPI_Receive() << 8) | SPI_Send();
      if(buffSize > SPM_PAGESIZE)
      {
	SPI_Send('?');
	break;
      }

      char memType = SPI_Receive();
      switch(memType)
      {
      case 'F':
	if(gFlashAddress > BOOT_START)
	  SPI_Send(0);

	uint16_t tmpAddr = gFlashAddress;
	uint16_t tmp = 0;
	boot_spm_busy_wait();
	for(uint16_t i = 0; i < buffSize/2 ; i++)
	{
	  tmp = SPI_Receive() | (SPI_Receive() << 8);
	  boot_page_fill(tmpAddr, tmp);
	  tmpAddr += 2;
	}

	boot_page_write_safe(gFlashAddress);
	boot_spm_busy_wait();
	boot_rww_enable();
	gFlashAddress = tmpAddr;
	SPI_Send('\r');
	break;
      case 'E':
	uint8_t tmp = 0;
	for(uint16_t i = 0; i < buffSize; i++)
	{
	  tmp = SPI_Receive();
	  eeprom_write_byte((uint8_t *)gEepromAddress, tmp);
	  gEepromAddress++;
	}
	SPI_Send('\r');
	break;
      default:
	SPI_Send('?');
	break;
      };

      break;

    case AVR_SETLED:
      // We have 3 Leds Gree, Yellow, Green
      uint8_t led = SPI_Receive();
      if(led > 3) led = 0;
      if(led == 0) 
	LED_PORT &= ~(1 << LED_GREEN);
      else if(led == 1)
	LED_PORT &= ~(1 << LED_YELLOW);
      else if(led == 2)
	LED_PORT &= ~(1 << LED_RED);
      else
      {
	SPI_Send('?');
	break;
      }
      
      SPI_Send('\r');
      break;

    case AVR_CLEARLED:
      uint8_t led = SPI_Receive();
      if(led > 3) led = 0;
      if(led == 0) 
	LED_PORT |= (1 << LED_GREEN);
      else if(led == 1)
	LED_PORT |= (1 << LED_YELLOW);
      else if(led == 2)
	LED_PORT |= (1 << LED_RED);
      else
      {
	SPI_Send('?');
	break;
      }
      
      SPI_Send('\r');
      break;

    case AVR_READLOWFUSEBITS:
      uint8_t fuses = _SPM_GET_LOW_FUSEBITS();
    case AVR_READHIGHFUSEBITS:
      uint8_t fuses = _SPM_GET_HIGH_FUSEBITS();
    case AVR_READEXTFUSEBITS:
      uint8_t fuses = _SPM_GET_EXTENDED_FUSEBITS();
    case AVR_READLOCKBITS:
      uint8_t fuses = _SPM_GET_LOCKBITS();
      SPI_Send(fuses);
      break;


    // Not Implemented yet
    /*
    case AVR_WRITEPROGLOW:
    case AVR_WRITEPROGHIGH:
    case AVR_PAGEWRITE:
    case AVR_READPROGRAM:
    case AVR_READDATA:
    case AVR_WRITEDATA:
    */

    case AVR_WRITELOCKBITS: // Not supported
    default:
      SPI_Send('?');
      break;
    };
  }

  ExitBootloader();
  return 0;
}

ISR(TIMER2_COMPA_vect)
{
  gMilliseconds++;
}
