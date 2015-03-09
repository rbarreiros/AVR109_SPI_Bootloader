
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr109.h>

#define VERSION_H '1'
#define VERSION_L '0'

// Util types
typedef uint8_t byte;
typedef uint8_t bool;
#define true  1
#define false 0

#define PROG_START   0x0000
#define BOOT_TIMEOUT 4000 // 4 seconds
#define SPI_TIMEOUT  4000 // 4 seconds

//#ifndef BOOT_START
//#error "Makefile needs BOOTADDR Defined"
//#endif

#define RELOAD_1MSEC (((F_CPU/1000)/256) - 1)

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

// Global

static volatile unsigned long long gMilliseconds;
static volatile unsigned long long temp = 0;
static unsigned long long lastSpiTimeout = 0;
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
      if(gMilliseconds > SPI_TIMEOUT) return 0x00;
  }

  return SPDR;
}

void SPI_Send(byte data)
{
  SPDR = data;
  while(!(SPSR & (1<<SPIF)))
  {
    if(gMilliseconds - lastSpiTimeout > SPI_TIMEOUT)
    {
      lastSpiTimeout = gMilliseconds;
      return;
    }
  }
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
  TCCR1B |= (1 << WGM12) | (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  OCR1A = (((F_CPU/1000) / 256) - 1);
}

int main(void)
{
  byte data, led, fuses;
  uint16_t buffSize;
  char memType;

  // Disable Interrupts and move IVT
  cli();
  MCUCR = (1 << IVCE);
  MCUCR = (1 << IVSEL);

  // Watchdog
  MCUSR = 0;
  wdt_disable();

  // Setup Board LEDS
  SetupLeds();

  // Setup the 1ms Timer
  SetupTimer();
  
  // Setup SPI
  SetupSPI();

  // Turn Yellow ON Signal wait for data
  /*
  LED_PORT &= ~(1 << LED_RED);
  _delay_ms(500);
  LED_PORT |= (1 << LED_RED);
  LED_PORT &= ~(1 << LED_GREEN);
  */

  // Enable Interrupts
  sei();

  while(1)
  {
    _delay_us(1);
    
    if(!gInProgramming)
      if(gMilliseconds > BOOT_TIMEOUT) break;

    data = SPI_Receive();
    if(data == 0x00 && !gInProgramming) break;
    
    switch(data)
    {
    case AVR_ENTERPROGRAMMING:  // Enter programming, suspend boot timeout
      LED_PORT &= ~(1 << LED_YELLOW);
      gInProgramming = true;
      SPI_Send('\r');
      break;
    case AVR_LEAVEPROGRAMMING:  // Leave programming return to application
    case AVR_EXITBOOTLOADER:
      LED_PORT &= ~(1 << LED_GREEN);
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
      memType = SPI_Receive();
      if(memType == 'F')
      {
	uint16_t tmp = 0;
	for(uint16_t i = 0; i < buffSize; i += 2)
	{
	  tmp = pgm_read_word(gFlashAddress);
	  SPI_Send(LOW(tmp));
	  SPI_Send(HIGH(tmp));
	  gFlashAddress += 2;
	}
      }
      else if(memType == 'E')
      {
	uint8_t tmp = 0;
	for(uint8_t i = 0; i < buffSize; i++)
	{
	  tmp = eeprom_read_byte((uint8_t *)gEepromAddress);
	  SPI_Send(tmp);
	  gEepromAddress++;
	}
      }
      else
	SPI_Send('?');

      break;

    case AVR_STARTBLOCKLOAD: // Write block
      buffSize = (SPI_Receive() << 8) | SPI_Receive();
      if(buffSize > SPM_PAGESIZE)
      {
	SPI_Send('?');
	break;
      }

      memType = SPI_Receive();
      if(memType == 'F')
      {
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
      }
      else if(memType == 'E')
      {
	uint8_t tmp = 0;
	for(uint16_t i = 0; i < buffSize; i++)
	{
	  tmp = SPI_Receive();
	  eeprom_write_byte((uint8_t *)gEepromAddress, tmp);
	  gEepromAddress++;
	}
	SPI_Send('\r');
      }
      else
	SPI_Send('?');

      break;

    case AVR_SETLED:
      // We have 3 Leds Gree, Yellow, Green
      led = SPI_Receive();
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
      led = SPI_Receive();
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

    /*
    case AVR_READLOWFUSEBITS:
      fuses = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
      SPI_Send(fuses);
      break;
    case AVR_READHIGHFUSEBITS:
      fuses = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
      SPI_Send(fuses);
      break;
    case AVR_READEXTFUSEBITS:
      fuses = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
      SPI_Send(fuses);
      break;
    case AVR_READLOCKBITS:
      fuses = boot_lock_fuse_bits_get(GET_LOCK_BITS);
      SPI_Send(fuses);
      break;

    // Not Implemented yet
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

ISR(TIMER1_COMPA_vect)
{
  if(gMilliseconds - temp > 500)
    {
      LED_PORT ^= (1 << LED_RED);
      temp = gMilliseconds;
    }

  gMilliseconds++;
}
