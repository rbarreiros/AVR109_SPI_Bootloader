
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

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
bool gInProgramming = false;

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

int main(void)
{
  byte data;

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
    if(data == EXEC_PROG)
      gInProgramming = true;


  }

  ExitBootloader();
  return 0;
}

ISR(TIMER2_COMPA_vect)
{
  gMilliseconds++;
}
