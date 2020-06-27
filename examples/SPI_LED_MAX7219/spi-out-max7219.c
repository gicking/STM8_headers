/**********************
  8 digit 7-segment LED display with MAX7219 controller chip
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    
  setup:
    CS  <-> pin  3 = PD2 = chip select
    DIN <-> pin 11 = PC6 = MOSI
    CLK <-> pin 13 = PC5 = clock (same as builtin LED)
  
  original: https://github.com/jukkas/stm8-sdcc-examples
  
  Functionality:
    - init SPI and LED
    - init TIM1 for interrupt every 1s
    - print seconds to LED display via SPI
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"


/*----------------------------------------------------------
    GLOBAL MACROS
----------------------------------------------------------*/
#define CSN_SELECT    sfr_PORTD.ODR.ODR2 = 0
#define CSN_RELEASE   sfr_PORTD.ODR.ODR2 = 1


void setup_spi(void) {

  // SPI port setup: MISO is pullup in, MOSI & SCK are push-pull out
  sfr_PORTC.DDR.byte |= (uint8_t) (PIN5 | PIN6);     // input(=0) or output(=1)
  sfr_PORTC.CR1.byte |= (uint8_t) (PIN5 | PIN6);     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull

  // CS/SS (PD2) as output
  sfr_PORTD.DDR.DDR2 = 1;     // input(=0) or output(=1)
  sfr_PORTD.CR1.C12  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  CSN_RELEASE;                // CS high

  // SPI registers: First reset everything
  sfr_SPI.CR1.byte = 0x00;
  sfr_SPI.CR2.byte = 0x00;

  // SPI_CR1 LSBFIRST=0 (MSB is transmitted first)
  sfr_SPI.CR1.LSBFIRST = 0;
    
  // Baud Rate Control: 0b111 = fmaster / 256 (62,500 baud)
  sfr_SPI.CR1.BR = 7;
  
  // Clock Phase, The first clock transition is the first data capture edge
  sfr_SPI.CR1.CPOL = 0;
  
  // Clock Polarity, SCK=0 when idle
  sfr_SPI.CR1.CPHA = 0;
  
  sfr_SPI.CR2.SSM  = 1;       // Software slave management, enabled
  sfr_SPI.CR2.SSI  = 1;       // Internal slave select, Master mode
  sfr_SPI.CR1.MSTR = 1;       // Master configuration.

} // setup_spi()



uint8_t SPIOut(uint8_t data) {
    
  sfr_SPI.CR1.MSTR = 1;       // Master device.
  sfr_SPI.CR1.SPE  = 1;       // SPI Enable, Peripheral enabled
  sfr_SPI.DR.byte  = data;    // send 1B
  while (sfr_SPI.SR.BSY);     // wait until SPI not busy
  sfr_SPI.CR1.SPE  = 0;       // Disable SPI
  data = sfr_SPI.DR.byte;     // read data
  return data;                // Not yet used.
    
} // SPIOut()



void output_max(uint8_t address, uint8_t data) {
  CSN_SELECT;                 // CSN low
  SPIOut(address);            // send address
  SPIOut(data);               // send data
  CSN_RELEASE;                // CSN high
} // output_max()



void init_max7219(void) {

  uint8_t i;

  output_max(0x0f, 0x00);     // display test register - test mode off
  output_max(0x0c, 0x01);     // shutdown register - normal operation
  output_max(0x0b, 0x07);     // scan limit register - display digits 0 thru 7
  output_max(0x0a, 0x01);     // intensity register (1 = 3/32 on.  0xf is max)
  output_max(0x09, 0xff);     // decode mode register - CodeB decode all digits

  // Blank all digits
  for (i=1; i <= 8; i++) {
    output_max(i, 0xf);
  }

} // init_max7219()



void display_number(uint32_t number) {
  uint8_t pos=1;
  uint8_t digit;
  
  if (number == 0)
    output_max(pos++, 0);
  
  while (number > 0) {
    digit = number % 10;
    output_max(pos++, digit);
    number /= 10;
  }
  
  // clear rest of digits
  while (pos <= 8) {
    output_max(pos++, 0xf);
  }
  
} // display_number()



ISR_HANDLER(TIM1_UPD_ISR, _TIM1_OVR_UIF_VECTOR_)
{
  // clear timer interrupt flag
  sfr_TIM1.SR1.UIF = 0;
  
} // TIM1_UPD_ISR



int main(void)
{
  uint32_t counter = 0;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // setup SPI and LED
  setup_spi();
  init_max7219();

  // TIM1 setup: generate interrupt every 1000ms
  sfr_TIM1.CR1.byte   = 0x00;
  sfr_TIM1.CR1.DIR    = 1;       // Just for fun, let's count down (TIM1 can do that)
  sfr_TIM1.PSCRH.byte = 0x0E;    // Prescaler: divide clock with 16000 (0x3E7F + 1) (to 1ms)
  sfr_TIM1.PSCRL.byte = 0x7F;
  sfr_TIM1.ARRH.byte  = 0x13;    // Auto-reload registers. Count to 1000 (0x03E8)
  sfr_TIM1.ARRL.byte  = 0xE8;
  sfr_TIM1.IER.UIE    = 1;       // Update interrupt (UIE)
  sfr_TIM1.CR1.CEN    = 1;       // Start TIM1
  
  // enable interrupts
  ENABLE_INTERRUPTS();   

  // Loop infinitely waiting for an interrupt
  while(1) {
    
    WAIT_FOR_INTERRUPT();
    display_number(counter++);
    //sfr_PORTC.ODR.ODR5 ^= 1;     // debug: toggle LED
          
    // In case we are running this for years :-)
    if (counter > 99999999)
      counter = 0;
      
  } // while(1)
    
} // main()
