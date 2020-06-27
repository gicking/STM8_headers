/**********************
  Read temperature from DS18B20 sensor via 1-wire interface and 
  display on 8 digit 7-segment LED display with MAX7219 controller chip
  
  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    
  setup:
    DS18B20: 
      Supply with 3.3V! 
      DATA <-> to pin 5 = PD4
      4.7kohm resistor b/w PD4 and 3.3V.
    MAX7219:
      CS  <-> pin  3 = PD2 = chip select
      DIN <-> pin 11 = PC6 = MOSI
      CLK <-> pin 13 = PC5 = clock (same as builtin LED)
  
  original: https://github.com/jukkas/stm8-sdcc-examples
  
  Functionality:
    - init init SPI and LED
    - init TIM1 for interrupt every 1s
    - print seconds to LED display via SPI
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"

/* SPI CSN pin macros */
#define CSN_SELECT    sfr_PORTD.ODR.ODR2 = 0
#define CSN_RELEASE   sfr_PORTD.ODR.ODR2 = 1

/* 1-Wire (DS18B20 data) pin macros */
#define OW_INPUT_MODE()     sfr_PORTC.DDR.DDR4 = 0 
#define OW_OUTPUT_MODE()    sfr_PORTC.DDR.DDR4 = 1
#define OW_LOW()            sfr_PORTD.ODR.ODR4 = 0
#define OW_HIGH()           sfr_PORTD.ODR.ODR4 = 1
#define OW_READ()           (sfr_PORTD.IDR.IDR4)


// Simple busy loop delay
void delay(uint32_t count) {
  while (count--)
    NOP();
} // delay()


/********************** For LED-display ***************************/

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



void display_number_dot(uint32_t number, uint8_t dot_pos, uint8_t is_negative) {
  uint8_t pos=1;
  uint8_t digit;
  
  if (number == 0)
    output_max(pos++, 0);
  
  while ((number > 0) || (dot_pos >= pos)) {
    digit = number % 10;
    if (pos == dot_pos) {
      digit = digit | 0x80;
    }
    output_max(pos++, digit);
    number /= 10;
  }
  if (is_negative) {
      output_max(pos++, 0xa);
  }

  // clear rest of digits
  while (pos <= 8) {
    output_max(pos++, 0xf);
  }
  
} // display_number()


/********************** OneWire/DS18B20 routines ***************************/

void delay_us(uint16_t i) {
  volatile uint16_t counter;
    
  if (i < 9) { // FIXME: Really ugly
    NOP();
    return;
  }
  sfr_TIM2.CNTRH.byte = 0;
  sfr_TIM2.CNTRL.byte = 0;
  sfr_TIM2.EGR.UG     = 1;      // Update Generation
  while(1) {
    counter = (((sfr_TIM2.CNTRH.byte) << 8) | sfr_TIM2.CNTRL.byte);
    if (i-6 < counter)
      return;
  }
    
} // delay_us()


void ow_pull_low(unsigned int us) {
  OW_OUTPUT_MODE();
  OW_LOW();
  delay_us(us);
  OW_INPUT_MODE();
} // ow_pull_low()


void ow_write_byte(uint8_t out) {
  uint8_t i;
  for (i=0; i < 8; i++) {
    if ( out & ((uint8_t)1<<i) ) {
      // write 1
      ow_pull_low(1);
      delay_us(60);
    } 
    else {
      // write 0
      ow_pull_low(60);
      delay_us(1);
    }
  }
} // ow_write_byte()


uint8_t ow_read_byte(void) {
  uint8_t val = 0;
  uint8_t i;
  for (i=0; i < 8; i++) {
    ow_pull_low(1);
    delay_us(5);
    if (OW_READ()) {
      val |= ((uint8_t)1<<i);
    }
    delay_us(55);
  }
  return val;

} // ow_read_byte()


uint8_t ow_init(void) {

  uint8_t input;

  ow_pull_low(480);
  delay_us(60);

  input = !OW_READ();
  delay_us(420);

  return input;

} // ow_init()


unsigned int ow_convert_temperature(void) {
  int cycles = 1; // For debugging purposes

  ow_write_byte(0x44); // Convert Temperature

  while (1) {
    ow_pull_low(1);
    delay_us(5);
    if (OW_READ()) {
      return cycles;
    }
    delay_us(55);
    cycles++;
  }
  
} // ow_convert_temperature()


void display_ds_temperature(uint8_t high, uint8_t low) {
  uint8_t is_negative = 0;
  uint16_t decimals = 0; // 4 decimals (e.g. decimals 625 means 0.0625)
  uint16_t i;

  uint16_t temp = ((int16_t)high << 8) | low;
  if (temp & 0x8000) {
    is_negative = 1;
    temp = (~temp) + 1;
  }
  low = temp & 0x0f;
  temp = temp >> 4;

  // low[3:0] mean values 0.5,0.25,0.125 and 0.0625
  for (i=625; i <= 5000; i=i*2) {
    if (low & 0x01) {
      decimals += i;
    }
    low = low >> 1;
  }

  // Display temperature rounded to one decimal
  display_number_dot((temp*1000 + ((decimals+5)/10) + 50)/100, 2, is_negative);

} // display_ds_temperature()


void read_ds18b20(void) {
  uint8_t i;
  uint8_t scratchpad[9];

  if (ow_init()) {
    ow_write_byte(0xcc); // Skip ROM
    ow_convert_temperature();

    ow_init();
    ow_write_byte(0xcc); // Skip ROM
    ow_write_byte(0xbe); // Read Scratchpad
    for (i=0; i<9; i++) {
      scratchpad[i] = ow_read_byte();
    }

    display_ds_temperature(scratchpad[1], scratchpad[0]);

  } 
  else {
    /* DS18B20 was not detected */
    output_max(0x8, 0xa);
  }
  
} // read_ds18b20()


/***************************************************************************/


int main(void)
{
  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // Timer setup (for delay_us)
  sfr_TIM2.PSCR.byte = 0x4;      // Prescaler: to 1MHz
  sfr_TIM2.CR1.CEN   = 1;        // Start timer
    
  // setup SPI and LED
  setup_spi();
  init_max7219();

  // periodically read temperature and display
  while(1) {
    read_ds18b20();
    delay(10000L);
  }

} // main()
