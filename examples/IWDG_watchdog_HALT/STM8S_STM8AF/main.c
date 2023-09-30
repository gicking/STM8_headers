/**********************
  Test independent watchdog IWDG
  
  supported hardware:
    - muDuino (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
    - NUCLEO-8S207K8 (https://www.st.com/en/evaluation-tools/nucleo-8s207k8.html)
    - NUCLEO-8S208RB (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)

  Functionality:
    - configure wake pin as input pull-up with interrupt on falling edge
    - initialize IWDG to 100ms, service every 50ms
    - print millis to UART every 500ms
    - if 'h' received, enter HALT mode. Wake periodically to service IWDG
    - on wake via wake pin=low, resume normal operation

  Note:
    - STM8S/AF cannot stop IWDG in HALT mode, see https://community.st.com/t5/stm8-mcus/entering-halt-mode-with-the-iwdg-enabled-causes-reset-for/td-p/538801
    - STM8L/AL has option to stop IWDG in HALT mode
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "config.h"
#define _MAIN_          // required for global variables
  #include "uart.h"
  #include "timer4.h"
  #include "iwdg.h"
  #include "awu.h"
  #include "power_saving.h"
#undef _MAIN_


/*----------------------------------------------------------
    LOCAL MACROS
----------------------------------------------------------*/
#define PERIOD_IWDG     100     // IWDG timeout period [ms]  
#define SCHEDULE_IWDG    50     // scheduler period [ms] for IWDG service  
#define SCHEDULE_PRINT  500     // scheduler period for printing time


/*----------------------------------------------------------
    GLOBAL VARIABLES
----------------------------------------------------------*/
volatile uint8_t  flagPinWake = 0;

/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void WakePin_ISR(void)
   
  \brief ISR for wake pin
   
  interrupt service routine for wake pin

  Note: 
    SDCC: ISR must be declared in file containing main(). Header inclusion is ok
    Cosmic: interrupt service table is defined in file "stm8_interrupt_vector.c"
*/
ISR_HANDLER(WakePin_ISR, EXTI_VECTOR)
{
  flagPinWake = 1;

  // toggle LED state (debug only)
  //ODR_LED ^= 1;

} // WakePin_ISR


/**
  \fn int putchar(int byte)
   
  \brief output routine for printf()
  
  \param[in]  data   byte to send
  
  \return  sent byte

  implementation of putchar() for printf(), using selected output channel.
  Use send routine set via putchar_attach()
  Return type depends on used compiler (see respective stdio.h)
*/
#if defined(__CSMC__)
  char putchar(char data) {
#else // Standard C
  int putchar(int data) {
#endif

  // send byte
  UART_write(data);
  
  // return sent byte
  return(data);
  
} // putchar



/////////////////
//    main routine
/////////////////
void main (void) {

  uint32_t  millis;
  uint32_t  lastIWDG = 0;
  uint32_t  lastPrint = 0;

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // init UART2 for 19.2kBaud
  UART_begin(19200);
  

  // configure GPIOs
  #if defined(MUBOARD)
    
    // configure button pin (=PE5) as input pull-up with interrupt
    sfr_PORTE.DDR.DDR5 = 0;     // input(=0) or output(=1)
    sfr_PORTE.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTE.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    sfr_ITC.CR2.PEIS   = 2;     // edge sensitivity for port (all 8 pins): falling

    // configure LED pins PH2+PH3 as output
    sfr_PORTH.DDR.byte = (1 << 2) | (1 << 3);     // input(=0) or output(=1)
    sfr_PORTH.CR1.byte = (1 << 2) | (1 << 3);     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTH.CR2.byte = (1 << 2) | (1 << 3);     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  #elif defined(NUCLEO_8S207K8)
    
    // configure wake pin D2 (=PD0) as input pull-up with interrupt
    sfr_PORTD.DDR.DDR0 = 0;     // input(=0) or output(=1)
    sfr_PORTD.CR1.C10  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTD.CR2.C20  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    sfr_ITC.CR1.PDIS   = 2;     // edge sensitivity for port (all 8 pins): falling

    // configure LED pin (=PC5) as output
    sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
    sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope

  #elif defined(NUCLEO_8S208RB) // wake via button = PE4
    
    // configure button pin (=PE4) as input pull-up with interrupt
    sfr_PORTE.DDR.DDR4 = 0;     // input(=0) or output(=1)
    sfr_PORTE.CR1.C14  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTE.CR2.C24  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
    sfr_ITC.CR2.PEIS   = 2;     // edge sensitivity for port (all 8 pins): falling

    // configure LED pin (=PC5) as output
    sfr_PORTC.DDR.DDR5 = 1;     // input(=0) or output(=1)
    sfr_PORTC.CR1.C15  = 1;     // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
    sfr_PORTC.CR2.C25  = 1;     // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope    

  #else
    #error undefined board
  #endif


  // init 1ms interrupt
  TIM4_init();
  
  // start IWDG watchdog and set timeout period
  iwdg_init(PERIOD_IWDG);

  // enable interrupts
  ENABLE_INTERRUPTS();   
  
  // print instruction  
  #if defined(NUCLEO_8S207K8)
    printf("\npress 'h' to enter HALT mode, wake via PD0/D2 = low\n\n");
  #elif defined(NUCLEO_8S208RB)
    printf("\npress 'h' to enter HALT mode, wake via button\n\n");
  #endif
    
  // main loop
  while(1) {
  
    // avoid race condition
    DISABLE_INTERRUPTS();
    millis = g_millis;
    ENABLE_INTERRUPTS();
    
    // service IWDG every 50ms
    if (millis - lastIWDG > SCHEDULE_IWDG) {
      lastIWDG = millis;
      
      // service IWDG
      iwdg_service();
      
    } // 50ms task
    

    // print time every 500ms
    if (millis - lastPrint > SCHEDULE_PRINT) {
      lastPrint = millis;
      
      // toggle LED
      ODR_LED ^= 1;
    
      // print millis
      printf("  time: %ld\n", millis);
      
    } // 500ms task
    

    // received a byte via UART (stored in receive ISR)
    if (g_key) {
      
      // on 'h' enter HALT mode with periodic IWDG refresh
      if (g_key == 'h') {
        printf("enter HALT ... ");
        
        // reset flag for pin wake
        flagPinWake = 0;

        // change IWDG period to max. 1s
        iwdg_set_period(0x06, 0xFF);
        iwdg_service();

        // halt SW clock (TIM4)
        sfr_TIM4.CR1.CEN = 0;

        // enter HALT mode with periodic IWDG refresh
        do
        {
          // enter HALT mode with wake via AWU or EXINT
          // Note: IWDG and AWU run on LSI -> no clock mismatch
          lowPower_HaltAWU(950);
          
          // service IWDG
          iwdg_service();

          // toggle LED in AWU ISR

        } while (!flagPinWake);
        
        // resume SW clock (TIM4)
        sfr_TIM4.CR1.CEN = 1;

        // restore IWDG period [ms] 
        iwdg_set_period(0x04, PERIOD_IWDG);
        iwdg_service();
        
        // print to console
        printf("resume\n");
        
      } // enter HALT mode
      
      // clear key
      g_key = 0;

    } // byte received    
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
