/**********************
  Simple blink project w/o interrupts
   
  supported hardware:
    - STM8S Discovery board (https://www.st.com/en/evaluation-tools/stm8s-discovery.html)
    - STM8L Discovery board (https://www.st.com/en/evaluation-tools/stm8l-discovery.html)
    - NUCLEO-8S207K8 (https://www.st.com/en/evaluation-tools/nucleo-8s207k8.html)
    - NUCLEO-8S208RB (https://www.st.com/en/evaluation-tools/nucleo-8s208rb.html)
    - STM8-SO8 STM8S001 Discovery board (https://www.st.com/en/evaluation-tools/stm8-so8-disco.html)
    - STM8-SO8 STM8L001 Discovery board (https://www.st.com/en/evaluation-tools/stm8-so8-disco.html)
    - STM8-SO8 STM8S050 Discovery board (https://www.st.com/en/evaluation-tools/stm8-so8-disco.html)
    - Sduino Uno (https://github.com/roybaer/sduino_uno)
    - muBoard (http://www.cream-tea.de/presentations/160305_PiAndMore.pdf)
  
  Functionality:
    - blink LED w/o ISR. Mainly for testing toolchain
    - pass port structure to function
**********************/

/*----------------------------------------------------------
    INCLUDE FILES / MACROS
----------------------------------------------------------*/

// STM8 device header & LED pin
#if defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"
  #define LED_PORT   sfr_PORTD
  #define LED_PIN    PIN0
#elif defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN7
#elif defined(NUCLEO_8S207K8)
  #include "../../include/STM8S207K8.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(NUCLEO_8S208RB)
  #include "../../include/STM8S208RB.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(STM8_SO8_DISCO_STM8S001)
  #include "../../include/STM8S001J3.h"
  #define LED_PORT   sfr_PORTA
  #define LED_PIN    PIN3
#elif defined(STM8_SO8_DISCO_STM8L001)
  #include "../../include/STM8L001J3.h"
  #define LED_PORT   sfr_PORTB
  #define LED_PIN    PIN3
#elif defined(STM8_SO8_DISCO_STM8L050)
  #include "../../include/STM8L050J3.h"
  #define LED_PORT   sfr_PORTB
  #define LED_PIN    PIN3
#elif defined(SDUINO)
  #include "../../include/STM8S105K6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(MUBOARD)
  #include "../../include/STM8S207MB.h"
  #define LED_PORT   sfr_PORTH
  #define LED_PIN    PIN2
#else
  #error undefined board
#endif

// required for global variables
#define _MAIN_
  // no modules with globals required
#undef _MAIN_


/*----------------------------------------------------------
    GLOBAL FUNCTIONS
----------------------------------------------------------*/

// toggle specified pin. Pass port struct as pointer
void toggle_pin(PORT_t *port, uint8_t pin) {
  
  port->ODR.byte ^= pin;
  
} // toggle_pin



/////////////////
//    main routine
/////////////////
void main (void) {

  uint32_t  i;
      
  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // configure LED pin as output
  LED_PORT.DDR.byte |= LED_PIN;    // input(=0) or output(=1)
  LED_PORT.CR1.byte |= LED_PIN;    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  LED_PORT.CR2.byte |= LED_PIN;    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope
  

  // main loop
  while(1) {
	
    // toggle LED. Pass port struct as pointer
    toggle_pin(&LED_PORT, LED_PIN);
    
    // simple wait ~500ms @ 16MHz
    for (i=0; i<300000L; i++)
      NOP();
    
  } // main loop

} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
