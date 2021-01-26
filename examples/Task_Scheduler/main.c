/**
  Simple blink project demonstrating how to use the task scheduler library.

  Task scheduler library is adapted from https://github.com/kcl93/Tasks and is 
  published under MIT license

  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)

  Functionality:
    - toggle 2 testpins in 2 independent tasks
    - 
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"
#include "Tasks.h"


// define first testpin PC5 (=Sduino D13/LED)
#define TEST1_PORT   sfr_PORTC
#define TEST1_PIN    PIN5

// define second testpin PC7 (=Sduino D12)
#define TEST2_PORT   sfr_PORTC
#define TEST2_PIN    PIN7


// first scheduler task
void toggle_1(void) {

  // toggle testpin 1
  TEST1_PORT.ODR.byte ^= TEST1_PIN;  

} // toggle_1



// second scheduler task
void toggle_2(void) {

  // after 20 calls...
  static int count = 0;
  if (++count == 20) {
    count = 0;
    //Tasks_Pause_Task(toggle_2);    // temporarily pause task
    //Tasks_Remove(toggle_2);         // remove task from scheduler
    Tasks_Delay (toggle_2, 2000);   // delay next call by 2s
  }
  
  // toggle testpin 2
  TEST2_PORT.ODR.byte ^= TEST2_PIN;  

} // toggle_2



/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // configure testpins as output
  sfr_PORTC.DDR.byte |= (PIN5 | PIN7);    // input(=0) or output(=1)
  sfr_PORTC.CR1.byte |= (PIN5 | PIN7);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.byte |= (PIN5 | PIN7);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope


  // init 1ms clock and task scheduler
  Tasks_Init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  

  // add tasks (up to MAX_TASK_CNT)
  Tasks_Add((Task) toggle_1, 500, 0);     // toggle pin 1 every 500ms without delay
  Tasks_Add((Task) toggle_2, 100, 2000);  // toggle pin 2 every 100ms with 2s delay for 1st call
  
  // start task scheduler
  Tasks_Start();
  
  // dummy main loop. Tasks are executed in background by scheduler  
  while(1);
  
} // main

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
