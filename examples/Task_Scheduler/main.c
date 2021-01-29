/**
  Simple blink project demonstrating how to use the task scheduler library.

  Task scheduler library is adapted from https://github.com/kcl93/Tasks and is 
  published under MIT license

  supported hardware:
    - Sduino Uno (https://github.com/roybaer/sduino_uno)

  Functionality:
    - run 3 independent tasks in "background":
      - task 1 (long): generate 20ms high pulse ("blocking") on testpin 1. Call every 100ms
      - task 2 (short): toggle testpin 2. Call every 1ms (=min. timeslot)
      - task 3 (short): toggle testpin 3. Call every 1ms. Start/stop periodically
    - in main loop toggle testpin 4 every 1ms to show idle task
**********************/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "config.h"
#include "Tasks.h"


// define testpin 1 PC5 (=Sduino D13/LED)
#define TEST1_PORT   sfr_PORTC
#define TEST1_PIN    PIN5

// define testpin 2 PC7 (=Sduino D12)
#define TEST2_PORT   sfr_PORTC
#define TEST2_PIN    PIN7

// define testpin 3 PC6 (=Sduino D11)
#define TEST3_PORT   sfr_PORTC
#define TEST3_PIN    PIN6

// define testpin 4 PC4 (=Sduino D10)
#define TEST4_PORT   sfr_PORTC
#define TEST4_PIN    PIN4


// first scheduler task: generate 20ms high pulse ("blocking") on testpin 1. Call every 100ms
void toggle_1(void) {

  // set testpin 1
  TEST1_PORT.ODR.byte |= TEST1_PIN;  

  // block for 20ms
  delay(20);

  // clear testpin 1
  TEST1_PORT.ODR.byte &= ~TEST1_PIN;  

} // toggle_1()



// second scheduler task: toggle testpin 2. Call every 1ms (=min. timeslot)
void toggle_2(void) {

  // toggle testpin 2
  TEST2_PORT.ODR.byte ^= TEST2_PIN;  

} // toggle_2()



// third scheduler task: toggle testpin 3. Call every 1ms. Start/stop periodically
void toggle_3(void) {

  // after 20 calls...
  static int count = 0;
  if (++count == 20) {
    count = 0;
    //Tasks_Pause_Task(toggle_3);    // temporarily pause task
    //Tasks_Remove(toggle_3);        // remove task from scheduler
    Tasks_Delay(toggle_3, 200);    // delay next call by 200ms
  }
  
  // toggle testpin 3
  TEST3_PORT.ODR.byte ^= TEST3_PIN;  

} // toggle_3()



/////////////////
//    main routine
/////////////////
void main (void) {

  // disable interrupts
  DISABLE_INTERRUPTS();

  // switch to 16MHz (default is 2MHz)
  sfr_CLK.CKDIVR.byte = 0x00;
    
  // configure testpins as output (all on same port)
  sfr_PORTC.DDR.byte |= (PIN4 | PIN5 | PIN6 | PIN7);    // input(=0) or output(=1)
  sfr_PORTC.CR1.byte |= (PIN4 | PIN5 | PIN6 | PIN7);    // input: 0=float, 1=pull-up; output: 0=open-drain, 1=push-pull
  sfr_PORTC.CR2.byte |= (PIN4 | PIN5 | PIN6 | PIN7);    // input: 0=no exint, 1=exint; output: 0=2MHz slope, 1=10MHz slope


  // init 1ms clock and task scheduler
  Tasks_Init();
  
  // enable interrupts
  ENABLE_INTERRUPTS();   
  

  // add tasks (up to MAX_TASK_CNT)
  Tasks_Add((Task) toggle_1, 150, 0);     // add task 1
  Tasks_Add((Task) toggle_2, 1, 100);     // add task 2 with 100ms initial delay
  Tasks_Add((Task) toggle_3, 1, 0);       // add task 3
  
  // start task scheduler
  Tasks_Start();
  
  // main loop. Toggle testpin 4 every 1ms, other tasks are executed in background by scheduler  
  while(1) {
  
    // toggle testpin 4
    TEST4_PORT.ODR.byte ^= TEST4_PIN;
    
    // wait 1ms
    delay(1);
     
  } // main loop
  
} // main()

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
