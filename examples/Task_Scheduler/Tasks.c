/**
    \file       Tasks.c
    \copybrief  Tasks.h
    \details    For more details please refer to Tasks.h
*/

#include "config.h"      // STM8 selection
#define _TASKS_MAIN_     // for declaring globals
  #include "Tasks.h"
#undef _TASKS_MAIN_

/**************************************/
/******* start skip in doxygen ********/
/**************************************/
/// @cond INTERNAL

// macro to pause / resume interrupt (interrupts are only reactivated in case they have been active in the beginning)
//uint8_t oldISR = 0;
//#define PAUSE_INTERRUPTS    { oldISR = SREG; noInterrupts(); }
//#define RESUME_INTERRUPTS   { SREG = oldISR; interrupts();     }
#define PAUSE_INTERRUPTS    DISABLE_INTERRUPTS()
#define RESUME_INTERRUPTS   ENABLE_INTERRUPTS()


// task container
struct SchedulingStruct
{
    Task    func;       // function to call
    bool    active;     // task is active
    bool    running;    // task is currently being executed
    int16_t period;     // period of task (0 = call only once)
    int16_t time;       // time of next call
};


// global variables for scheduler
struct SchedulingStruct SchedulingTable[MAX_TASK_CNT] = { {(Task)NULL, false, false, 0, 0} }; // array containing all tasks
bool    SchedulingActive;   // false = Scheduling stopped, true = Scheduling active (no configuration allowed)
int16_t _timebase;          // 1ms counter
int16_t _nexttime;          // time of next task call 
uint8_t _lasttask;          // last task in the tasks array (cauting! This variable starts is not counting from 0 to x but from 1 to x meaning that a single tasks will be at SchedulingTable[0] but _lasttask will have the value '1')



/**
  \fn void delay(uint32_t ms)
   
  \brief delay code execution for 'ms'
  
  \param[in]  ms   duration[ms] to halt code
   
  delay code execution for 'ms'. 
  Requires TIM4 interrupt -> is not vulnerable to 
  interrupts (within limits).
  Note: 
    - for ISR-free functions use sw_delay() (uses NOPs)
    - for high accuracy use highRez_delay() (uses HW timer 3)
*/
void delay(uint32_t ms) {

  uint32_t start = micros();

  // wait until time [us] has passed
  ms *= 1000L;
  while (micros() - start < ms)
    NOP();
	
} // delay()



/**
  \fn void delayMicroseconds(uint32_t us)
   
  \brief delay code execution for 'us'
  
  \param[in]  us   duration[us] to halt code
   
  delay code execution for 'us'. 
  Requires TIM4 interrupt -> is not vulnerable to 
  interrupts (within limits).
  Note: 
    - for ISR-free functions use sw_delayMicroseconds() (uses NOPs)
    - for high accuracy use highRez_delayMicroseconds() (uses HW timer 3)
*/
void delayMicroseconds(uint32_t us) {

  uint32_t start = micros();

  // wait until time [us] has passed
  while (micros() - start < us)
    NOP();
	
} // delayMicroseconds()


void Scheduler_update_nexttime(void)
{
    uint8_t i;
		
		// stop interrupts, store old setting
    PAUSE_INTERRUPTS;
    
    // find time of next task execution    
    _nexttime = _timebase + INT16_MAX; // Max. possible delay of the next time
    for (i = 0; i < _lasttask; i++)
    {
        if ((SchedulingTable[i].active == true) && (SchedulingTable[i].func != NULL))
        {
            //Serial.print(i); Serial.print("    "); Serial.println(SchedulingTable[i].time);

            if ((int16_t)(SchedulingTable[i].time - _nexttime) < 0)
            {
                _nexttime = SchedulingTable[i].time;
            }
        }
    }

    //Serial.print("timebase: "); Serial.println(_timebase);
    //Serial.print("nexttime: "); Serial.println(_nexttime);
    //Serial.println();

    //Serial.print(_timebase); Serial.print("    "); Serial.println(_nexttime - _timebase);
    
    // resume stored interrupt setting
    RESUME_INTERRUPTS;

} // Scheduler_update_nexttime()


/// @endcond
/************************************/
/******* end skip in doxygen ********/
/************************************/


void Tasks_Init(void)
{
  // stop the timer
  sfr_TIM4.CR1.CEN = 0;
  
  // for low-power device activate TIM4 clock
  #if defined(FAMILY_STM8L)
    sfr_CLK.PCKENR1.PCKEN12 = 1;
  #endif
   
  // clear counter
  sfr_TIM4.CNTR.byte = 0x00;

  // auto-reload value buffered
  sfr_TIM4.CR1.ARPE = 1;

  // clear pending events
  sfr_TIM4.EGR.byte  = 0x00;

  // set clock to 16Mhz/2^6 = 250kHz -> 4us period
  sfr_TIM4.PSCR.PSC = 6;

  // set autoreload value for 1ms (=250*4us)
  sfr_TIM4.ARR.byte  = 250;

  // enable timer 4 interrupt
  sfr_TIM4.IER.UIE = 1;
  
  // start the timer
  sfr_TIM4.CR1.CEN = 1;

} // Tasks_Init()



void Tasks_Clear(void)
{
    uint8_t i;
    
    // stop interrupts, store old setting
    PAUSE_INTERRUPTS;
    
    // init scheduler
    SchedulingActive = false;
    _timebase = 0;
    _nexttime = 0;
    _lasttask = 0;
    for(i = 0; i < MAX_TASK_CNT; i++)
    {
        //Reset scheduling table
        SchedulingTable[i].func = NULL;
        SchedulingTable[i].active = false;
        SchedulingTable[i].running = false;
        SchedulingTable[i].period = 0;
        SchedulingTable[i].time = 0;
    } // loop over scheduler slots
    
    // resume stored interrupt setting
    RESUME_INTERRUPTS;
    
} // Tasks_Clear()



bool Tasks_Add(Task func, int16_t period, int16_t delay)
{
    uint8_t i;
    
    // Check range of period and delay
    if ((period < 0) || (delay < 0))
        return false;
    
    // Check if task already exists and update it in this case
    for(i = 0; i < _lasttask; i++)
    {
        // stop interrupts when accessing any element within the scheduler (also neccessary for if checks!), store old setting
        PAUSE_INTERRUPTS;

        // same function found
        if (SchedulingTable[i].func == func)
        {
            SchedulingTable[i].active    = true;
            SchedulingTable[i].running = false;
            SchedulingTable[i].period    = period;
            SchedulingTable[i].time        = _timebase + delay;
            
            // resume stored interrupt setting
            RESUME_INTERRUPTS;

            // find time for next task execution
            Scheduler_update_nexttime();

            // return success        
            return true;
        }

        // resume stored interrupt setting
        RESUME_INTERRUPTS;

    } // loop over scheduler slots
    
    // find free scheduler slot
    for (i = 0; i < MAX_TASK_CNT; i++)
    {
        // stop interrupts when accessing any element within the scheduler (also neccessary for if checks!), store old setting
        PAUSE_INTERRUPTS;

        // free slot found    
        if (SchedulingTable[i].func == NULL)
        {
            // add task to scheduler table
            SchedulingTable[i].func        = func;
            SchedulingTable[i].active    = true;
            SchedulingTable[i].running = false;
            SchedulingTable[i].period    = period;
            SchedulingTable[i].time        = _timebase + delay;
            
            // update _lasttask
            if (i >= _lasttask)
                _lasttask = i + 1;

            // resume stored interrupt setting
            RESUME_INTERRUPTS;

            // find time for next task execution
            Scheduler_update_nexttime();

            // return success            
            return true;

        } // if free slot found

        // resume stored interrupt setting
        RESUME_INTERRUPTS;

    } // loop over scheduler slots

    // did not change anything, thus no scheduler_update_nexttime neccessary
    // no free slot found -> error
    return false;

} // Tasks_Add()



bool Tasks_Remove(Task func)
{
    uint8_t i;
    
    // find function in scheduler table
    for (i = 0; i < _lasttask; i++)
    {
        // stop interrupts when accessing any element within the scheduler (also neccessary for if checks!), store old setting
        PAUSE_INTERRUPTS;
    
        // function pointer found in list    
        if (SchedulingTable[i].func == func)
        {
            // remove task from scheduler table
            SchedulingTable[i].func        = NULL;
            SchedulingTable[i].active    = false;
            SchedulingTable[i].running = false;
            SchedulingTable[i].period    = 0;
            SchedulingTable[i].time        = 0;
            
            // update _lasttask
            if (i == (_lasttask - 1))
            {
                _lasttask--;
                while(_lasttask != 0)
                {
                    if(SchedulingTable[_lasttask - 1].func != NULL)
                    {
                        break;
                    }
                    _lasttask--;
                }
            }

            // resume stored interrupt setting
            RESUME_INTERRUPTS;

            // find time for next task execution
            Scheduler_update_nexttime();

            // return success
            return true;

        } // if function found

        // resume stored interrupt setting
        RESUME_INTERRUPTS;

    } // loop over scheduler slots

    // did not change anything, thus no scheduler_update_nexttime neccessary
    // function not in scheduler -> error
    return false;

} // Tasks_Remove()



bool Tasks_Delay(Task func, int16_t delay)
{
    uint8_t i;
    
    // Check range of delay
    if (delay < 0)
        return false;
    
    // find function in scheduler table
    for (i = 0; i < _lasttask; i++)
    {
        // stop interrupts, store old setting
        PAUSE_INTERRUPTS;
        
        // function pointer found in list
        if (SchedulingTable[i].func == func)
        {
            // if task is currently running, delay next call
            if (SchedulingTable[i].running == true)
                SchedulingTable[i].time = SchedulingTable[i].time - SchedulingTable[i].period;
        
            // set time to next execution
            SchedulingTable[i].time = _timebase + delay;

            // resume stored interrupt setting
            RESUME_INTERRUPTS;

            // find time for next task execution
            Scheduler_update_nexttime();

            // return success
            return true;

        } // if function found

        // resume stored interrupt setting
        RESUME_INTERRUPTS;

    } // loop over scheduler slots
    
    // did not change anything, thus no scheduler_update_nexttime neccessary
    // function not in scheduler -> error
    return false;
    
} // Tasks_Delay()



bool Tasks_SetState(Task func, bool state)
{
    uint8_t i;
    
    // find function in scheduler table
    for (i = 0; i < _lasttask; i++)
    {
        // stop interrupts when accessing any element within the scheduler (also neccessary for if checks!), store old setting
        PAUSE_INTERRUPTS;
            
        // function pointer found in list        
        if(SchedulingTable[i].func == func)
        {
            // set new function state            
            SchedulingTable[i].active = state;
            SchedulingTable[i].time = _timebase + SchedulingTable[i].period;

            // resume stored interrupt setting
            RESUME_INTERRUPTS;

            // find time for next task execution
            Scheduler_update_nexttime();

            // return success            
            return true;

        } // if function found
        
        // resume stored interrupt setting
        RESUME_INTERRUPTS;
	
    } // loop over scheduler slots
    
    // did not change anything, thus no scheduler_update_nexttime neccessary
    // function not in scheduler -> error
    return false;
    
} // Tasks_SetState()



void Tasks_Start(void)
{
    // enable scheduler
    SchedulingActive = true;
    //_timebase = 0;        // unwanted delay after resume, see time-print() output! -> likely delete
    
    _nexttime = _timebase;  // Scheduler should perform a full check of all tasks after the next start
    
    // enable timer interrupt
    sfr_TIM4.IER.UIE = 1;

    // find time for next task execution
    Scheduler_update_nexttime();
    
} // Tasks_Start()



void Tasks_Pause(void)
{
    // pause scheduler
    SchedulingActive = false;
    //_timebase = 0; // unwanted delay after resume, see time-print() output! -> likely delete 
    
    // disable timer interrupt
    sfr_TIM4.IER.UIE = 1;

} // Tasks_Pause()



/**************************************/
/******* start skip in doxygen ********/
/**************************************/
/// @cond INTERNAL

#if defined(_TIM4_OVR_UIF_VECTOR_)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_OVR_UIF_VECTOR_)
#elif defined(_TIM4_UIF_VECTOR_)
  ISR_HANDLER(TIM4_UPD_ISR, _TIM4_UIF_VECTOR_)
#else
  #error TIM4 vector undefined
#endif
{
    uint8_t i;
    
    // clear timer 4 interrupt flag
    #if defined(FAMILY_STM8S)
        sfr_TIM4.SR.UIF = 0;
    #else
        sfr_TIM4.SR1.UIF = 0;
    #endif

    // set/increase global variables for millis(), micros() etc.
    g_micros += 1000L;
    g_millis++;
    g_flagMilli = 1;


    // Skip if scheduling was stopped or is in the process of being stopped
    if (SchedulingActive == false) {
        return;
    }
    
    // increase 1ms counter    
    _timebase++;

    // no task is pending -> return immediately
    if ((int16_t)(_nexttime - _timebase) > 0) {
        return;
    }

    // loop over scheduler slots
    for(i = 0; i < _lasttask; i++)
    {
        // disable interrupts
        DISABLE_INTERRUPTS();

        // function pointer found in list, function is active and not running (arguments ordered to provide maximum speed
        if ((SchedulingTable[i].active == true) && (SchedulingTable[i].running == false) && (SchedulingTable[i].func != NULL))
        {
            // function period has passed
            if((int16_t)(SchedulingTable[i].time - _timebase) <= 0)
            {
                // execute task
                SchedulingTable[i].running = true;                                  // avoid dual function call
                SchedulingTable[i].time = _timebase + SchedulingTable[i].period;    // set time of next call
                
                // re-enable interrupts
                ENABLE_INTERRUPTS();

                // execute function
                SchedulingTable[i].func();
                
                // disable interrupts
                DISABLE_INTERRUPTS();
                
                // re-allow function call by scheduler                     
                SchedulingTable[i].running = false;
                
                // if function period is 0, remove it from scheduler after execution                     
                if(SchedulingTable[i].period == 0)
                {
                    SchedulingTable[i].func = NULL;
                }
                
                // re-enable interrupts
                ENABLE_INTERRUPTS();

            } // if function period has passed
        } // if function found
        
        // re-enable interrupts
        ENABLE_INTERRUPTS();
    
    } // loop over scheduler slots

    // find time for next task execution
    Scheduler_update_nexttime();
 
} // ISR()


/// @endcond
/************************************/
/******* end skip in doxygen ********/
/************************************/
