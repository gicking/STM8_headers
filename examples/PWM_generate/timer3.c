/**
  \file timer3.c
   
  \brief implementation of TIM3 functions/macros 
   
  supported hardware:
    - Sduino

  implementation of functions for PWM generation via TIM3
*/

/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#include "timer3.h"


/// scale a number from one range to another (like map() but with 0 offsets)
#define scale(x,inMax,outMax)      (((int32_t)x*(int32_t)outMax)/inMax)


/*----------------------------------------------------------
    FUNCTIONS
----------------------------------------------------------*/

/**
  \fn void TIM3_init(void)
   
  \brief init timer 3
   
  init timer TIM3, i.e. reset to default values
*/
void TIM3_init(void) {

  // (mainly) stop timer
  sfr_TIM3.CR1.byte = sfr_TIM3_CR1_RESET_VALUE;

  // disable TIM3 interrupts
  sfr_TIM3.IER.byte = sfr_TIM3_IER_RESET_VALUE;
  
  // reset status registers
  sfr_TIM3.SR1.byte = sfr_TIM3_SR1_RESET_VALUE;
  sfr_TIM3.SR2.byte = sfr_TIM3_SR2_RESET_VALUE;

  // reset event generation register
  sfr_TIM3.EGR.byte = sfr_TIM3_EGR_RESET_VALUE;

  // reset PWM mode
  sfr_TIM3.CCMR1.byte = sfr_TIM3_CCMR1_RESET_VALUE;
  sfr_TIM3.CCMR2.byte = sfr_TIM3_CCMR2_RESET_VALUE;
  
  // reset capture/compare enable register
  sfr_TIM3.CCER1.byte = sfr_TIM3_CCER1_RESET_VALUE;

  // reset counter register (write high byte first)
  sfr_TIM3.CNTRH.byte = sfr_TIM3_CNTRH_RESET_VALUE;
  sfr_TIM3.CNTRL.byte = sfr_TIM3_CNTRL_RESET_VALUE;
  
  // set prescaler to fclk/2^4
  sfr_TIM3.PSCR.PSC = 4;
  
  // set max period
  sfr_TIM3.ARRH.byte = sfr_TIM3_ARRH_RESET_VALUE;
  sfr_TIM3.ARRL.byte = sfr_TIM3_ARRL_RESET_VALUE;

  // reset duty cycles
  sfr_TIM3.CCR1H.byte = sfr_TIM3_CCR1H_RESET_VALUE;
  sfr_TIM3.CCR1L.byte = sfr_TIM3_CCR1L_RESET_VALUE;
  sfr_TIM3.CCR2H.byte = sfr_TIM3_CCR2H_RESET_VALUE;
  sfr_TIM3.CCR2L.byte = sfr_TIM3_CCR2L_RESET_VALUE;
  
  // request register update
  sfr_TIM3.EGR.UG = 1;

} // TIM3_init



/**
  \fn void TIM3_setFrequency(uint32_t centHz)

  \brief set PWM frequency for all channels
   
  \param[in] centHz   PWM frequency [0.01Hz]
   
  Set PWM frequency in 0.01Hz for all TIM3 compare channels. 
*/
void TIM3_setFrequency(uint32_t centHz) {

  uint8_t     pre;          // 16b timer prescaler
  uint16_t    ARR;          // 16b reload value
  uint16_t    tmp;


  //////////////
  // calculate timer parameter
  //////////////

  // find smallest usable prescaler
  tmp = (uint16_t)(1600000000L / (centHz * UINT16_MAX));
  pre = 0;
  while (tmp >= (0x0001 << pre))
    pre++;
	
  // set period to spec. value (fPWM = fCPU/((2^pre)*(ARR+1))
  ARR = (uint16_t) ((1600000000L >> pre) / (uint32_t) centHz) - 1;


  //////////////
  // set PWM period
  //////////////

  // reset timer registers (just to make sure)
  TIM3_init();
	
  // exit on zero or too low frequency. Above init already resets timer
  if ((centHz == 0) || (pre > 15))
    return;
   
    
  ////
  // all channels
  ////

  // use buffered period register to avoid glitches
  sfr_TIM3.CR1.ARPE = 1;
  
  // set TIM3 prescaler fTim = fcpu/2^pre with pre in [0..15]
  sfr_TIM3.PSCR.PSC = pre;

  // set reload period = (ARR+1)/fTim
  sfr_TIM3.ARRH.byte = (uint8_t) (ARR >> 8);
  sfr_TIM3.ARRL.byte = (uint8_t) ARR;
  

  ////
  // channel 1
  ////

  // use timer in output compare mode
  sfr_TIM3.CCMR1.CC1S  = 0;

  // set PWM mode 1
  sfr_TIM3.CCMR1.OC1M  = 6;

  // use buffered compare register to avoid glitches
  sfr_TIM3.CCMR1.OC1PE = 1;
  
  // set active polarity to high
  sfr_TIM3.CCER1.CC2P  = 0;
  

  ////
  // channel 2
  ////

  // use timer in output compare mode
  sfr_TIM3.CCMR2.CC2S  = 0;

  // set PWM mode 1
  sfr_TIM3.CCMR2.OC2M  = 6;

  // use buffered compare register to avoid glitches
  sfr_TIM3.CCMR2.OC2PE = 1;
  
  // set active polarity to high
  sfr_TIM3.CCER1.CC2P     = 0;


  // request register update
  sfr_TIM3.EGR.UG = 1;
  
  // activate timer
  sfr_TIM3.CR1.CEN = 1;

} // TIM3_setFrequency



/**
  \fn void TIM3_setDutyCycle(uint8_t channel, uint16_t deciPrc)

  \brief set PWM duty cycle for single compare channel
   
  \param[in] channel   compare channel
  \param[in] deciPrc   PWM duty cycle in [0.1%]
   
  Set PWM duty cycle in 0.1% for single compare channel. 
*/
void TIM3_setDutyCycle(uint8_t channel, uint16_t deciPrc) {

  uint16_t   CCR, ARR;
  
  // get reload value (=PWM period)
  ARR = ((uint16_t) (sfr_TIM3.ARRH.byte)) << 8 | (uint16_t) (sfr_TIM3.ARRL.byte);
  
  // map duty cycle [0.1%] to reload period
  if (deciPrc >= 1000)
    CCR = ARR+1;
  else
    CCR = scale(deciPrc, 1000, ARR);
  
  // set capture/compare value (DC=CCR/ARR) and enable output
  if (channel == 1) {
    sfr_TIM3.CCR1H.byte = (uint8_t) (CCR >> 8);
    sfr_TIM3.CCR1L.byte = (uint8_t) CCR;
    sfr_TIM3.CCER1.CC1E = 1;
  }
  else if (channel == 2) {
    sfr_TIM3.CCR2H.byte = (uint8_t) (CCR >> 8);
    sfr_TIM3.CCR2L.byte = (uint8_t) CCR;
    sfr_TIM3.CCER1.CC2E = 1;
  }
  
} // TIM3_setDutyCycle


/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
