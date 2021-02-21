/*-------------------------------------------------------------------------

  STLUX385A.h - Device Declarations

  STLUX without ROM bootloader

  Copyright (C) 2020, Georg Icking-Konert

  Digital controllers for lighting and power conversion applications with up to 6 programmable PWM generators, 96 MHz PLL, DALI 

  datasheet: https://www.st.com/resource/en/datasheet/stlux385a.pdf
  reference: check manually 
  MIT License

  Copyright (c) 2020 Georg Icking-Konert

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-------------------------------------------------------------------------*/
#ifndef STLUX385A_H
#define STLUX385A_H

// DEVICE NAME
#define DEVICE_STLUX385A

// DEVICE FAMILY
#define FAMILY_STLUX

// required for C++
#ifdef __cplusplus
  extern "C" {
#endif


/*-------------------------------------------------------------------------
  INCLUDE FILES
-------------------------------------------------------------------------*/
#include <stdint.h>


/*-------------------------------------------------------------------------
  COMPILER SPECIFIC SETTINGS
-------------------------------------------------------------------------*/

// Cosmic compiler
#if defined(__CSMC__)

  // macros to unify ISR declaration and implementation
  #define ISR_HANDLER(func,irq)  @far @interrupt void func(void)      ///< handler for interrupt service routine
  #define ISR_HANDLER_TRAP(func) void @far @interrupt func(void)      ///< handler for trap service routine

  // definition of inline functions
  #define INLINE                 @inline                              ///< keyword for inline functions

  // common assembler instructions
  #define NOP()                  _asm("nop")                          ///< perform a nop() operation (=minimum delay)
  #define DISABLE_INTERRUPTS()   _asm("sim")                          ///< disable interrupt handling
  #define ENABLE_INTERRUPTS()    _asm("rim")                          ///< enable interrupt handling
  #define TRIGGER_TRAP           _asm("trap")                         ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
  #define WAIT_FOR_INTERRUPT()   _asm("wfi")                          ///< stop code execution and wait for interrupt
  #define ENTER_HALT()           _asm("halt")                         ///< put controller to HALT mode
  #define SW_RESET()             _asm("dc.b $75")                     ///< reset via illegal opcode (works for all devices)

  // data type in bit fields
  #define BITS                   unsigned int                         ///< data type in bit structs (follow C90 standard)


// IAR Compiler
#elif defined(__ICCSTM8__)

  // include intrinsic functions
  #include <intrinsics.h>

  // macros to unify ISR declaration and implementation
  #define STRINGVECTOR(x) #x
  #define VECTOR_ID(x) STRINGVECTOR( vector = (x) )
  #define ISR_HANDLER( a, b )  \
    _Pragma( VECTOR_ID( (b)+2 ) )        \
    __interrupt void (a)( void )
  #define ISR_HANDLER_TRAP(a) \
    _Pragma( VECTOR_ID( 1 ) ) \
    __interrupt void (a) (void)  

  // definition of inline functions
  #define INLINE                 static inline                        ///< keyword for inline functions

  // common assembler instructions
  #define NOP()                  __no_operation()                     ///< perform a nop() operation (=minimum delay)
  #define DISABLE_INTERRUPTS()   __disable_interrupt()                ///< disable interrupt handling
  #define ENABLE_INTERRUPTS()    __enable_interrupt()                 ///< enable interrupt handling
  #define TRIGGER_TRAP           __trap()                             ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
  #define WAIT_FOR_INTERRUPT()   __wait_for_interrupt()               ///< stop code execution and wait for interrupt
  #define ENTER_HALT()           __halt()                             ///< put controller to HALT mode
  #define SW_RESET()             __asm("dc8 0x75")                    ///< reset via illegal opcode (works for all devices)

  // data type in bit fields
  #define BITS                   unsigned char                        ///< data type in bit structs (deviating from C90 standard)


// SDCC compiler
#elif defined(__SDCC)

  // store SDCC version in preprocessor friendly way
  #define SDCC_VERSION (__SDCC_VERSION_MAJOR * 10000 \
                      + __SDCC_VERSION_MINOR * 100 \
                      + __SDCC_VERSION_PATCH)

  // unify ISR declaration and implementation
  #define ISR_HANDLER(func,irq)   void func(void) __interrupt(irq)    ///< handler for interrupt service routine
  #if SDCC_VERSION >= 30403  // traps require >=v3.4.3
    #define ISR_HANDLER_TRAP(func)  void func() __trap                ///< handler for trap service routine
  #else
    #error traps require SDCC >=3.4.3. Please update!
  #endif

  // definition of inline functions
  #define INLINE                 static inline                        ///< keyword for inline functions

  // common assembler instructions
  #define NOP()                  __asm__("nop")                       ///< perform a nop() operation (=minimum delay)
  #define DISABLE_INTERRUPTS()   __asm__("sim")                       ///< disable interrupt handling
  #define ENABLE_INTERRUPTS()    __asm__("rim")                       ///< enable interrupt handling
  #define TRIGGER_TRAP           __asm__("trap")                      ///< trigger a trap (=soft interrupt) e.g. for EMC robustness (see AN1015)
  #define WAIT_FOR_INTERRUPT()   __asm__("wfi")                       ///< stop code execution and wait for interrupt
  #define ENTER_HALT()           __asm__("halt")                      ///< put controller to HALT mode
  #define SW_RESET()             __asm__(".db 0x75")                  ///< reset via illegal opcode (works for all devices)

  // data type in bit fields
  #define BITS                   unsigned int                         ///< data type in bit structs (follow C90 standard)

// unsupported compiler -> stop
#else
  #error: compiler not supported
#endif


/*-------------------------------------------------------------------------
  FOR CONVENIENT PIN ACCESS
-------------------------------------------------------------------------*/

#define PIN0     0x01
#define PIN1     0x02
#define PIN2     0x04
#define PIN3     0x08
#define PIN4     0x10
#define PIN5     0x20
#define PIN6     0x40
#define PIN7     0x80


/*-------------------------------------------------------------------------
  DEVICE MEMORY (size in bytes)
-------------------------------------------------------------------------*/

// RAM
#define RAM_ADDR_START                0x000000
#define RAM_ADDR_END                  0x0007FF
#define RAM_SIZE                      2048


// FLASH
#define FLASH_ADDR_START              0x008000
#define FLASH_ADDR_END                0x00FFFF
#define FLASH_SIZE                    32768


// SFR1
#define SFR1_ADDR_START               0x005000
#define SFR1_ADDR_END                 0x0057FF
#define SFR1_SIZE                     2048


// SFR2
#define SFR2_ADDR_START               0x007F00
#define SFR2_ADDR_END                 0x007FFF
#define SFR2_SIZE                     256


// BOOTROM
#define BOOTROM_ADDR_START            0x006000
#define BOOTROM_ADDR_END              0x0067FF
#define BOOTROM_SIZE                  2048


// EEPROM
#define EEPROM_ADDR_START             0x004000
#define EEPROM_ADDR_END               0x0043FF
#define EEPROM_SIZE                   1024


// OPTION
#define OPTION_ADDR_START             0x004800
#define OPTION_ADDR_END               0x00487F
#define OPTION_SIZE                   128


// MEMORY WIDTH (>32kB flash exceeds 16bit, as flash starts at 0x8000)
#define FLASH_ADDR_WIDTH            16                    ///< width of address space
#define FLASH_POINTER_T             uint16_t              ///< address variable type


/*-------------------------------------------------------------------------
  MISC OPTIONS
-------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------
  ISR Vector Table (SDCC, IAR)
  Notes:
    - IAR has an IRQ offset of +2 compared to datasheet and below numbers
    - Cosmic uses a separate, device specific file 'stm8_interrupt_vector.c'
    - different interrupt sources may share the same IRQ
-------------------------------------------------------------------------*/

// interrupt                                   IRQ
#define _TLI_VECTOR_                             0          
#define _AWU_VECTOR_                             1          ///< AWU interrupt vector: enable: AWU_CSR.AWUEN, pending: AWU_CSR.AWUF, priority: ITC_SPR0.VECT1SPR
#define _CLK_CSS_VECTOR_                         2          ///< CLK_CSS interrupt vector: enable: CLK_CSSR.CSSDIE, pending: CLK_CSSR.CSSD, priority: ITC_SPR0.VECT2SPR
#define _CLK_SWITCH_VECTOR_                      2          ///< CLK_SWITCH interrupt vector: enable: CLK_SWCR.SWIEN, pending: CLK_SWCR.SWIF, priority: ITC_SPR0.VECT2SPR
#define _GPIO0_INT_VECTOR_                       3          ///< GPIO0_INT interrupt vector: enable: MSC_CFGP00.INT_ENB, pending: MSC_STSP0.BIT_0_INT, priority: ITC_SPR0.VECT3SPR
#define _GPIO1_INT_VECTOR_                       3          ///< GPIO1_INT interrupt vector: enable: MSC_CFGP01.INT_ENB, pending: MSC_STSP0.BIT_1_INT, priority: ITC_SPR0.VECT3SPR
#define _GPIO2_INT_VECTOR_                       3          ///< GPIO2_INT interrupt vector: enable: MSC_CFGP02.INT_ENB, pending: MSC_STSP0.BIT_2_INT, priority: ITC_SPR0.VECT3SPR
#define _GPIO3_INT_VECTOR_                       3          ///< GPIO3_INT interrupt vector: enable: MSC_CFGP03.INT_ENB, pending: MSC_STSP0.BIT_3_INT, priority: ITC_SPR0.VECT3SPR
#define _GPIO4_INT_VECTOR_                       3          ///< GPIO4_INT interrupt vector: enable: MSC_CFGP04.INT_ENB, pending: MSC_STSP0.BIT_4_INT, priority: ITC_SPR0.VECT3SPR
#define _GPIO5_INT_VECTOR_                       3          ///< GPIO5_INT interrupt vector: enable: MSC_CFGP05.INT_ENB, pending: MSC_STSP0.BIT_5_INT, priority: ITC_SPR0.VECT3SPR
#define _DIGIN0_INT_VECTOR_                      5          ///< DIGIN0_INT interrupt vector: enable: MSC_CFGP20.INT_ENB, pending: MSC_STSP2.BIT_0_INT, priority: ITC_SPR1.VECT5SPR
#define _DIGIN1_INT_VECTOR_                      5          ///< DIGIN1_INT interrupt vector: enable: MSC_CFGP21.INT_ENB, pending: MSC_STSP2.BIT_1_INT, priority: ITC_SPR1.VECT5SPR
#define _DIGIN2_INT_VECTOR_                      5          ///< DIGIN2_INT interrupt vector: enable: MSC_CFGP22.INT_ENB, pending: MSC_STSP2.BIT_2_INT, priority: ITC_SPR1.VECT5SPR
#define _DIGIN3_INT_VECTOR_                      5          ///< DIGIN3_INT interrupt vector: enable: MSC_CFGP23.INT_ENB, pending: MSC_STSP2.BIT_3_INT, priority: ITC_SPR1.VECT5SPR
#define _DIGIN4_INT_VECTOR_                      5          ///< DIGIN4_INT interrupt vector: enable: MSC_CFGP24.INT_ENB, pending: MSC_STSP2.BIT_4_INT, priority: ITC_SPR1.VECT5SPR
#define _DIGIN5_INT_VECTOR_                      5          ///< DIGIN5_INT interrupt vector: enable: MSC_CFGP25.INT_ENB, pending: MSC_STSP2.BIT_5_INT, priority: ITC_SPR1.VECT5SPR
#define _SMED0_OVF_VECTOR_                       6          ///< SMED0_OVF interrupt vector: enable: SMD0_IMR.CNT_OV_R, pending: SMD0_ISR.CNT_OVER, priority: ITC_SPR1.VECT6SPR
#define _SMED0_STATE0_VECTOR_                    6          ///< SMED0_STATE0 interrupt vector: enable: SMD0_IMR.IT_STA_S0, pending: SMD0_ISR.STA_S0_IT, priority: ITC_SPR1.VECT6SPR
#define _SMED0_STATE1_VECTOR_                    6          ///< SMED0_STATE1 interrupt vector: enable: SMD0_IMR.IT_STA_S1, pending: SMD0_ISR.STA_S1_IT, priority: ITC_SPR1.VECT6SPR
#define _SMED0_STATE2_VECTOR_                    6          ///< SMED0_STATE2 interrupt vector: enable: SMD0_IMR.IT_STA_S2, pending: SMD0_ISR.STA_S2_IT, priority: ITC_SPR1.VECT6SPR
#define _SMED0_STATE3_VECTOR_                    6          ///< SMED0_STATE3 interrupt vector: enable: SMD0_IMR.IT_STA_S3, pending: SMD0_ISR.STA_S3_IT, priority: ITC_SPR1.VECT6SPR
#define _SMEDO_INSIG0_VECTOR_                    6          ///< SMEDO_INSIG0 interrupt vector: enable: SMD0_IMR.IT_EXT0, pending: SMD0_ISR.EXT0_INT, priority: ITC_SPR1.VECT6SPR
#define _SMEDO_INSIG1_VECTOR_                    6          ///< SMEDO_INSIG1 interrupt vector: enable: SMD0_IMR.IT_EXT1, pending: SMD0_ISR.EXT1_INT, priority: ITC_SPR1.VECT6SPR
#define _SMEDO_INSIG2_VECTOR_                    6          ///< SMEDO_INSIG2 interrupt vector: enable: SMD0_IMR.IT_EXT2, pending: SMD0_ISR.EXT2_INT, priority: ITC_SPR1.VECT6SPR
#define _SMED1_INSIG0_VECTOR_                    7          ///< SMED1_INSIG0 interrupt vector: enable: SMD1_IMR.IT_EXT0, pending: SMD1_ISR.EXT0_INT, priority: ITC_SPR1.VECT7SPR
#define _SMED1_INSIG1_VECTOR_                    7          ///< SMED1_INSIG1 interrupt vector: enable: SMD1_IMR.IT_EXT1, pending: SMD1_ISR.EXT1_INT, priority: ITC_SPR1.VECT7SPR
#define _SMED1_INSIG2_VECTOR_                    7          ///< SMED1_INSIG2 interrupt vector: enable: SMD1_IMR.IT_EXT2, pending: SMD1_ISR.EXT2_INT, priority: ITC_SPR1.VECT7SPR
#define _SMED1_OVF_VECTOR_                       7          ///< SMED1_OVF interrupt vector: enable: SMD1_IMR.CNT_OV_R, pending: SMD1_ISR.CNT_OVER, priority: ITC_SPR1.VECT7SPR
#define _SMED1_STATE0_VECTOR_                    7          ///< SMED1_STATE0 interrupt vector: enable: SMD1_IMR.IT_STA_S0, pending: SMD1_ISR.STA_S0_IT, priority: ITC_SPR1.VECT7SPR
#define _SMED1_STATE1_VECTOR_                    7          ///< SMED1_STATE1 interrupt vector: enable: SMD1_IMR.IT_STA_S1, pending: SMD1_ISR.STA_S1_IT, priority: ITC_SPR1.VECT7SPR
#define _SMED1_STATE2_VECTOR_                    7          ///< SMED1_STATE2 interrupt vector: enable: SMD1_IMR.IT_STA_S2, pending: SMD1_ISR.STA_S2_IT, priority: ITC_SPR1.VECT7SPR
#define _SMED1_STATE3_VECTOR_                    7          ///< SMED1_STATE3 interrupt vector: enable: SMD1_IMR.IT_STA_S3, pending: SMD1_ISR.STA_S3_IT, priority: ITC_SPR1.VECT7SPR
#define _SMED2_INSIG0_VECTOR_                    15         ///< SMED2_INSIG0 interrupt vector: enable: SMD2_IMR.IT_EXT0, pending: SMD2_ISR.EXT0_INT, priority: ITC_SPR3.VECT15SPR
#define _SMED2_INSIG1_VECTOR_                    15         ///< SMED2_INSIG1 interrupt vector: enable: SMD2_IMR.IT_EXT1, pending: SMD2_ISR.EXT1_INT, priority: ITC_SPR3.VECT15SPR
#define _SMED2_INSIG2_VECTOR_                    15         ///< SMED2_INSIG2 interrupt vector: enable: SMD2_IMR.IT_EXT2, pending: SMD2_ISR.EXT2_INT, priority: ITC_SPR3.VECT15SPR
#define _SMED2_OVF_VECTOR_                       15         ///< SMED2_OVF interrupt vector: enable: SMD2_IMR.CNT_OV_R, pending: SMD2_ISR.CNT_OVER, priority: ITC_SPR3.VECT15SPR
#define _SMED2_STATE0_VECTOR_                    15         ///< SMED2_STATE0 interrupt vector: enable: SMD2_IMR.IT_STA_S0, pending: SMD2_ISR.STA_S0_IT, priority: ITC_SPR3.VECT15SPR
#define _SMED2_STATE1_VECTOR_                    15         ///< SMED2_STATE1 interrupt vector: enable: SMD2_IMR.IT_STA_S1, pending: SMD2_ISR.STA_S1_IT, priority: ITC_SPR3.VECT15SPR
#define _SMED2_STATE2_VECTOR_                    15         ///< SMED2_STATE2 interrupt vector: enable: SMD2_IMR.IT_STA_S2, pending: SMD2_ISR.STA_S2_IT, priority: ITC_SPR3.VECT15SPR
#define _SMED2_STATE3_VECTOR_                    15         ///< SMED2_STATE3 interrupt vector: enable: SMD2_IMR.IT_STA_S3, pending: SMD2_ISR.STA_S3_IT, priority: ITC_SPR3.VECT15SPR
#define _SMED3_INSIG0_VECTOR_                    16         ///< SMED3_INSIG0 interrupt vector: enable: SMD3_IMR.IT_EXT0, pending: SMD3_ISR.EXT0_INT, priority: ITC_SPR4.VECT16SPR
#define _SMED3_INSIG1_VECTOR_                    16         ///< SMED3_INSIG1 interrupt vector: enable: SMD3_IMR.IT_EXT1, pending: SMD3_ISR.EXT1_INT, priority: ITC_SPR4.VECT16SPR
#define _SMED3_INSIG2_VECTOR_                    16         ///< SMED3_INSIG2 interrupt vector: enable: SMD3_IMR.IT_EXT2, pending: SMD3_ISR.EXT2_INT, priority: ITC_SPR4.VECT16SPR
#define _SMED3_OVF_VECTOR_                       16         ///< SMED3_OVF interrupt vector: enable: SMD3_IMR.CNT_OV_R, pending: SMD3_ISR.CNT_OVER, priority: ITC_SPR4.VECT16SPR
#define _SMED3_STATE0_VECTOR_                    16         ///< SMED3_STATE0 interrupt vector: enable: SMD3_IMR.IT_STA_S0, pending: SMD3_ISR.STA_S0_IT, priority: ITC_SPR4.VECT16SPR
#define _SMED3_STATE1_VECTOR_                    16         ///< SMED3_STATE1 interrupt vector: enable: SMD3_IMR.IT_STA_S1, pending: SMD3_ISR.STA_S1_IT, priority: ITC_SPR4.VECT16SPR
#define _SMED3_STATE2_VECTOR_                    16         ///< SMED3_STATE2 interrupt vector: enable: SMD3_IMR.IT_STA_S2, pending: SMD3_ISR.STA_S2_IT, priority: ITC_SPR4.VECT16SPR
#define _SMED3_STATE3_VECTOR_                    16         ///< SMED3_STATE3 interrupt vector: enable: SMD3_IMR.IT_STA_S3, pending: SMD3_ISR.STA_S3_IT, priority: ITC_SPR4.VECT16SPR
#define _UART_T_TC_VECTOR_                       17         ///< UART_T_TC interrupt vector: enable: UART_CR2.TCIEN, pending: UART_SR.TC, priority: ITC_SPR4.VECT17SPR
#define _UART_T_TXE_VECTOR_                      17         ///< UART_T_TXE interrupt vector: enable: UART_CR2.TIEN, pending: UART_SR.TXE, priority: ITC_SPR4.VECT17SPR
#define _UART_R_IDLE_VECTOR_                     18         ///< UART_R_IDLE interrupt vector: enable: UART_CR2.ILIEN, pending: UART_SR.IDLE, priority: ITC_SPR4.VECT18SPR
#define _UART_R_OR_VECTOR_                       18         ///< UART_R_OR interrupt vector: enable: UART_CR2.RIEN, pending: UART_SR.OR, priority: ITC_SPR4.VECT18SPR
#define _UART_R_PE_VECTOR_                       18         ///< UART_R_PE interrupt vector: enable: UART_CR1.PIEN, pending: UART_SR.PE, priority: ITC_SPR4.VECT18SPR
#define _UART_R_RXNE_VECTOR_                     18         ///< UART_R_RXNE interrupt vector: enable: UART_CR2.RIEN, pending: UART_SR.RXNE, priority: ITC_SPR4.VECT18SPR
#define _I2C_ADD10_VECTOR_                       19         ///< I2C_ADD10 interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.ADD10, priority: ITC_SPR4.VECT19SPR
#define _I2C_ADDR_VECTOR_                        19         ///< I2C_ADDR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.ADDR, priority: ITC_SPR4.VECT19SPR
#define _I2C_AF_VECTOR_                          19         ///< I2C_AF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.AF, priority: ITC_SPR4.VECT19SPR
#define _I2C_ARLO_VECTOR_                        19         ///< I2C_ARLO interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.ARLO, priority: ITC_SPR4.VECT19SPR
#define _I2C_BERR_VECTOR_                        19         ///< I2C_BERR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.BERR, priority: ITC_SPR4.VECT19SPR
#define _I2C_BTF_VECTOR_                         19         ///< I2C_BTF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.BTF, priority: ITC_SPR4.VECT19SPR
#define _I2C_OVR_VECTOR_                         19         ///< I2C_OVR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.OVR, priority: ITC_SPR4.VECT19SPR
#define _I2C_RXNE_VECTOR_                        19         ///< I2C_RXNE interrupt vector: enable: I2C_ITR.ITBUFEN, pending: I2C_SR1.RXNE, priority: ITC_SPR4.VECT19SPR
#define _I2C_SB_VECTOR_                          19         ///< I2C_SB interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.SB, priority: ITC_SPR4.VECT19SPR
#define _I2C_STOPF_VECTOR_                       19         ///< I2C_STOPF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.STOPF, priority: ITC_SPR4.VECT19SPR
#define _I2C_TXE_VECTOR_                         19         ///< I2C_TXE interrupt vector: enable: I2C_ITR.ITBUFEN, pending: I2C_SR1.TXE, priority: ITC_SPR4.VECT19SPR
#define _ADC_EOC_VECTOR_                         22         ///< ADC_EOC interrupt vector: enable: ADC_IER.EOC_EN, pending: ADC_SR.EOC, priority: ITC_SPR5.VECT22SPR
#define _ADC_EOS_VECTOR_                         22         ///< ADC_EOS interrupt vector: enable: ADC_IER.EOS_EN, pending: ADC_SR.EOS, priority: ITC_SPR5.VECT22SPR
#define _ADC_SEQ_VECTOR_                         22         ///< ADC_SEQ interrupt vector: enable: ADC_IER.SEQ_FULL_EN, pending: ADC_SR.SEQ_FULL, priority: ITC_SPR5.VECT22SPR
#define _STMR_OVF_VECTOR_                        23         ///< STMR_OVF interrupt vector: enable: STMR_IER.UIE, pending: STMR_SR1.UIF, priority: ITC_SPR5.VECT23SPR
#define _FLASH_EOP_VECTOR_                       24         ///< FLASH_EOP interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.EOP, priority: ITC_SPR6.VECT24SPR
#define _FLASH_WR_PG_DIS_VECTOR_                 24         ///< FLASH_WR_PG_DIS interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.WR_PG_DIS, priority: ITC_SPR6.VECT24SPR
#define _DALI_ITF_VECTOR_                        25         ///< DALI_ITF interrupt vector: enable: DALI_CSR.IEN, pending: DALI_CSR.ITF, priority: ITC_SPR6.VECT25SPR
#define _DALI_WDGF_VECTOR_                       25         ///< DALI_WDGF interrupt vector: enable: DALI_CSR.WDGE, pending: DALI_CSR.WDGF, priority: ITC_SPR6.VECT25SPR
#define _SMED4_INSIG0_VECTOR_                    26         ///< SMED4_INSIG0 interrupt vector: enable: SMD4_IMR.IT_EXT0, pending: SMD4_ISR.EXT0_INT, priority: ITC_SPR6.VECT26SPR
#define _SMED4_INSIG1_VECTOR_                    26         ///< SMED4_INSIG1 interrupt vector: enable: SMD4_IMR.IT_EXT1, pending: SMD4_ISR.EXT1_INT, priority: ITC_SPR6.VECT26SPR
#define _SMED4_INSIG2_VECTOR_                    26         ///< SMED4_INSIG2 interrupt vector: enable: SMD4_IMR.IT_EXT2, pending: SMD4_ISR.EXT2_INT, priority: ITC_SPR6.VECT26SPR
#define _SMED4_OVF_VECTOR_                       26         ///< SMED4_OVF interrupt vector: enable: SMD4_IMR.CNT_OV_R, pending: SMD4_ISR.CNT_OVER, priority: ITC_SPR6.VECT26SPR
#define _SMED4_STATE0_VECTOR_                    26         ///< SMED4_STATE0 interrupt vector: enable: SMD4_IMR.IT_STA_S0, pending: SMD4_ISR.STA_S0_IT, priority: ITC_SPR6.VECT26SPR
#define _SMED4_STATE1_VECTOR_                    26         ///< SMED4_STATE1 interrupt vector: enable: SMD4_IMR.IT_STA_S1, pending: SMD4_ISR.STA_S1_IT, priority: ITC_SPR6.VECT26SPR
#define _SMED4_STATE2_VECTOR_                    26         ///< SMED4_STATE2 interrupt vector: enable: SMD4_IMR.IT_STA_S2, pending: SMD4_ISR.STA_S2_IT, priority: ITC_SPR6.VECT26SPR
#define _SMED4_STATE3_VECTOR_                    26         ///< SMED4_STATE3 interrupt vector: enable: SMD4_IMR.IT_STA_S3, pending: SMD4_ISR.STA_S3_IT, priority: ITC_SPR6.VECT26SPR
#define _SMED5_INSIG0_VECTOR_                    27         ///< SMED5_INSIG0 interrupt vector: enable: SMD5_IMR.IT_EXT0, pending: SMD5_ISR.EXT0_INT, priority: ITC_SPR6.VECT27SPR
#define _SMED5_INSIG1_VECTOR_                    27         ///< SMED5_INSIG1 interrupt vector: enable: SMD5_IMR.IT_EXT1, pending: SMD5_ISR.EXT1_INT, priority: ITC_SPR6.VECT27SPR
#define _SMED5_INSIG2_VECTOR_                    27         ///< SMED5_INSIG2 interrupt vector: enable: SMD5_IMR.IT_EXT2, pending: SMD5_ISR.EXT2_INT, priority: ITC_SPR6.VECT27SPR
#define _SMED5_OVF_VECTOR_                       27         ///< SMED5_OVF interrupt vector: enable: SMD5_IMR.CNT_OV_R, pending: SMD5_ISR.CNT_OVER, priority: ITC_SPR6.VECT27SPR
#define _SMED5_STATE0_VECTOR_                    27         ///< SMED5_STATE0 interrupt vector: enable: SMD5_IMR.IT_STA_S0, pending: SMD5_ISR.STA_S0_IT, priority: ITC_SPR6.VECT27SPR
#define _SMED5_STATE1_VECTOR_                    27         ///< SMED5_STATE1 interrupt vector: enable: SMD5_IMR.IT_STA_S1, pending: SMD5_ISR.STA_S1_IT, priority: ITC_SPR6.VECT27SPR
#define _SMED5_STATE2_VECTOR_                    27         ///< SMED5_STATE2 interrupt vector: enable: SMD5_IMR.IT_STA_S2, pending: SMD5_ISR.STA_S2_IT, priority: ITC_SPR6.VECT27SPR
#define _SMED5_STATE3_VECTOR_                    27         ///< SMED5_STATE3 interrupt vector: enable: SMD5_IMR.IT_STA_S3, pending: SMD5_ISR.STA_S3_IT, priority: ITC_SPR6.VECT27SPR


/*-------------------------------------------------------------------------
  DEFINITION OF STM8 PERIPHERAL REGISTERS
-------------------------------------------------------------------------*/

//------------------------
// Module ADC
//------------------------

/** struct containing ADC module registers */
typedef struct {

  /** Configuration register (CFG at 0x5400) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS   PD                  : 1;      // bit 0
      BITS   STOP                : 1;      // bit 1
      BITS   FIFO_FLUSH          : 1;      // bit 2
      BITS   CIRCULAR            : 1;      // bit 3
      BITS   FORMAT              : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _ADC_CFG reset value
    #define sfr_ADC_CFG_RESET_VALUE   ((uint8_t) 0x01)

  } CFG;


  /** Start Of Conversion (SOC at 0x5401) */
  union {

    /// bytewise access to SOC
    uint8_t  byte;

    /// bitwise access to register SOC
    struct {
      BITS   SOC                 : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // SOC bitfield

    /// register _ADC_SOC reset value
    #define sfr_ADC_SOC_RESET_VALUE   ((uint8_t) 0x00)

  } SOC;


  /** Interrupt Enable Register (IER at 0x5402) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   EOC_EN              : 1;      // bit 0
      BITS   EOS_EN              : 1;      // bit 1
      BITS   SEQ_FULL_EN         : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // IER bitfield

    /// register _ADC_IER reset value
    #define sfr_ADC_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** Sequencer Register (SEQ at 0x5403) */
  union {

    /// bytewise access to SEQ
    uint8_t  byte;

    /// bitwise access to register SEQ
    struct {
      BITS   CH                  : 3;      // bits 0-2
      BITS   GAIN                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SEQ bitfield

    /// register _ADC_SEQ reset value
    #define sfr_ADC_SEQ_RESET_VALUE   ((uint8_t) 0x00)

  } SEQ;


  /** Data Low Register (DATL0 at 0x5404) */
  union {

    /// bytewise access to DATL0
    uint8_t  byte;

    /// bitwise access to register DATL0
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL0 bitfield

    /// register _ADC_DATL0 reset value
    #define sfr_ADC_DATL0_RESET_VALUE   ((uint8_t) 0x00)

  } DATL0;


  /** Data High Register (DATH0 at 0x5405) */
  union {

    /// bytewise access to DATH0
    uint8_t  byte;

    /// bitwise access to register DATH0
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH0 bitfield

    /// register _ADC_DATH0 reset value
    #define sfr_ADC_DATH0_RESET_VALUE   ((uint8_t) 0x00)

  } DATH0;


  /** Data Low Register (DATL1 at 0x5406) */
  union {

    /// bytewise access to DATL1
    uint8_t  byte;

    /// bitwise access to register DATL1
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL1 bitfield

    /// register _ADC_DATL1 reset value
    #define sfr_ADC_DATL1_RESET_VALUE   ((uint8_t) 0x00)

  } DATL1;


  /** Data High Register (DATH1 at 0x5407) */
  union {

    /// bytewise access to DATH1
    uint8_t  byte;

    /// bitwise access to register DATH1
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH1 bitfield

    /// register _ADC_DATH1 reset value
    #define sfr_ADC_DATH1_RESET_VALUE   ((uint8_t) 0x00)

  } DATH1;


  /** Data Low Register (DATL2 at 0x5408) */
  union {

    /// bytewise access to DATL2
    uint8_t  byte;

    /// bitwise access to register DATL2
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL2 bitfield

    /// register _ADC_DATL2 reset value
    #define sfr_ADC_DATL2_RESET_VALUE   ((uint8_t) 0x00)

  } DATL2;


  /** Data High Register (DATH2 at 0x5409) */
  union {

    /// bytewise access to DATH2
    uint8_t  byte;

    /// bitwise access to register DATH2
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH2 bitfield

    /// register _ADC_DATH2 reset value
    #define sfr_ADC_DATH2_RESET_VALUE   ((uint8_t) 0x00)

  } DATH2;


  /** Data Low Register (DATL3 at 0x540a) */
  union {

    /// bytewise access to DATL3
    uint8_t  byte;

    /// bitwise access to register DATL3
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL3 bitfield

    /// register _ADC_DATL3 reset value
    #define sfr_ADC_DATL3_RESET_VALUE   ((uint8_t) 0x00)

  } DATL3;


  /** Data High Register (DATH3 at 0x540b) */
  union {

    /// bytewise access to DATH3
    uint8_t  byte;

    /// bitwise access to register DATH3
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH3 bitfield

    /// register _ADC_DATH3 reset value
    #define sfr_ADC_DATH3_RESET_VALUE   ((uint8_t) 0x00)

  } DATH3;


  /** Data Low Register (DATL4 at 0x540c) */
  union {

    /// bytewise access to DATL4
    uint8_t  byte;

    /// bitwise access to register DATL4
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL4 bitfield

    /// register _ADC_DATL4 reset value
    #define sfr_ADC_DATL4_RESET_VALUE   ((uint8_t) 0x00)

  } DATL4;


  /** Data High Register (DATH4 at 0x540d) */
  union {

    /// bytewise access to DATH4
    uint8_t  byte;

    /// bitwise access to register DATH4
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH4 bitfield

    /// register _ADC_DATH4 reset value
    #define sfr_ADC_DATH4_RESET_VALUE   ((uint8_t) 0x00)

  } DATH4;


  /** Data Low Register (DATL5 at 0x540e) */
  union {

    /// bytewise access to DATL5
    uint8_t  byte;

    /// bitwise access to register DATL5
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL5 bitfield

    /// register _ADC_DATL5 reset value
    #define sfr_ADC_DATL5_RESET_VALUE   ((uint8_t) 0x00)

  } DATL5;


  /** Data High Register (DATH5 at 0x540f) */
  union {

    /// bytewise access to DATH5
    uint8_t  byte;

    /// bitwise access to register DATH5
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH5 bitfield

    /// register _ADC_DATH5 reset value
    #define sfr_ADC_DATH5_RESET_VALUE   ((uint8_t) 0x00)

  } DATH5;


  /** Data Low Register (DATL6 at 0x5410) */
  union {

    /// bytewise access to DATL6
    uint8_t  byte;

    /// bitwise access to register DATL6
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL6 bitfield

    /// register _ADC_DATL6 reset value
    #define sfr_ADC_DATL6_RESET_VALUE   ((uint8_t) 0x00)

  } DATL6;


  /** Data High Register (DATH6 at 0x5411) */
  union {

    /// bytewise access to DATH6
    uint8_t  byte;

    /// bitwise access to register DATH6
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH6 bitfield

    /// register _ADC_DATH6 reset value
    #define sfr_ADC_DATH6_RESET_VALUE   ((uint8_t) 0x00)

  } DATH6;


  /** Data Low Register (DATL7 at 0x5412) */
  union {

    /// bytewise access to DATL7
    uint8_t  byte;

    /// bitwise access to register DATL7
    struct {
      BITS   DATA_LOW            : 8;      // bits 0-7
    };  // DATL7 bitfield

    /// register _ADC_DATL7 reset value
    #define sfr_ADC_DATL7_RESET_VALUE   ((uint8_t) 0x00)

  } DATL7;


  /** Data High Register (DATH7 at 0x5413) */
  union {

    /// bytewise access to DATH7
    uint8_t  byte;

    /// bitwise access to register DATH7
    struct {
      BITS   DATA_HIGH           : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DATH7 bitfield

    /// register _ADC_DATH7 reset value
    #define sfr_ADC_DATH7_RESET_VALUE   ((uint8_t) 0x00)

  } DATH7;


  /** Status Register (SR at 0x5414) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   EOC                 : 1;      // bit 0
      BITS   EOS                 : 1;      // bit 1
      BITS   SEQ_FULL            : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // SR bitfield

    /// register _ADC_SR reset value
    #define sfr_ADC_SR_RESET_VALUE   ((uint8_t) 0x00)

  } SR;


  /** SOC Delay Counter Register (DLYCNT at 0x5415) */
  union {

    /// bytewise access to DLYCNT
    uint8_t  byte;

    /// bitwise access to register DLYCNT
    struct {
      BITS   SOC_DLY_CNT         : 8;      // bits 0-7
    };  // DLYCNT bitfield

    /// register _ADC_DLYCNT reset value
    #define sfr_ADC_DLYCNT_RESET_VALUE   ((uint8_t) 0x00)

  } DLYCNT;

} ADC_t;

/// access to ADC SFR registers
#define sfr_ADC   (*((ADC_t*) 0x5400))


//------------------------
// Module AWU
//------------------------

/** struct containing AWU module registers */
typedef struct {

  /** AWU control/status register 1 (AWU_CSR at 0x50f0) */
  union {

    /// bytewise access to AWU_CSR
    uint8_t  byte;

    /// bitwise access to register AWU_CSR
    struct {
      BITS                       : 1;      // 1 bit
      BITS   MR                  : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   AWUEN               : 1;      // bit 4
      BITS   AWUF                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // AWU_CSR bitfield

    /// register _AWU_AWU_CSR reset value
    #define sfr_AWU_AWU_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } AWU_CSR;


  /** AWU asynchronous prescaler buffer register (AWU_APR at 0x50f1) */
  union {

    /// bytewise access to AWU_APR
    uint8_t  byte;

    /// bitwise access to register AWU_APR
    struct {
      BITS   APR                 : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // AWU_APR bitfield

    /// register _AWU_AWU_APR reset value
    #define sfr_AWU_AWU_APR_RESET_VALUE   ((uint8_t) 0x3F)

  } AWU_APR;


  /** AWU timebase selection register (AWU_TBR at 0x50f2) */
  union {

    /// bytewise access to AWU_TBR
    uint8_t  byte;

    /// bitwise access to register AWU_TBR
    struct {
      BITS   AWUTB               : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // AWU_TBR bitfield

    /// register _AWU_AWU_TBR reset value
    #define sfr_AWU_AWU_TBR_RESET_VALUE   ((uint8_t) 0x00)

  } AWU_TBR;

} AWU_t;

/// access to AWU SFR registers
#define sfr_AWU   (*((AWU_t*) 0x50f0))


//------------------------
// Module CLK
//------------------------

/** struct containing CLK module registers */
typedef struct {

  /** SMED0 clock configuration register (SMED0 at 0x50b4) */
  union {

    /// bytewise access to SMED0
    uint8_t  byte;

    /// bitwise access to register SMED0
    struct {
      BITS   CK_SW0              : 1;      // bit 0
      BITS   CK_SW1              : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SMED_0_DIV          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // SMED0 bitfield

    /// register _CLK_SMED0 reset value
    #define sfr_CLK_SMED0_RESET_VALUE   ((uint8_t) 0x00)

  } SMED0;


  /** SMED1 clock configuration register (SMED1 at 0x50b5) */
  union {

    /// bytewise access to SMED1
    uint8_t  byte;

    /// bitwise access to register SMED1
    struct {
      BITS   CK_SW0              : 1;      // bit 0
      BITS   CK_SW1              : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SMED_1_DIV          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // SMED1 bitfield

    /// register _CLK_SMED1 reset value
    #define sfr_CLK_SMED1_RESET_VALUE   ((uint8_t) 0x00)

  } SMED1;


  /** SMED2 clock configuration register (SMED2 at 0x50b6) */
  union {

    /// bytewise access to SMED2
    uint8_t  byte;

    /// bitwise access to register SMED2
    struct {
      BITS   CK_SW0              : 1;      // bit 0
      BITS   CK_SW1              : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SMED_2_DIV          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // SMED2 bitfield

    /// register _CLK_SMED2 reset value
    #define sfr_CLK_SMED2_RESET_VALUE   ((uint8_t) 0x00)

  } SMED2;


  /** SMED3 clock configuration register (SMED3 at 0x50b7) */
  union {

    /// bytewise access to SMED3
    uint8_t  byte;

    /// bitwise access to register SMED3
    struct {
      BITS   CK_SW0              : 1;      // bit 0
      BITS   CK_SW1              : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SMED_3_DIV          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // SMED3 bitfield

    /// register _CLK_SMED3 reset value
    #define sfr_CLK_SMED3_RESET_VALUE   ((uint8_t) 0x00)

  } SMED3;


  /** SMED4 clock configuration register (SMED4 at 0x50b8) */
  union {

    /// bytewise access to SMED4
    uint8_t  byte;

    /// bitwise access to register SMED4
    struct {
      BITS   CK_SW0              : 1;      // bit 0
      BITS   CK_SW1              : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SMED_4_DIV          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // SMED4 bitfield

    /// register _CLK_SMED4 reset value
    #define sfr_CLK_SMED4_RESET_VALUE   ((uint8_t) 0x00)

  } SMED4;


  /** SMED5 clock configuration register (SMED5 at 0x50b9) */
  union {

    /// bytewise access to SMED5
    uint8_t  byte;

    /// bitwise access to register SMED5
    struct {
      BITS   CK_SW0              : 1;      // bit 0
      BITS   CK_SW1              : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   SMED_5_DIV          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // SMED5 bitfield

    /// register _CLK_SMED5 reset value
    #define sfr_CLK_SMED5_RESET_VALUE   ((uint8_t) 0x00)

  } SMED5;


  /// Reserved register (4B)
  uint8_t     Reserved_1[4];


  /** PLL prescaler configuration register (PLLDIVR at 0x50be) */
  union {

    /// bytewise access to PLLDIVR
    uint8_t  byte;

    /// bitwise access to register PLLDIVR
    struct {
      BITS   PLL_DIV             : 2;      // bits 0-1
      BITS                       : 1;      // 1 bit
      BITS   PLL_PRES_DIV        : 2;      // bits 3-4
      BITS                       : 3;      // 3 bits
    };  // PLLDIVR bitfield

    /// register _CLK_PLLDIVR reset value
    #define sfr_CLK_PLLDIVR_RESET_VALUE   ((uint8_t) 0x02)

  } PLLDIVR;


  /** AWU clock prescaler configuration register (AWUDIVR at 0x50bf) */
  union {

    /// bytewise access to AWUDIVR
    uint8_t  byte;

    /// bitwise access to register AWUDIVR
    struct {
      BITS   AWUDIV0             : 1;      // bit 0
      BITS   AWUDIV1             : 1;      // bit 1
      BITS   AWUDIV2             : 1;      // bit 2
      BITS   AWUDIV3             : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // AWUDIVR bitfield

    /// register _CLK_AWUDIVR reset value
    #define sfr_CLK_AWUDIVR_RESET_VALUE   ((uint8_t) 0x00)

  } AWUDIVR;


  /** Internal clock control register (ICKR at 0x50c0) */
  union {

    /// bytewise access to ICKR
    uint8_t  byte;

    /// bitwise access to register ICKR
    struct {
      BITS   HSIEN               : 1;      // bit 0
      BITS   HSIRDY              : 1;      // bit 1
      BITS   FHW                 : 1;      // bit 2
      BITS   LSIEN               : 1;      // bit 3
      BITS   LSIRDY              : 1;      // bit 4
      BITS   REGAH               : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // ICKR bitfield

    /// register _CLK_ICKR reset value
    #define sfr_CLK_ICKR_RESET_VALUE   ((uint8_t) 0x01)

  } ICKR;


  /** External clock control register (ECKR at 0x50c1) */
  union {

    /// bytewise access to ECKR
    uint8_t  byte;

    /// bitwise access to register ECKR
    struct {
      BITS   HSEEN               : 1;      // bit 0
      BITS   HSERDY              : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // ECKR bitfield

    /// register _CLK_ECKR reset value
    #define sfr_CLK_ECKR_RESET_VALUE   ((uint8_t) 0x00)

  } ECKR;


  /** External clock control register (PLLR at 0x50c2) */
  union {

    /// bytewise access to PLLR
    uint8_t  byte;

    /// bitwise access to register PLLR
    struct {
      BITS   PLLON               : 1;      // bit 0
      BITS   LOCKP               : 1;      // bit 1
      BITS   REF_SEL             : 1;      // bit 2
      BITS   BYPASS              : 1;      // bit 3
      BITS   SSCG_CTRL           : 1;      // bit 4
      BITS   SPREAD_CTRL         : 1;      // bit 5
      BITS   PLL_LOCK_INT        : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PLLR bitfield

    /// register _CLK_PLLR reset value
    #define sfr_CLK_PLLR_RESET_VALUE   ((uint8_t) 0x00)

  } PLLR;


  /** Clock master status register (CMSR at 0x50c3) */
  union {

    /// bytewise access to CMSR
    uint8_t  byte;

    /// bitwise access to register CMSR
    struct {
      BITS   CKM                 : 4;      // bits 0-3
      BITS   NCKM                : 4;      // bits 4-7
    };  // CMSR bitfield

    /// register _CLK_CMSR reset value
    #define sfr_CLK_CMSR_RESET_VALUE   ((uint8_t) 0xE1)

  } CMSR;


  /** Clock master switch register (SWR at 0x50c4) */
  union {

    /// bytewise access to SWR
    uint8_t  byte;

    /// bitwise access to register SWR
    struct {
      BITS   SWI                 : 4;      // bits 0-3
      BITS   NSWI                : 4;      // bits 4-7
    };  // SWR bitfield

    /// register _CLK_SWR reset value
    #define sfr_CLK_SWR_RESET_VALUE   ((uint8_t) 0xE1)

  } SWR;


  /** Clock switch control register (SWCR at 0x50c5) */
  union {

    /// bytewise access to SWCR
    uint8_t  byte;

    /// bitwise access to register SWCR
    struct {
      BITS   SWBSY               : 1;      // bit 0
      BITS   SWEN                : 1;      // bit 1
      BITS   SWIEN               : 1;      // bit 2
      BITS   SWIF                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SWCR bitfield

    /// register _CLK_SWCR reset value
    #define sfr_CLK_SWCR_RESET_VALUE   ((uint8_t) 0x00)

  } SWCR;


  /** Clock divider register (DIVR at 0x50c6) */
  union {

    /// bytewise access to DIVR
    uint8_t  byte;

    /// bitwise access to register DIVR
    struct {
      BITS   CPUDIV              : 3;      // bits 0-2
      BITS   HSIDIV              : 2;      // bits 3-4
      BITS                       : 3;      // 3 bits
    };  // DIVR bitfield

    /// register _CLK_DIVR reset value
    #define sfr_CLK_DIVR_RESET_VALUE   ((uint8_t) 0x18)

  } DIVR;


  /** Peripheral clock gating register 1 (PCKENR1 at 0x50c7) */
  union {

    /// bytewise access to PCKENR1
    uint8_t  byte;

    /// bitwise access to register PCKENR1
    struct {
      BITS   PCKEN               : 8;      // bits 0-7
    };  // PCKENR1 bitfield

    /// register _CLK_PCKENR1 reset value
    #define sfr_CLK_PCKENR1_RESET_VALUE   ((uint8_t) 0xFF)

  } PCKENR1;


  /** Clock security system register (CSSR at 0x50c8) */
  union {

    /// bytewise access to CSSR
    uint8_t  byte;

    /// bitwise access to register CSSR
    struct {
      BITS   CSSEN               : 1;      // bit 0
      BITS   AUX                 : 1;      // bit 1
      BITS   CSSDIE              : 1;      // bit 2
      BITS   CSSD                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // CSSR bitfield

    /// register _CLK_CSSR reset value
    #define sfr_CLK_CSSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSSR;


  /** Configurable clock control register (CCOR at 0x50c9) */
  union {

    /// bytewise access to CCOR
    uint8_t  byte;

    /// bitwise access to register CCOR
    struct {
      BITS   CCOEN               : 1;      // bit 0
      BITS   CCOSEL              : 4;      // bits 1-4
      BITS   CCORDY              : 1;      // bit 5
      BITS   CCOBSY              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // CCOR bitfield

    /// register _CLK_CCOR reset value
    #define sfr_CLK_CCOR_RESET_VALUE   ((uint8_t) 0x00)

  } CCOR;


  /** Peripheral clock gating register 2 (PCKENR2 at 0x50ca) */
  union {

    /// bytewise access to PCKENR2
    uint8_t  byte;

    /// bitwise access to register PCKENR2
    struct {
      BITS   PCKEN               : 8;      // bits 0-7
    };  // PCKENR2 bitfield

    /// register _CLK_PCKENR2 reset value
    #define sfr_CLK_PCKENR2_RESET_VALUE   ((uint8_t) 0xFF)

  } PCKENR2;


  /// Reserved register (1B)
  uint8_t     Reserved_2[1];


  /** HSI clock calibration trimming register (HSITRIMR at 0x50cc) */
  union {

    /// bytewise access to HSITRIMR
    uint8_t  byte;

    /// bitwise access to register HSITRIMR
    struct {
      BITS   HSITRIM             : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // HSITRIMR bitfield

    /// register _CLK_HSITRIMR reset value
    #define sfr_CLK_HSITRIMR_RESET_VALUE   ((uint8_t) 0x00)

  } HSITRIMR;


  /** SWIM clock control register (SWIMCCR at 0x50cd) */
  union {

    /// bytewise access to SWIMCCR
    uint8_t  byte;

    /// bitwise access to register SWIMCCR
    struct {
      BITS   SWIMCLK             : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // SWIMCCR bitfield

    /// register _CLK_SWIMCCR reset value
    #define sfr_CLK_SWIMCCR_RESET_VALUE   ((uint8_t) 0x00)

  } SWIMCCR;


  /** CCO divider register (CCODIVR at 0x50ce) */
  union {

    /// bytewise access to CCODIVR
    uint8_t  byte;

    /// bitwise access to register CCODIVR
    struct {
      BITS   CCODIV              : 8;      // bits 0-7
    };  // CCODIVR bitfield

    /// register _CLK_CCODIVR reset value
    #define sfr_CLK_CCODIVR_RESET_VALUE   ((uint8_t) 0x00)

  } CCODIVR;


  /** ADC clock configuration register (ADCR at 0x50cf) */
  union {

    /// bytewise access to ADCR
    uint8_t  byte;

    /// bitwise access to register ADCR
    struct {
      BITS   SEL                 : 2;      // bits 0-1
      BITS                       : 2;      // 2 bits
      BITS   ADC_DIV             : 4;      // bits 4-7
    };  // ADCR bitfield

    /// register _CLK_ADCR reset value
    #define sfr_CLK_ADCR_RESET_VALUE   ((uint8_t) 0x20)

  } ADCR;

} CLK_t;

/// access to CLK SFR registers
#define sfr_CLK   (*((CLK_t*) 0x50b4))


//------------------------
// Module DALI
//------------------------

/** struct containing DALI module registers */
typedef struct {

  /** Clock prescaler LSB (DALI_L at 0x53c0) */
  union {

    /// bytewise access to DALI_L
    uint8_t  byte;

    /// bitwise access to register DALI_L
    struct {
      BITS   DALI_CLK            : 8;      // bits 0-7
    };  // DALI_L bitfield

    /// register _DALI_DALI_L reset value
    #define sfr_DALI_DALI_L_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_L;


  /** Clock prescaler MSB (DALI_H at 0x53c1) */
  union {

    /// bytewise access to DALI_H
    uint8_t  byte;

    /// bitwise access to register DALI_H
    struct {
      BITS   DALI_CLK            : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // DALI_H bitfield

    /// register _DALI_DALI_H reset value
    #define sfr_DALI_DALI_H_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_H;


  /** Message byte 0 register (DALI_FB0 at 0x53c2) */
  union {

    /// bytewise access to DALI_FB0
    uint8_t  byte;

    /// bitwise access to register DALI_FB0
    struct {
      BITS   DALI_FB0            : 8;      // bits 0-7
    };  // DALI_FB0 bitfield

    /// register _DALI_DALI_FB0 reset value
    #define sfr_DALI_DALI_FB0_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_FB0;


  /** Message byte 1 register (DALI_FB1 at 0x53c3) */
  union {

    /// bytewise access to DALI_FB1
    uint8_t  byte;

    /// bitwise access to register DALI_FB1
    struct {
      BITS   DALI_FB1            : 8;      // bits 0-7
    };  // DALI_FB1 bitfield

    /// register _DALI_DALI_FB1 reset value
    #define sfr_DALI_DALI_FB1_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_FB1;


  /** Message byte 2 register (DALI_FB2 at 0x53c4) */
  union {

    /// bytewise access to DALI_FB2
    uint8_t  byte;

    /// bitwise access to register DALI_FB2
    struct {
      BITS   DALI_FB2            : 8;      // bits 0-7
    };  // DALI_FB2 bitfield

    /// register _DALI_DALI_FB2 reset value
    #define sfr_DALI_DALI_FB2_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_FB2;


  /** Backward Data register (DALI_BD at 0x53c5) */
  union {

    /// bytewise access to DALI_BD
    uint8_t  byte;

    /// bitwise access to register DALI_BD
    struct {
      BITS   DALI_BD             : 8;      // bits 0-7
    };  // DALI_BD bitfield

    /// register _DALI_DALI_BD reset value
    #define sfr_DALI_DALI_BD_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_BD;


  /** Control register (DALI_CR at 0x53c6) */
  union {

    /// bytewise access to DALI_CR
    uint8_t  byte;

    /// bitwise access to register DALI_CR
    struct {
      BITS   FTS                 : 1;      // bit 0
      BITS   RTS                 : 1;      // bit 1
      BITS   RTA                 : 1;      // bit 2
      BITS   DCME                : 1;      // bit 3
      BITS   MLN                 : 2;      // bits 4-5
      BITS   SMK                 : 1;      // bit 6
      BITS   LNWDG_EN            : 1;      // bit 7
    };  // DALI_CR bitfield

    /// register _DALI_DALI_CR reset value
    #define sfr_DALI_DALI_CR_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_CR;


  /** Control and Status register (DALI_CSR at 0x53c7) */
  union {

    /// bytewise access to DALI_CSR
    uint8_t  byte;

    /// bitwise access to register DALI_CSR
    struct {
      BITS                       : 2;      // 2 bits
      BITS   WDGF                : 1;      // bit 2
      BITS   WDGE                : 1;      // bit 3
      BITS   RTF                 : 1;      // bit 4
      BITS   EF                  : 1;      // bit 5
      BITS   ITF                 : 1;      // bit 6
      BITS   IEN                 : 1;      // bit 7
    };  // DALI_CSR bitfield

    /// register _DALI_DALI_CSR reset value
    #define sfr_DALI_DALI_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_CSR;


  /** Control and Status register1 (DALI_CSR1 at 0x53c8) */
  union {

    /// bytewise access to DALI_CSR1
    uint8_t  byte;

    /// bitwise access to register DALI_CSR1
    struct {
      BITS   WDG_PRSC            : 3;      // bits 0-2
      BITS   RDY_REC             : 1;      // bit 3
      BITS   CKS                 : 4;      // bits 4-7
    };  // DALI_CSR1 bitfield

    /// register _DALI_DALI_CSR1 reset value
    #define sfr_DALI_DALI_CSR1_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_CSR1;


  /** Control reverse signal line (DALI_REVLN at 0x53c9) */
  union {

    /// bytewise access to DALI_REVLN
    uint8_t  byte;

    /// bitwise access to register DALI_REVLN
    struct {
      BITS   EN_REV              : 1;      // bit 0
      BITS   REV_DIN             : 1;      // bit 1
      BITS   REV_DOUT            : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // DALI_REVLN bitfield

    /// register _DALI_DALI_REVLN reset value
    #define sfr_DALI_DALI_REVLN_RESET_VALUE   ((uint8_t) 0x00)

  } DALI_REVLN;

} DALI_t;

/// access to DALI SFR registers
#define sfr_DALI   (*((DALI_t*) 0x53c0))


//------------------------
// Module FLASH
//------------------------

/** struct containing FLASH module registers */
typedef struct {

  /** Flash control register 1 (FLASH_CR1 at 0x505a) */
  union {

    /// bytewise access to FLASH_CR1
    uint8_t  byte;

    /// bitwise access to register FLASH_CR1
    struct {
      BITS   FIX                 : 1;      // bit 0
      BITS   IE                  : 1;      // bit 1
      BITS   AHALT               : 1;      // bit 2
      BITS   HALT                : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // FLASH_CR1 bitfield

    /// register _FLASH_FLASH_CR1 reset value
    #define sfr_FLASH_FLASH_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } FLASH_CR1;


  /** Flash control register 2 (FLASH_CR2 at 0x505b) */
  union {

    /// bytewise access to FLASH_CR2
    uint8_t  byte;

    /// bitwise access to register FLASH_CR2
    struct {
      BITS   PRG                 : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   FPRG                : 1;      // bit 4
      BITS   ERASE               : 1;      // bit 5
      BITS   WWO                 : 1;      // bit 6
      BITS   OPT                 : 1;      // bit 7
    };  // FLASH_CR2 bitfield

    /// register _FLASH_FLASH_CR2 reset value
    #define sfr_FLASH_FLASH_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } FLASH_CR2;


  /** Flash complementary control register 2 (FLASH_NCR2 at 0x505c) */
  union {

    /// bytewise access to FLASH_NCR2
    uint8_t  byte;

    /// bitwise access to register FLASH_NCR2
    struct {
      BITS   NPRG                : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   NFPRG               : 1;      // bit 4
      BITS   NERASE              : 1;      // bit 5
      BITS   NWWO                : 1;      // bit 6
      BITS   NOPT                : 1;      // bit 7
    };  // FLASH_NCR2 bitfield

    /// register _FLASH_FLASH_NCR2 reset value
    #define sfr_FLASH_FLASH_NCR2_RESET_VALUE   ((uint8_t) 0xFF)

  } FLASH_NCR2;


  /** Flash protection register (FLASH_FPR at 0x505d) */
  union {

    /// bytewise access to FLASH_FPR
    uint8_t  byte;

    /// bitwise access to register FLASH_FPR
    struct {
      BITS   WPB                 : 8;      // bits 0-7
    };  // FLASH_FPR bitfield

    /// register _FLASH_FLASH_FPR reset value
    #define sfr_FLASH_FLASH_FPR_RESET_VALUE   ((uint8_t) 0x00)

  } FLASH_FPR;


  /** Flash complementary protection register (FLASH_NFPR at 0x505e) */
  union {

    /// bytewise access to FLASH_NFPR
    uint8_t  byte;

    /// bitwise access to register FLASH_NFPR
    struct {
      BITS   NWPB                : 8;      // bits 0-7
    };  // FLASH_NFPR bitfield

    /// register _FLASH_FLASH_NFPR reset value
    #define sfr_FLASH_FLASH_NFPR_RESET_VALUE   ((uint8_t) 0xFF)

  } FLASH_NFPR;


  /** Flash status register (FLASH_IAPSR at 0x505f) */
  union {

    /// bytewise access to FLASH_IAPSR
    uint8_t  byte;

    /// bitwise access to register FLASH_IAPSR
    struct {
      BITS   WR_PG_DIS           : 1;      // bit 0
      BITS   PUL                 : 1;      // bit 1
      BITS   EOP                 : 1;      // bit 2
      BITS   DUL                 : 1;      // bit 3
      BITS                       : 2;      // 2 bits
      BITS   HVOFF               : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FLASH_IAPSR bitfield

    /// register _FLASH_FLASH_IAPSR reset value
    #define sfr_FLASH_FLASH_IAPSR_RESET_VALUE   ((uint8_t) 0x40)

  } FLASH_IAPSR;


  /// Reserved register (2B)
  uint8_t     Reserved_1[2];


  /** Flash program memory unprotection register (FLASH_PUKR at 0x5062) */
  union {

    /// bytewise access to FLASH_PUKR
    uint8_t  byte;

    /// bitwise access to register FLASH_PUKR
    struct {
      BITS   WP                  : 8;      // bits 0-7
    };  // FLASH_PUKR bitfield

    /// register _FLASH_FLASH_PUKR reset value
    #define sfr_FLASH_FLASH_PUKR_RESET_VALUE   ((uint8_t) 0x00)

  } FLASH_PUKR;


  /// Reserved register (1B)
  uint8_t     Reserved_2[1];


  /** Flash data unprotection register (FLASH_DUKR at 0x5064) */
  union {

    /// bytewise access to FLASH_DUKR
    uint8_t  byte;

    /// bitwise access to register FLASH_DUKR
    struct {
      BITS   WD                  : 8;      // bits 0-7
    };  // FLASH_DUKR bitfield

    /// register _FLASH_FLASH_DUKR reset value
    #define sfr_FLASH_FLASH_DUKR_RESET_VALUE   ((uint8_t) 0x00)

  } FLASH_DUKR;


  /// Reserved register (2B)
  uint8_t     Reserved_3[2];


  /** Flash wait state register (FLASH_WAIT at 0x5067) */
  union {

    /// bytewise access to FLASH_WAIT
    uint8_t  byte;

    /// bitwise access to register FLASH_WAIT
    struct {
      BITS   Wait                : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // FLASH_WAIT bitfield

    /// register _FLASH_FLASH_WAIT reset value
    #define sfr_FLASH_FLASH_WAIT_RESET_VALUE   ((uint8_t) 0x00)

  } FLASH_WAIT;

} FLASH_t;

/// access to FLASH SFR registers
#define sfr_FLASH   (*((FLASH_t*) 0x505a))


//------------------------
// Module I2C
//------------------------

/** struct containing I2C module registers */
typedef struct {

  /** I2C control register 1 (CR1 at 0x5210) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   PE                  : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   ENGC                : 1;      // bit 6
      BITS   NOSTRETCH           : 1;      // bit 7
    };  // CR1 bitfield

    /// register _I2C_CR1 reset value
    #define sfr_I2C_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** I2C control register 2 (CR2 at 0x5211) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   START               : 1;      // bit 0
      BITS   STOP                : 1;      // bit 1
      BITS   ACK                 : 1;      // bit 2
      BITS   POS                 : 1;      // bit 3
      BITS                       : 3;      // 3 bits
      BITS   SWRST               : 1;      // bit 7
    };  // CR2 bitfield

    /// register _I2C_CR2 reset value
    #define sfr_I2C_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** I2C frequency register (FREQR at 0x5212) */
  union {

    /// bytewise access to FREQR
    uint8_t  byte;

    /// bitwise access to register FREQR
    struct {
      BITS   FREQ                : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // FREQR bitfield

    /// register _I2C_FREQR reset value
    #define sfr_I2C_FREQR_RESET_VALUE   ((uint8_t) 0x00)

  } FREQR;


  /** I2C own address register low (OARL at 0x5213) */
  union {

    /// bytewise access to OARL
    uint8_t  byte;

    /// bitwise access to register OARL
    struct {
      BITS   ADD0                : 1;      // bit 0
      BITS   ADD1                : 1;      // bit 1
      BITS   ADD2                : 1;      // bit 2
      BITS   ADD3                : 1;      // bit 3
      BITS   ADD4                : 1;      // bit 4
      BITS   ADD5                : 1;      // bit 5
      BITS   ADD6                : 1;      // bit 6
      BITS   ADD7                : 1;      // bit 7
    };  // OARL bitfield

    /// register _I2C_OARL reset value
    #define sfr_I2C_OARL_RESET_VALUE   ((uint8_t) 0x00)

  } OARL;


  /** I2C own address register high (OARH at 0x5214) */
  union {

    /// bytewise access to OARH
    uint8_t  byte;

    /// bitwise access to register OARH
    struct {
      BITS                       : 1;      // 1 bit
      BITS   ADD8                : 1;      // bit 1
      BITS   ADD9                : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   ADDCONF             : 1;      // bit 6
      BITS   ADDMODE             : 1;      // bit 7
    };  // OARH bitfield

    /// register _I2C_OARH reset value
    #define sfr_I2C_OARH_RESET_VALUE   ((uint8_t) 0x00)

  } OARH;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** I2C data register (DR at 0x5216) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _I2C_DR reset value
    #define sfr_I2C_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** I2C status register 1 (SR1 at 0x5217) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   SB                  : 1;      // bit 0
      BITS   ADDR                : 1;      // bit 1
      BITS   BTF                 : 1;      // bit 2
      BITS   ADD10               : 1;      // bit 3
      BITS   STOPF               : 1;      // bit 4
      BITS                       : 1;      // 1 bit
      BITS   RXNE                : 1;      // bit 6
      BITS   TXE                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _I2C_SR1 reset value
    #define sfr_I2C_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** I2C status register 2 (SR2 at 0x5218) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS   BERR                : 1;      // bit 0
      BITS   ARLO                : 1;      // bit 1
      BITS   AF                  : 1;      // bit 2
      BITS   OVR                 : 1;      // bit 3
      BITS                       : 1;      // 1 bit
      BITS   WUFH                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // SR2 bitfield

    /// register _I2C_SR2 reset value
    #define sfr_I2C_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** I2C status register 3 (SR3 at 0x5219) */
  union {

    /// bytewise access to SR3
    uint8_t  byte;

    /// bitwise access to register SR3
    struct {
      BITS   MSL                 : 1;      // bit 0
      BITS   BUSY                : 1;      // bit 1
      BITS   TRA                 : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   GENCALL             : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // SR3 bitfield

    /// register _I2C_SR3 reset value
    #define sfr_I2C_SR3_RESET_VALUE   ((uint8_t) 0x00)

  } SR3;


  /** I2C interrupt control register (ITR at 0x521a) */
  union {

    /// bytewise access to ITR
    uint8_t  byte;

    /// bitwise access to register ITR
    struct {
      BITS   ITERREN             : 1;      // bit 0
      BITS   ITEVTEN             : 1;      // bit 1
      BITS   ITBUFEN             : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // ITR bitfield

    /// register _I2C_ITR reset value
    #define sfr_I2C_ITR_RESET_VALUE   ((uint8_t) 0x00)

  } ITR;


  /** I2C clock control register low (CCRL at 0x521b) */
  union {

    /// bytewise access to CCRL
    uint8_t  byte;

    /// bitwise access to register CCRL
    struct {
      BITS   CCR                 : 8;      // bits 0-7
    };  // CCRL bitfield

    /// register _I2C_CCRL reset value
    #define sfr_I2C_CCRL_RESET_VALUE   ((uint8_t) 0x00)

  } CCRL;


  /** I2C clock control register high (CCRH at 0x521c) */
  union {

    /// bytewise access to CCRH
    uint8_t  byte;

    /// bitwise access to register CCRH
    struct {
      BITS   CCR                 : 4;      // bits 0-3
      BITS                       : 2;      // 2 bits
      BITS   DUTY                : 1;      // bit 6
      BITS   F_S                 : 1;      // bit 7
    };  // CCRH bitfield

    /// register _I2C_CCRH reset value
    #define sfr_I2C_CCRH_RESET_VALUE   ((uint8_t) 0x00)

  } CCRH;


  /** I2C TRISE register (TRISER at 0x521d) */
  union {

    /// bytewise access to TRISER
    uint8_t  byte;

    /// bitwise access to register TRISER
    struct {
      BITS   TRISE               : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // TRISER bitfield

    /// register _I2C_TRISER reset value
    #define sfr_I2C_TRISER_RESET_VALUE   ((uint8_t) 0x02)

  } TRISER;

} I2C_t;

/// access to I2C SFR registers
#define sfr_I2C   (*((I2C_t*) 0x5210))


//------------------------
// Module ITC
//------------------------

/** struct containing ITC module registers */
typedef struct {

  /** Interrupt software priority register 0 (SPR0 at 0x7f70) */
  union {

    /// bytewise access to SPR0
    uint8_t  byte;

    /// bitwise access to register SPR0
    struct {
      BITS                       : 2;      // 2 bits
      BITS   VECT1SPR            : 2;      // bits 2-3
      BITS   VECT2SPR            : 2;      // bits 4-5
      BITS   VECT3SPR            : 2;      // bits 6-7
    };  // SPR0 bitfield

    /// register _ITC_SPR0 reset value
    #define sfr_ITC_SPR0_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR0;


  /** Interrupt software priority register 1 (SPR1 at 0x7f71) */
  union {

    /// bytewise access to SPR1
    uint8_t  byte;

    /// bitwise access to register SPR1
    struct {
      BITS   VECT4SPR            : 2;      // bits 0-1
      BITS   VECT5SPR            : 2;      // bits 2-3
      BITS   VECT6SPR            : 2;      // bits 4-5
      BITS   VECT7SPR            : 2;      // bits 6-7
    };  // SPR1 bitfield

    /// register _ITC_SPR1 reset value
    #define sfr_ITC_SPR1_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR1;


  /** Interrupt software priority register 2 (SPR2 at 0x7f72) */
  union {

    /// bytewise access to SPR2
    uint8_t  byte;

    /// bitwise access to register SPR2
    struct {
      BITS   VECT8SPR            : 2;      // bits 0-1
      BITS   VECT9SPR            : 2;      // bits 2-3
      BITS   VECT10SPR           : 2;      // bits 4-5
      BITS   VECT11SPR           : 2;      // bits 6-7
    };  // SPR2 bitfield

    /// register _ITC_SPR2 reset value
    #define sfr_ITC_SPR2_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR2;


  /** Interrupt software priority register 3 (SPR3 at 0x7f73) */
  union {

    /// bytewise access to SPR3
    uint8_t  byte;

    /// bitwise access to register SPR3
    struct {
      BITS   VECT12SPR           : 2;      // bits 0-1
      BITS   VECT13SPR           : 2;      // bits 2-3
      BITS   VECT14SPR           : 2;      // bits 4-5
      BITS   VECT15SPR           : 2;      // bits 6-7
    };  // SPR3 bitfield

    /// register _ITC_SPR3 reset value
    #define sfr_ITC_SPR3_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR3;


  /** Interrupt software priority register 4 (SPR4 at 0x7f74) */
  union {

    /// bytewise access to SPR4
    uint8_t  byte;

    /// bitwise access to register SPR4
    struct {
      BITS   VECT16SPR           : 2;      // bits 0-1
      BITS   VECT17SPR           : 2;      // bits 2-3
      BITS   VECT18SPR           : 2;      // bits 4-5
      BITS   VECT19SPR           : 2;      // bits 6-7
    };  // SPR4 bitfield

    /// register _ITC_SPR4 reset value
    #define sfr_ITC_SPR4_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR4;


  /** Interrupt software priority register 5 (SPR5 at 0x7f75) */
  union {

    /// bytewise access to SPR5
    uint8_t  byte;

    /// bitwise access to register SPR5
    struct {
      BITS   VECT20SPR           : 2;      // bits 0-1
      BITS   VECT21SPR           : 2;      // bits 2-3
      BITS   VECT22SPR           : 2;      // bits 4-5
      BITS   VECT23SPR           : 2;      // bits 6-7
    };  // SPR5 bitfield

    /// register _ITC_SPR5 reset value
    #define sfr_ITC_SPR5_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR5;


  /** Interrupt software priority register 6 (SPR6 at 0x7f76) */
  union {

    /// bytewise access to SPR6
    uint8_t  byte;

    /// bitwise access to register SPR6
    struct {
      BITS   VECT24SPR           : 2;      // bits 0-1
      BITS   VECT25SPR           : 2;      // bits 2-3
      BITS   VECT26SPR           : 2;      // bits 4-5
      BITS   VECT27SPR           : 2;      // bits 6-7
    };  // SPR6 bitfield

    /// register _ITC_SPR6 reset value
    #define sfr_ITC_SPR6_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR6;


  /** Interrupt software priority register 7 (SPR7 at 0x7f77) */
  union {

    /// bytewise access to SPR7
    uint8_t  byte;

    /// bitwise access to register SPR7
    struct {
      BITS   VECT28SPR           : 2;      // bits 0-1
      BITS   VECT29SPR           : 2;      // bits 2-3
      BITS                       : 4;      // 4 bits
    };  // SPR7 bitfield

    /// register _ITC_SPR7 reset value
    #define sfr_ITC_SPR7_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR7;

} ITC_t;

/// access to ITC SFR registers
#define sfr_ITC   (*((ITC_t*) 0x7f70))


//------------------------
// Module MSC
//------------------------

/** struct containing MSC module registers */
typedef struct {

  /** Control register for P0-0 input line (MSC_CFGP00 at 0x5010) */
  union {

    /// bytewise access to MSC_CFGP00
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP00
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP00 bitfield

    /// register _MSC_MSC_CFGP00 reset value
    #define sfr_MSC_MSC_CFGP00_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP00;


  /** Control register for P0-1 input line (MSC_CFGP01 at 0x5011) */
  union {

    /// bytewise access to MSC_CFGP01
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP01
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP01 bitfield

    /// register _MSC_MSC_CFGP01 reset value
    #define sfr_MSC_MSC_CFGP01_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP01;


  /** Control register for P0-2 input line (MSC_CFGP02 at 0x5012) */
  union {

    /// bytewise access to MSC_CFGP02
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP02
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP02 bitfield

    /// register _MSC_MSC_CFGP02 reset value
    #define sfr_MSC_MSC_CFGP02_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP02;


  /** Control register for P0-3 input line (MSC_CFGP03 at 0x5013) */
  union {

    /// bytewise access to MSC_CFGP03
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP03
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP03 bitfield

    /// register _MSC_MSC_CFGP03 reset value
    #define sfr_MSC_MSC_CFGP03_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP03;


  /** Control register for P0-4 input line (MSC_CFGP04 at 0x5014) */
  union {

    /// bytewise access to MSC_CFGP04
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP04
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP04 bitfield

    /// register _MSC_MSC_CFGP04 reset value
    #define sfr_MSC_MSC_CFGP04_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP04;


  /** Control register for P0-5 input line (MSC_CFGP05 at 0x5015) */
  union {

    /// bytewise access to MSC_CFGP05
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP05
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP05 bitfield

    /// register _MSC_MSC_CFGP05 reset value
    #define sfr_MSC_MSC_CFGP05_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP05;


  /** Control register for P2-0 input line (MSC_CFGP20 at 0x5016) */
  union {

    /// bytewise access to MSC_CFGP20
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP20
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP20 bitfield

    /// register _MSC_MSC_CFGP20 reset value
    #define sfr_MSC_MSC_CFGP20_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP20;


  /** Control register for P2-1 input line (MSC_CFGP21 at 0x5017) */
  union {

    /// bytewise access to MSC_CFGP21
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP21
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP21 bitfield

    /// register _MSC_MSC_CFGP21 reset value
    #define sfr_MSC_MSC_CFGP21_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP21;


  /** Control register for P2-2 input line (MSC_CFGP22 at 0x5018) */
  union {

    /// bytewise access to MSC_CFGP22
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP22
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP22 bitfield

    /// register _MSC_MSC_CFGP22 reset value
    #define sfr_MSC_MSC_CFGP22_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP22;


  /** Control register for P2-3 input line (MSC_CFGP23 at 0x5019) */
  union {

    /// bytewise access to MSC_CFGP23
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP23
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP23 bitfield

    /// register _MSC_MSC_CFGP23 reset value
    #define sfr_MSC_MSC_CFGP23_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP23;


  /** Control register for P2-4 input line (MSC_CFGP24 at 0x501a) */
  union {

    /// bytewise access to MSC_CFGP24
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP24
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP24 bitfield

    /// register _MSC_MSC_CFGP24 reset value
    #define sfr_MSC_MSC_CFGP24_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP24;


  /** Control register for P2-5 input line (MSC_CFGP25 at 0x501b) */
  union {

    /// bytewise access to MSC_CFGP25
    uint8_t  byte;

    /// bitwise access to register MSC_CFGP25
    struct {
      BITS   INT_LEV             : 1;      // bit 0
      BITS   INT_SEL0            : 1;      // bit 1
      BITS   INT_SEL1            : 1;      // bit 2
      BITS   INT_ENB             : 1;      // bit 3
      BITS   INT_TYPE            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // MSC_CFGP25 bitfield

    /// register _MSC_MSC_CFGP25 reset value
    #define sfr_MSC_MSC_CFGP25_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP25;


  /** Port 0 Status register (MSC_STSP0 at 0x501c) */
  union {

    /// bytewise access to MSC_STSP0
    uint8_t  byte;

    /// bitwise access to register MSC_STSP0
    struct {
      BITS   BIT_0_INT           : 1;      // bit 0
      BITS   BIT_1_INT           : 1;      // bit 1
      BITS   BIT_2_INT           : 1;      // bit 2
      BITS   BIT_3_INT           : 1;      // bit 3
      BITS   BIT_4_INT           : 1;      // bit 4
      BITS   BIT_5_INT           : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_STSP0 bitfield

    /// register _MSC_MSC_STSP0 reset value
    #define sfr_MSC_MSC_STSP0_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_STSP0;


  /** Port 2 Status register (MSC_STSP2 at 0x501d) */
  union {

    /// bytewise access to MSC_STSP2
    uint8_t  byte;

    /// bitwise access to register MSC_STSP2
    struct {
      BITS   BIT_0_INT           : 1;      // bit 0
      BITS   BIT_1_INT           : 1;      // bit 1
      BITS   BIT_2_INT           : 1;      // bit 2
      BITS   BIT_3_INT           : 1;      // bit 3
      BITS   BIT_4_INT           : 1;      // bit 4
      BITS   BIT_5_INT           : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_STSP2 bitfield

    /// register _MSC_MSC_STSP2 reset value
    #define sfr_MSC_MSC_STSP2_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_STSP2;


  /** Port 2 inputs read register (MSC_INPP2 at 0x501e) */
  union {

    /// bytewise access to MSC_INPP2
    uint8_t  byte;

    /// bitwise access to register MSC_INPP2
    struct {
      BITS   DATA_IN             : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // MSC_INPP2 bitfield

    /// register _MSC_MSC_INPP2 reset value
    #define sfr_MSC_MSC_INPP2_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_INPP2;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** Enable DACs (MSC_DACCTR at 0x5020) */
  union {

    /// bytewise access to MSC_DACCTR
    uint8_t  byte;

    /// bitwise access to register MSC_DACCTR
    struct {
      BITS   DAC0_EN             : 1;      // bit 0
      BITS   DAC1_EN             : 1;      // bit 1
      BITS   DAC2_EN             : 1;      // bit 2
      BITS   DAC3_EN             : 1;      // bit 3
      BITS   CP3_EN              : 1;      // bit 4
      BITS   CP3_SEL             : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   DACBIAS_EN          : 1;      // bit 7
    };  // MSC_DACCTR bitfield

    /// register _MSC_MSC_DACCTR reset value
    #define sfr_MSC_MSC_DACCTR_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_DACCTR;


  /** Four bit input value for DAC 0 (MSC_DACIN0 at 0x5021) */
  union {

    /// bytewise access to MSC_DACIN0
    uint8_t  byte;

    /// bitwise access to register MSC_DACIN0
    struct {
      BITS   DAC_IN0             : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // MSC_DACIN0 bitfield

    /// register _MSC_MSC_DACIN0 reset value
    #define sfr_MSC_MSC_DACIN0_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_DACIN0;


  /** Four bit input value for DAC 1 (MSC_DACIN1 at 0x5022) */
  union {

    /// bytewise access to MSC_DACIN1
    uint8_t  byte;

    /// bitwise access to register MSC_DACIN1
    struct {
      BITS   DAC_IN1             : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // MSC_DACIN1 bitfield

    /// register _MSC_MSC_DACIN1 reset value
    #define sfr_MSC_MSC_DACIN1_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_DACIN1;


  /** Four bit input value for DAC 2 (MSC_DACIN2 at 0x5023) */
  union {

    /// bytewise access to MSC_DACIN2
    uint8_t  byte;

    /// bitwise access to register MSC_DACIN2
    struct {
      BITS   DAC_IN2             : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // MSC_DACIN2 bitfield

    /// register _MSC_MSC_DACIN2 reset value
    #define sfr_MSC_MSC_DACIN2_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_DACIN2;


  /** Four bit input value for DAC 3 (MSC_DACIN3 at 0x5024) */
  union {

    /// bytewise access to MSC_DACIN3
    uint8_t  byte;

    /// bitwise access to register MSC_DACIN3
    struct {
      BITS   DAC_IN3             : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // MSC_DACIN3 bitfield

    /// register _MSC_MSC_DACIN3 reset value
    #define sfr_MSC_MSC_DACIN3_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_DACIN3;


  /** SMED0,1 global configuration reg (MSC_SMDCFG01 at 0x5025) */
  union {

    /// bytewise access to MSC_SMDCFG01
    uint8_t  byte;

    /// bitwise access to register MSC_SMDCFG01
    struct {
      BITS   SMED0_GLBCONF       : 4;      // bits 0-3
      BITS   SMED1_GLBCONF       : 4;      // bits 4-7
    };  // MSC_SMDCFG01 bitfield

    /// register _MSC_MSC_SMDCFG01 reset value
    #define sfr_MSC_MSC_SMDCFG01_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_SMDCFG01;


  /** SMED2,3 global configuration reg (MSC_SMDCFG23 at 0x5026) */
  union {

    /// bytewise access to MSC_SMDCFG23
    uint8_t  byte;

    /// bitwise access to register MSC_SMDCFG23
    struct {
      BITS   SMED2_GLBCONF       : 4;      // bits 0-3
      BITS   SMED3_GLBCONF       : 4;      // bits 4-7
    };  // MSC_SMDCFG23 bitfield

    /// register _MSC_MSC_SMDCFG23 reset value
    #define sfr_MSC_MSC_SMDCFG23_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_SMDCFG23;


  /** SMED4,5 global configuration reg (MSC_SMDCFG45 at 0x5027) */
  union {

    /// bytewise access to MSC_SMDCFG45
    uint8_t  byte;

    /// bitwise access to register MSC_SMDCFG45
    struct {
      BITS   SMED4_GLBCONF       : 4;      // bits 0-3
      BITS   SMED5_GLBCONF       : 4;      // bits 4-7
    };  // MSC_SMDCFG45 bitfield

    /// register _MSC_MSC_SMDCFG45 reset value
    #define sfr_MSC_MSC_SMDCFG45_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_SMDCFG45;


  /** Software events for all SMED (MSC_SMSWEV at 0x5028) */
  union {

    /// bytewise access to MSC_SMSWEV
    uint8_t  byte;

    /// bitwise access to register MSC_SMSWEV
    struct {
      BITS   SW0                 : 1;      // bit 0
      BITS   SW1                 : 1;      // bit 1
      BITS   SW2                 : 1;      // bit 2
      BITS   SW3                 : 1;      // bit 3
      BITS   SW4                 : 1;      // bit 4
      BITS   SW5                 : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_SMSWEV bitfield

    /// register _MSC_MSC_SMSWEV reset value
    #define sfr_MSC_MSC_SMSWEV_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_SMSWEV;


  /** Unlock controls for SMED (MSC_SMULOCK at 0x5029) */
  union {

    /// bytewise access to MSC_SMULOCK
    uint8_t  byte;

    /// bitwise access to register MSC_SMULOCK
    struct {
      BITS   USE_UNLOCK_01       : 1;      // bit 0
      BITS   USE_UNLOCK_23       : 1;      // bit 1
      BITS   USE_UNLOCK_45       : 1;      // bit 2
      BITS   UNLOCK_01           : 1;      // bit 3
      BITS   UNLOCK_23           : 1;      // bit 4
      BITS   UNLOCK_45           : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_SMULOCK bitfield

    /// register _MSC_MSC_SMULOCK reset value
    #define sfr_MSC_MSC_SMULOCK_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_SMULOCK;


  /** Connect Box selection for SMED0 (MSC_CBOXS0 at 0x502a) */
  union {

    /// bytewise access to MSC_CBOXS0
    uint8_t  byte;

    /// bitwise access to register MSC_CBOXS0
    struct {
      BITS   CONB_S0_00          : 1;      // bit 0
      BITS   CONB_S0_01          : 1;      // bit 1
      BITS   CONB_S0_10          : 1;      // bit 2
      BITS   CONB_S0_11          : 1;      // bit 3
      BITS   CONB_S0_20          : 1;      // bit 4
      BITS   CONB_S0_21          : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_CBOXS0 bitfield

    /// register _MSC_MSC_CBOXS0 reset value
    #define sfr_MSC_MSC_CBOXS0_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CBOXS0;


  /** Connect Box selection for SMED1 (MSC_CBOXS1 at 0x502b) */
  union {

    /// bytewise access to MSC_CBOXS1
    uint8_t  byte;

    /// bitwise access to register MSC_CBOXS1
    struct {
      BITS   CONB_S1_00          : 1;      // bit 0
      BITS   CONB_S1_01          : 1;      // bit 1
      BITS   CONB_S1_10          : 1;      // bit 2
      BITS   CONB_S1_11          : 1;      // bit 3
      BITS   CONB_S1_20          : 1;      // bit 4
      BITS   CONB_S1_21          : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_CBOXS1 bitfield

    /// register _MSC_MSC_CBOXS1 reset value
    #define sfr_MSC_MSC_CBOXS1_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CBOXS1;


  /** Connect Box selection for SMED2 (MSC_CBOXS2 at 0x502c) */
  union {

    /// bytewise access to MSC_CBOXS2
    uint8_t  byte;

    /// bitwise access to register MSC_CBOXS2
    struct {
      BITS   CONB_S2_00          : 1;      // bit 0
      BITS   CONB_S2_01          : 1;      // bit 1
      BITS   CONB_S2_10          : 1;      // bit 2
      BITS   CONB_S2_11          : 1;      // bit 3
      BITS   CONB_S2_20          : 1;      // bit 4
      BITS   CONB_S2_21          : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_CBOXS2 bitfield

    /// register _MSC_MSC_CBOXS2 reset value
    #define sfr_MSC_MSC_CBOXS2_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CBOXS2;


  /** Connect Box selection for SMED3 (MSC_CBOXS3 at 0x502d) */
  union {

    /// bytewise access to MSC_CBOXS3
    uint8_t  byte;

    /// bitwise access to register MSC_CBOXS3
    struct {
      BITS   CONB_S3_00          : 1;      // bit 0
      BITS   CONB_S3_01          : 1;      // bit 1
      BITS   CONB_S3_10          : 1;      // bit 2
      BITS   CONB_S3_11          : 1;      // bit 3
      BITS   CONB_S3_20          : 1;      // bit 4
      BITS   CONB_S3_21          : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_CBOXS3 bitfield

    /// register _MSC_MSC_CBOXS3 reset value
    #define sfr_MSC_MSC_CBOXS3_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CBOXS3;


  /** Connect Box selection for SMED4 (MSC_CBOXS4 at 0x502e) */
  union {

    /// bytewise access to MSC_CBOXS4
    uint8_t  byte;

    /// bitwise access to register MSC_CBOXS4
    struct {
      BITS   CONB_S4_00          : 1;      // bit 0
      BITS   CONB_S4_01          : 1;      // bit 1
      BITS   CONB_S4_10          : 1;      // bit 2
      BITS   CONB_S4_11          : 1;      // bit 3
      BITS   CONB_S4_20          : 1;      // bit 4
      BITS   CONB_S4_21          : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_CBOXS4 bitfield

    /// register _MSC_MSC_CBOXS4 reset value
    #define sfr_MSC_MSC_CBOXS4_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CBOXS4;


  /** Connect Box selection for SMED5 (MSC_CBOXS5 at 0x502f) */
  union {

    /// bytewise access to MSC_CBOXS5
    uint8_t  byte;

    /// bitwise access to register MSC_CBOXS5
    struct {
      BITS   CONB_S5_00          : 1;      // bit 0
      BITS   CONB_S5_01          : 1;      // bit 1
      BITS   CONB_S5_10          : 1;      // bit 2
      BITS   CONB_S5_11          : 1;      // bit 3
      BITS   CONB_S5_20          : 1;      // bit 4
      BITS   CONB_S5_21          : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_CBOXS5 bitfield

    /// register _MSC_MSC_CBOXS5 reset value
    #define sfr_MSC_MSC_CBOXS5_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CBOXS5;


  /** I/O mux Port0 P0 SMED FSM state (MSC_IOMXSMD at 0x5030) */
  union {

    /// bytewise access to MSC_IOMXSMD
    uint8_t  byte;

    /// bitwise access to register MSC_IOMXSMD
    struct {
      BITS   SMD_FSML0           : 3;      // bits 0-2
      BITS   SEL_FSMEN0          : 1;      // bit 3
      BITS   SMD_FSML1           : 3;      // bits 4-6
      BITS   SEL_FSMEN1          : 1;      // bit 7
    };  // MSC_IOMXSMD bitfield

    /// register _MSC_MSC_IOMXSMD reset value
    #define sfr_MSC_MSC_IOMXSMD_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_IOMXSMD;


  /// Reserved register (5B)
  uint8_t     Reserved_2[5];


  /** Control register for AUXTIM input line (MSC_CFGP15 at 0x5036) */
  union {

    /// bytewise access to MSC_CFGP15
    uint8_t  byte;

    /// skip bitwise access to register MSC_CFGP15

    /// register _MSC_MSC_CFGP15 reset value
    #define sfr_MSC_MSC_CFGP15_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_CFGP15;


  /** AUXTIM status register (MSC_STSP1 at 0x5037) */
  union {

    /// bytewise access to MSC_STSP1
    uint8_t  byte;

    /// skip bitwise access to register MSC_STSP1

    /// register _MSC_MSC_STSP1 reset value
    #define sfr_MSC_MSC_STSP1_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_STSP1;


  /// Reserved register (1B)
  uint8_t     Reserved_3[1];


  /** Port 3 inputs read register (MSC_INPP3 at 0x5039) */
  union {

    /// bytewise access to MSC_INPP3
    uint8_t  byte;

    /// bitwise access to register MSC_INPP3
    struct {
      BITS   COMP0               : 1;      // bit 0
      BITS   COMP1               : 1;      // bit 1
      BITS   COMP2               : 1;      // bit 2
      BITS   COMP3               : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // MSC_INPP3 bitfield

    /// register _MSC_MSC_INPP3 reset value
    #define sfr_MSC_MSC_INPP3_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_INPP3;


  /** I/O mux Port0 (GPIO0s) signals (MSC_IOMXP0 at 0x503a) */
  union {

    /// bytewise access to MSC_IOMXP0
    uint8_t  byte;

    /// bitwise access to register MSC_IOMXP0
    struct {
      BITS   SEL_P010            : 2;      // bits 0-1
      BITS   SEL_P032            : 2;      // bits 2-3
      BITS   SEL_P054            : 2;      // bits 4-5
      BITS                       : 2;      // 2 bits
    };  // MSC_IOMXP0 bitfield

    /// register _MSC_MSC_IOMXP0 reset value
    #define sfr_MSC_MSC_IOMXP0_RESET_VALUE   ((uint8_t) 0x11)

  } MSC_IOMXP0;


  /** I/O mux Port1 (PWMs/GPIO1s) signals (MSC_IOMXP1 at 0x503b) */
  union {

    /// bytewise access to MSC_IOMXP1
    uint8_t  byte;

    /// bitwise access to register MSC_IOMXP1
    struct {
      BITS   SEL_P10             : 1;      // bit 0
      BITS   SEL_P11             : 1;      // bit 1
      BITS   SEL_P12             : 1;      // bit 2
      BITS   SEL_P13             : 1;      // bit 3
      BITS   SEL_P14             : 1;      // bit 4
      BITS   SEL_P15             : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // MSC_IOMXP1 bitfield

    /// register _MSC_MSC_IOMXP1 reset value
    #define sfr_MSC_MSC_IOMXP1_RESET_VALUE   ((uint8_t) 0x3F)

  } MSC_IOMXP1;


  /** Miscellaneous indirect address (MSC_IDXADD at 0x503c) */
  union {

    /// bytewise access to MSC_IDXADD
    uint8_t  byte;

    /// skip bitwise access to register MSC_IDXADD

    /// register _MSC_MSC_IDXADD reset value
    #define sfr_MSC_MSC_IDXADD_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_IDXADD;


  /** Miscellaneous indirect data (MSC_IDXDAT at 0x503d) */
  union {

    /// bytewise access to MSC_IDXDAT
    uint8_t  byte;

    /// skip bitwise access to register MSC_IDXDAT

    /// register _MSC_MSC_IDXDAT reset value
    #define sfr_MSC_MSC_IDXDAT_RESET_VALUE   ((uint8_t) 0x00)

  } MSC_IDXDAT;

} MSC_t;

/// access to MSC SFR registers
#define sfr_MSC   (*((MSC_t*) 0x5010))


//------------------------
// Module OPT
//------------------------

/** struct containing OPT module registers */
typedef struct {

  /** Read-out protection register (ROP at 0x4800) */
  union {

    /// bytewise access to ROP
    uint8_t  byte;

    /// skip bitwise access to register ROP

    /// register _OPT_ROP reset value
    #define sfr_OPT_ROP_RESET_VALUE   ((uint8_t) 0x00)

  } ROP;


  /** User boot code register (UCB at 0x4801) */
  union {

    /// bytewise access to UCB
    uint8_t  byte;

    /// skip bitwise access to register UCB

    /// register _OPT_UCB reset value
    #define sfr_OPT_UCB_RESET_VALUE   ((uint8_t) 0x00)

  } UCB;


  /** User boot code register (complementary byte) (NUCB at 0x4802) */
  union {

    /// bytewise access to NUCB
    uint8_t  byte;

    /// skip bitwise access to register NUCB

    /// register _OPT_NUCB reset value
    #define sfr_OPT_NUCB_RESET_VALUE   ((uint8_t) 0xFF)

  } NUCB;


  /** General configuration register (GENCFG at 0x4803) */
  union {

    /// bytewise access to GENCFG
    uint8_t  byte;

    /// skip bitwise access to register GENCFG

    /// register _OPT_GENCFG reset value
    #define sfr_OPT_GENCFG_RESET_VALUE   ((uint8_t) 0x00)

  } GENCFG;


  /** General configuration register (complementary byte) (NGENCFG at 0x4804) */
  union {

    /// bytewise access to NGENCFG
    uint8_t  byte;

    /// skip bitwise access to register NGENCFG

    /// register _OPT_NGENCFG reset value
    #define sfr_OPT_NGENCFG_RESET_VALUE   ((uint8_t) 0xFF)

  } NGENCFG;


  /** Misc. configuration register (MISCUOPT at 0x4805) */
  union {

    /// bytewise access to MISCUOPT
    uint8_t  byte;

    /// skip bitwise access to register MISCUOPT

    /// register _OPT_MISCUOPT reset value
    #define sfr_OPT_MISCUOPT_RESET_VALUE   ((uint8_t) 0x28)

  } MISCUOPT;


  /** Misc. configuration register (complementary byte) (NMISCUOPT at 0x4806) */
  union {

    /// bytewise access to NMISCUOPT
    uint8_t  byte;

    /// skip bitwise access to register NMISCUOPT

    /// register _OPT_NMISCUOPT reset value
    #define sfr_OPT_NMISCUOPT_RESET_VALUE   ((uint8_t) 0xD7)

  } NMISCUOPT;


  /** CKC configuration register (CLKCTL at 0x4807) */
  union {

    /// bytewise access to CLKCTL
    uint8_t  byte;

    /// skip bitwise access to register CLKCTL

    /// register _OPT_CLKCTL reset value
    #define sfr_OPT_CLKCTL_RESET_VALUE   ((uint8_t) 0x09)

  } CLKCTL;


  /** CKC configuration register (complementary byte) (NCLKCTL at 0x4808) */
  union {

    /// bytewise access to NCLKCTL
    uint8_t  byte;

    /// skip bitwise access to register NCLKCTL

    /// register _OPT_NCLKCTL reset value
    #define sfr_OPT_NCLKCTL_RESET_VALUE   ((uint8_t) 0xF6)

  } NCLKCTL;


  /** HSE clock stabilization register (HSESTAB at 0x4809) */
  union {

    /// bytewise access to HSESTAB
    uint8_t  byte;

    /// skip bitwise access to register HSESTAB

    /// register _OPT_HSESTAB reset value
    #define sfr_OPT_HSESTAB_RESET_VALUE   ((uint8_t) 0x00)

  } HSESTAB;


  /** HSE clock stabilization register (complementary byte) (NHSESTAB at 0x480a) */
  union {

    /// bytewise access to NHSESTAB
    uint8_t  byte;

    /// skip bitwise access to register NHSESTAB

    /// register _OPT_NHSESTAB reset value
    #define sfr_OPT_NHSESTAB_RESET_VALUE   ((uint8_t) 0xFF)

  } NHSESTAB;


  /// Reserved register (2B)
  uint8_t     Reserved_1[2];


  /** Flash wait state register (WAITSTATE at 0x480d) */
  union {

    /// bytewise access to WAITSTATE
    uint8_t  byte;

    /// skip bitwise access to register WAITSTATE

    /// register _OPT_WAITSTATE reset value
    #define sfr_OPT_WAITSTATE_RESET_VALUE   ((uint8_t) 0x00)

  } WAITSTATE;


  /** Flash wait state register (complementary byte) (NWAITSTATE at 0x480e) */
  union {

    /// bytewise access to NWAITSTATE
    uint8_t  byte;

    /// skip bitwise access to register NWAITSTATE

    /// register _OPT_NWAITSTATE reset value
    #define sfr_OPT_NWAITSTATE_RESET_VALUE   ((uint8_t) 0xFF)

  } NWAITSTATE;


  /** Alternative Port 0 configuration register (AFR_IOMXP0 at 0x480f) */
  union {

    /// bytewise access to AFR_IOMXP0
    uint8_t  byte;

    /// skip bitwise access to register AFR_IOMXP0

    /// register _OPT_AFR_IOMXP0 reset value
    #define sfr_OPT_AFR_IOMXP0_RESET_VALUE   ((uint8_t) 0x00)

  } AFR_IOMXP0;


  /** Alternative Port 0 configuration register (complementary byte) (NAFR_IOMXP0 at 0x4810) */
  union {

    /// bytewise access to NAFR_IOMXP0
    uint8_t  byte;

    /// skip bitwise access to register NAFR_IOMXP0

    /// register _OPT_NAFR_IOMXP0 reset value
    #define sfr_OPT_NAFR_IOMXP0_RESET_VALUE   ((uint8_t) 0xFF)

  } NAFR_IOMXP0;


  /** Alternative Port 1 configuration register (AFR_IOMXP1 at 0x4811) */
  union {

    /// bytewise access to AFR_IOMXP1
    uint8_t  byte;

    /// skip bitwise access to register AFR_IOMXP1

    /// register _OPT_AFR_IOMXP1 reset value
    #define sfr_OPT_AFR_IOMXP1_RESET_VALUE   ((uint8_t) 0x00)

  } AFR_IOMXP1;


  /** Alternative Port 1 configuration register (complementary byte) (NAFR_IOMXP1 at 0x4812) */
  union {

    /// bytewise access to NAFR_IOMXP1
    uint8_t  byte;

    /// skip bitwise access to register NAFR_IOMXP1

    /// register _OPT_NAFR_IOMXP1 reset value
    #define sfr_OPT_NAFR_IOMXP1_RESET_VALUE   ((uint8_t) 0xFF)

  } NAFR_IOMXP1;


  /** Alternative Port 2 configuration register (AFR_IOMXP2 at 0x4813) */
  union {

    /// bytewise access to AFR_IOMXP2
    uint8_t  byte;

    /// skip bitwise access to register AFR_IOMXP2

    /// register _OPT_AFR_IOMXP2 reset value
    #define sfr_OPT_AFR_IOMXP2_RESET_VALUE   ((uint8_t) 0x10)

  } AFR_IOMXP2;


  /** Alternative Port 2 configuration register (complementary byte) (NAFR_IOMXP2 at 0x4814) */
  union {

    /// bytewise access to NAFR_IOMXP2
    uint8_t  byte;

    /// skip bitwise access to register NAFR_IOMXP2

    /// register _OPT_NAFR_IOMXP2 reset value
    #define sfr_OPT_NAFR_IOMXP2_RESET_VALUE   ((uint8_t) 0xEF)

  } NAFR_IOMXP2;


  /** Misc. configuration register 0 (MSC_OPT0 at 0x4815) */
  union {

    /// bytewise access to MSC_OPT0
    uint8_t  byte;

    /// skip bitwise access to register MSC_OPT0

    /// register _OPT_MSC_OPT0 reset value
    #define sfr_OPT_MSC_OPT0_RESET_VALUE   ((uint8_t) 0x01)

  } MSC_OPT0;


  /** Misc. configuration register 0 (complementary byte) (NMSC_OPT0 at 0x4816) */
  union {

    /// bytewise access to NMSC_OPT0
    uint8_t  byte;

    /// skip bitwise access to register NMSC_OPT0

    /// register _OPT_NMSC_OPT0 reset value
    #define sfr_OPT_NMSC_OPT0_RESET_VALUE   ((uint8_t) 0xFE)

  } NMSC_OPT0;


  /// Reserved register (103B)
  uint8_t     Reserved_2[103];


  /** Bootloader register (OPTBL at 0x487e) */
  union {

    /// bytewise access to OPTBL
    uint8_t  byte;

    /// skip bitwise access to register OPTBL

    /// register _OPT_OPTBL reset value
    #define sfr_OPT_OPTBL_RESET_VALUE   ((uint8_t) 0x00)

  } OPTBL;


  /** Bootloader register (complementary byte) (NOPTBL at 0x487f) */
  union {

    /// bytewise access to NOPTBL
    uint8_t  byte;

    /// skip bitwise access to register NOPTBL

    /// register _OPT_NOPTBL reset value
    #define sfr_OPT_NOPTBL_RESET_VALUE   ((uint8_t) 0xFF)

  } NOPTBL;

} OPT_t;

/// access to OPT SFR registers
#define sfr_OPT   (*((OPT_t*) 0x4800))


//------------------------
// Module PORT
//------------------------

/** struct containing PORT0 module registers */
typedef struct {

  /** Port 0 output register (ODR at 0x5000) */
  union {

    /// bytewise access to ODR
    uint8_t  byte;

    /// bitwise access to register ODR
    struct {
      BITS   ODR0                : 1;      // bit 0
      BITS   ODR1                : 1;      // bit 1
      BITS   ODR2                : 1;      // bit 2
      BITS   ODR3                : 1;      // bit 3
      BITS   ODR4                : 1;      // bit 4
      BITS   ODR5                : 1;      // bit 5
      BITS   ODR6                : 1;      // bit 6
      BITS   ODR7                : 1;      // bit 7
    };  // ODR bitfield

    /// register _PORT_ODR reset value
    #define sfr_PORT_ODR_RESET_VALUE   ((uint8_t) 0x00)

  } ODR;


  /** Port 0 input register (IDR at 0x5001) */
  union {

    /// bytewise access to IDR
    uint8_t  byte;

    /// bitwise access to register IDR
    struct {
      BITS   IDR0                : 1;      // bit 0
      BITS   IDR1                : 1;      // bit 1
      BITS   IDR2                : 1;      // bit 2
      BITS   IDR3                : 1;      // bit 3
      BITS   IDR4                : 1;      // bit 4
      BITS   IDR5                : 1;      // bit 5
      BITS   IDR6                : 1;      // bit 6
      BITS   IDR7                : 1;      // bit 7
    };  // IDR bitfield

    /// register _PORT_IDR reset value
    #define sfr_PORT_IDR_RESET_VALUE   ((uint8_t) 0x00)

  } IDR;


  /** Port 0 data direction register (DDR at 0x5002) */
  union {

    /// bytewise access to DDR
    uint8_t  byte;

    /// bitwise access to register DDR
    struct {
      BITS   DDR0                : 1;      // bit 0
      BITS   DDR1                : 1;      // bit 1
      BITS   DDR2                : 1;      // bit 2
      BITS   DDR3                : 1;      // bit 3
      BITS   DDR4                : 1;      // bit 4
      BITS   DDR5                : 1;      // bit 5
      BITS   DDR6                : 1;      // bit 6
      BITS   DDR7                : 1;      // bit 7
    };  // DDR bitfield

    /// register _PORT_DDR reset value
    #define sfr_PORT_DDR_RESET_VALUE   ((uint8_t) 0x00)

  } DDR;


  /** Port 0 control register 1 (CR1 at 0x5003) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CR1_0               : 1;      // bit 0
      BITS   CR1_1               : 1;      // bit 1
      BITS   CR1_2               : 1;      // bit 2
      BITS   CR1_3               : 1;      // bit 3
      BITS   CR1_4               : 1;      // bit 4
      BITS   CR1_5               : 1;      // bit 5
      BITS   CR1_6               : 1;      // bit 6
      BITS   CR1_7               : 1;      // bit 7
    };  // CR1 bitfield

    /// register _PORT_CR1 reset value
    #define sfr_PORT_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Port 0 control register 2 (CR2 at 0x5004) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   CR2_0               : 1;      // bit 0
      BITS   CR2_1               : 1;      // bit 1
      BITS   CR2_2               : 1;      // bit 2
      BITS   CR2_3               : 1;      // bit 3
      BITS   CR2_4               : 1;      // bit 4
      BITS   CR2_5               : 1;      // bit 5
      BITS   CR2_6               : 1;      // bit 6
      BITS   CR2_7               : 1;      // bit 7
    };  // CR2 bitfield

    /// register _PORT_CR2 reset value
    #define sfr_PORT_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;

} PORT_t;

/// access to PORT0 SFR registers
#define sfr_PORT0   (*((PORT_t*) 0x5000))


/// access to PORT1 SFR registers
#define sfr_PORT1   (*((PORT_t*) 0x5005))


//------------------------
// Module RST
//------------------------

/** struct containing RST module registers */
typedef struct {

  /** Reset status register (RST_SR at 0x50b3) */
  union {

    /// bytewise access to RST_SR
    uint8_t  byte;

    /// bitwise access to register RST_SR
    struct {
      BITS   WWDGF               : 1;      // bit 0
      BITS   IWDGF               : 1;      // bit 1
      BITS   ILLOPF              : 1;      // bit 2
      BITS   SWIMF               : 1;      // bit 3
      BITS   EMCF                : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // RST_SR bitfield

    /// register _RST_RST_SR reset value
    #define sfr_RST_RST_SR_RESET_VALUE   ((uint8_t) 0x00)

  } RST_SR;

} RST_t;

/// access to RST SFR registers
#define sfr_RST   (*((RST_t*) 0x50b3))


//------------------------
// Module SMED0
//------------------------

/** struct containing SMED0 module registers */
typedef struct {

  /** Control register (CTR at 0x5500) */
  union {

    /// bytewise access to CTR
    uint8_t  byte;

    /// bitwise access to register CTR
    struct {
      BITS   START_CNT           : 1;      // bit 0
      BITS   FSM_ENA             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CTR bitfield

    /// register _SMED0_CTR reset value
    #define sfr_SMED0_CTR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR;


  /** Control timer register (CTR_TMR at 0x5501) */
  union {

    /// bytewise access to CTR_TMR
    uint8_t  byte;

    /// bitwise access to register CTR_TMR
    struct {
      BITS   TIME_T0_VAL         : 1;      // bit 0
      BITS   TIME_T1_VAL         : 1;      // bit 1
      BITS   TIME_T2_VAL         : 1;      // bit 2
      BITS   TIME_T3_VAL         : 1;      // bit 3
      BITS   DITHER_VAL          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CTR_TMR bitfield

    /// register _SMED0_CTR_TMR reset value
    #define sfr_SMED0_CTR_TMR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_TMR;


  /** Control input register (CTR_INP at 0x5502) */
  union {

    /// bytewise access to CTR_INP
    uint8_t  byte;

    /// bitwise access to register CTR_INP
    struct {
      BITS   RS_INSIG0           : 1;      // bit 0
      BITS   RS_INSIG1           : 1;      // bit 1
      BITS   RS_INSIG2           : 1;      // bit 2
      BITS   RAIS_EN             : 1;      // bit 3
      BITS   EL_INSIG0           : 1;      // bit 4
      BITS   EL_INSIG1           : 1;      // bit 5
      BITS   EL_INSIG2           : 1;      // bit 6
      BITS   EL_EN               : 1;      // bit 7
    };  // CTR_INP bitfield

    /// register _SMED0_CTR_INP reset value
    #define sfr_SMED0_CTR_INP_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_INP;


  /** Dithering register (CTR_DTHR at 0x5503) */
  union {

    /// bytewise access to CTR_DTHR
    uint8_t  byte;

    /// bitwise access to register CTR_DTHR
    struct {
      BITS   DITH0               : 1;      // bit 0
      BITS   DITH1               : 1;      // bit 1
      BITS   DITH2               : 1;      // bit 2
      BITS   DITH3               : 1;      // bit 3
      BITS   DITH4               : 1;      // bit 4
      BITS   DITH5               : 1;      // bit 5
      BITS   DITH6               : 1;      // bit 6
      BITS   DITH7               : 1;      // bit 7
    };  // CTR_DTHR bitfield

    /// register _SMED0_CTR_DTHR reset value
    #define sfr_SMED0_CTR_DTHR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_DTHR;


  /** Time T0 lsb register (TMR_T0L at 0x5504) */
  union {

    /// bytewise access to TMR_T0L
    uint8_t  byte;

    /// bitwise access to register TMR_T0L
    struct {
      BITS   TIM_T0L0            : 1;      // bit 0
      BITS   TIM_T0L1            : 1;      // bit 1
      BITS   TIM_T0L2            : 1;      // bit 2
      BITS   TIM_T0L3            : 1;      // bit 3
      BITS   TIM_T0L4            : 1;      // bit 4
      BITS   TIM_T0L5            : 1;      // bit 5
      BITS   TIM_T0L6            : 1;      // bit 6
      BITS   TIM_T0L7            : 1;      // bit 7
    };  // TMR_T0L bitfield

    /// register _SMED0_TMR_T0L reset value
    #define sfr_SMED0_TMR_T0L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0L;


  /** Time T0 msb register (TMR_T0H at 0x5505) */
  union {

    /// bytewise access to TMR_T0H
    uint8_t  byte;

    /// bitwise access to register TMR_T0H
    struct {
      BITS   TIM_T0H0            : 1;      // bit 0
      BITS   TIM_T0H1            : 1;      // bit 1
      BITS   TIM_T0H2            : 1;      // bit 2
      BITS   TIM_T0H3            : 1;      // bit 3
      BITS   TIM_T0H4            : 1;      // bit 4
      BITS   TIM_T0H5            : 1;      // bit 5
      BITS   TIM_T0H6            : 1;      // bit 6
      BITS   TIM_T0H7            : 1;      // bit 7
    };  // TMR_T0H bitfield

    /// register _SMED0_TMR_T0H reset value
    #define sfr_SMED0_TMR_T0H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0H;


  /** Time T1 lsb register (TMR_T1L at 0x5506) */
  union {

    /// bytewise access to TMR_T1L
    uint8_t  byte;

    /// bitwise access to register TMR_T1L
    struct {
      BITS   TIM_T1L0            : 1;      // bit 0
      BITS   TIM_T1L1            : 1;      // bit 1
      BITS   TIM_T1L2            : 1;      // bit 2
      BITS   TIM_T1L3            : 1;      // bit 3
      BITS   TIM_T1L4            : 1;      // bit 4
      BITS   TIM_T1L5            : 1;      // bit 5
      BITS   TIM_T1L6            : 1;      // bit 6
      BITS   TIM_T1L7            : 1;      // bit 7
    };  // TMR_T1L bitfield

    /// register _SMED0_TMR_T1L reset value
    #define sfr_SMED0_TMR_T1L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1L;


  /** Time T1 msb register (TMR_T1H at 0x5507) */
  union {

    /// bytewise access to TMR_T1H
    uint8_t  byte;

    /// bitwise access to register TMR_T1H
    struct {
      BITS   TIM_T1H0            : 1;      // bit 0
      BITS   TIM_T1H1            : 1;      // bit 1
      BITS   TIM_T1H2            : 1;      // bit 2
      BITS   TIM_T1H3            : 1;      // bit 3
      BITS   TIM_T1H4            : 1;      // bit 4
      BITS   TIM_T1H5            : 1;      // bit 5
      BITS   TIM_T1H6            : 1;      // bit 6
      BITS   TIM_T1H7            : 1;      // bit 7
    };  // TMR_T1H bitfield

    /// register _SMED0_TMR_T1H reset value
    #define sfr_SMED0_TMR_T1H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1H;


  /** Time T2 lsb register (TMR_T2L at 0x5508) */
  union {

    /// bytewise access to TMR_T2L
    uint8_t  byte;

    /// bitwise access to register TMR_T2L
    struct {
      BITS   TIM_T2L0            : 1;      // bit 0
      BITS   TIM_T2L1            : 1;      // bit 1
      BITS   TIM_T2L2            : 1;      // bit 2
      BITS   TIM_T2L3            : 1;      // bit 3
      BITS   TIM_T2L4            : 1;      // bit 4
      BITS   TIM_T2L5            : 1;      // bit 5
      BITS   TIM_T2L6            : 1;      // bit 6
      BITS   TIM_T2L7            : 1;      // bit 7
    };  // TMR_T2L bitfield

    /// register _SMED0_TMR_T2L reset value
    #define sfr_SMED0_TMR_T2L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2L;


  /** Time T2 msb register (TMR_T2H at 0x5509) */
  union {

    /// bytewise access to TMR_T2H
    uint8_t  byte;

    /// bitwise access to register TMR_T2H
    struct {
      BITS   TIM_T2H0            : 1;      // bit 0
      BITS   TIM_T2H1            : 1;      // bit 1
      BITS   TIM_T2H2            : 1;      // bit 2
      BITS   TIM_T2H3            : 1;      // bit 3
      BITS   TIM_T2H4            : 1;      // bit 4
      BITS   TIM_T2H5            : 1;      // bit 5
      BITS   TIM_T2H6            : 1;      // bit 6
      BITS   TIM_T2H7            : 1;      // bit 7
    };  // TMR_T2H bitfield

    /// register _SMED0_TMR_T2H reset value
    #define sfr_SMED0_TMR_T2H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2H;


  /** Time T3 lsb register (TMR_T3L at 0x550a) */
  union {

    /// bytewise access to TMR_T3L
    uint8_t  byte;

    /// bitwise access to register TMR_T3L
    struct {
      BITS   TIM_T3L0            : 1;      // bit 0
      BITS   TIM_T3L1            : 1;      // bit 1
      BITS   TIM_T3L2            : 1;      // bit 2
      BITS   TIM_T3L3            : 1;      // bit 3
      BITS   TIM_T3L4            : 1;      // bit 4
      BITS   TIM_T3L5            : 1;      // bit 5
      BITS   TIM_T3L6            : 1;      // bit 6
      BITS   TIM_T3L7            : 1;      // bit 7
    };  // TMR_T3L bitfield

    /// register _SMED0_TMR_T3L reset value
    #define sfr_SMED0_TMR_T3L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3L;


  /** Time T3 msb register (TMR_T3H at 0x550b) */
  union {

    /// bytewise access to TMR_T3H
    uint8_t  byte;

    /// bitwise access to register TMR_T3H
    struct {
      BITS   TIM_T3H0            : 1;      // bit 0
      BITS   TIM_T3H1            : 1;      // bit 1
      BITS   TIM_T3H2            : 1;      // bit 2
      BITS   TIM_T3H3            : 1;      // bit 3
      BITS   TIM_T3H4            : 1;      // bit 4
      BITS   TIM_T3H5            : 1;      // bit 5
      BITS   TIM_T3H6            : 1;      // bit 6
      BITS   TIM_T3H7            : 1;      // bit 7
    };  // TMR_T3H bitfield

    /// register _SMED0_TMR_T3H reset value
    #define sfr_SMED0_TMR_T3H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3H;


  /** Parameter 0 IDLE register (PRM_ID0 at 0x550c) */
  union {

    /// bytewise access to PRM_ID0
    uint8_t  byte;

    /// bitwise access to register PRM_ID0
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_ID0 bitfield

    /// register _SMED0_PRM_ID0 reset value
    #define sfr_SMED0_PRM_ID0_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID0;


  /** Parameter 1 IDLE register (PRM_ID1 at 0x550d) */
  union {

    /// bytewise access to PRM_ID1
    uint8_t  byte;

    /// bitwise access to register PRM_ID1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_ID1 bitfield

    /// register _SMED0_PRM_ID1 reset value
    #define sfr_SMED0_PRM_ID1_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID1;


  /** Parameter 2 IDLE register (PRM_ID2 at 0x550e) */
  union {

    /// bytewise access to PRM_ID2
    uint8_t  byte;

    /// bitwise access to register PRM_ID2
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_ID2 bitfield

    /// register _SMED0_PRM_ID2 reset value
    #define sfr_SMED0_PRM_ID2_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID2;


  /** Parameter 0 S0 register (PRM_S00 at 0x550f) */
  union {

    /// bytewise access to PRM_S00
    uint8_t  byte;

    /// bitwise access to register PRM_S00
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S00 bitfield

    /// register _SMED0_PRM_S00 reset value
    #define sfr_SMED0_PRM_S00_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S00;


  /** Parameter 1 S0 register (PRM_S01 at 0x5510) */
  union {

    /// bytewise access to PRM_S01
    uint8_t  byte;

    /// bitwise access to register PRM_S01
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S01 bitfield

    /// register _SMED0_PRM_S01 reset value
    #define sfr_SMED0_PRM_S01_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S01;


  /** Parameter 2 S0 register (PRM_S02 at 0x5511) */
  union {

    /// bytewise access to PRM_S02
    uint8_t  byte;

    /// bitwise access to register PRM_S02
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S02 bitfield

    /// register _SMED0_PRM_S02 reset value
    #define sfr_SMED0_PRM_S02_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S02;


  /** Parameter 0 S1 register (PRM_S10 at 0x5512) */
  union {

    /// bytewise access to PRM_S10
    uint8_t  byte;

    /// bitwise access to register PRM_S10
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S10 bitfield

    /// register _SMED0_PRM_S10 reset value
    #define sfr_SMED0_PRM_S10_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S10;


  /** Parameter 1 S1 register (PRM_S11 at 0x5513) */
  union {

    /// bytewise access to PRM_S11
    uint8_t  byte;

    /// bitwise access to register PRM_S11
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S11 bitfield

    /// register _SMED0_PRM_S11 reset value
    #define sfr_SMED0_PRM_S11_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S11;


  /** Parameter 2 S1 register (PRM_S12 at 0x5514) */
  union {

    /// bytewise access to PRM_S12
    uint8_t  byte;

    /// bitwise access to register PRM_S12
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S12 bitfield

    /// register _SMED0_PRM_S12 reset value
    #define sfr_SMED0_PRM_S12_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S12;


  /** Parameter 0 S2 register (PRM_S20 at 0x5515) */
  union {

    /// bytewise access to PRM_S20
    uint8_t  byte;

    /// bitwise access to register PRM_S20
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S20 bitfield

    /// register _SMED0_PRM_S20 reset value
    #define sfr_SMED0_PRM_S20_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S20;


  /** Parameter 1 S2 register (PRM_S21 at 0x5516) */
  union {

    /// bytewise access to PRM_S21
    uint8_t  byte;

    /// bitwise access to register PRM_S21
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S21 bitfield

    /// register _SMED0_PRM_S21 reset value
    #define sfr_SMED0_PRM_S21_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S21;


  /** Parameter 2 S2 register (PRM_S22 at 0x5517) */
  union {

    /// bytewise access to PRM_S22
    uint8_t  byte;

    /// bitwise access to register PRM_S22
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S22 bitfield

    /// register _SMED0_PRM_S22 reset value
    #define sfr_SMED0_PRM_S22_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S22;


  /** Parameter 0 S3 register (PRM_S30 at 0x5518) */
  union {

    /// bytewise access to PRM_S30
    uint8_t  byte;

    /// bitwise access to register PRM_S30
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S30 bitfield

    /// register _SMED0_PRM_S30 reset value
    #define sfr_SMED0_PRM_S30_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S30;


  /** Parameter 1 S3 register (PRM_S31 at 0x5519) */
  union {

    /// bytewise access to PRM_S31
    uint8_t  byte;

    /// bitwise access to register PRM_S31
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S31 bitfield

    /// register _SMED0_PRM_S31 reset value
    #define sfr_SMED0_PRM_S31_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S31;


  /** Parameter 2 S3 register (PRM_S32 at 0x551a) */
  union {

    /// bytewise access to PRM_S32
    uint8_t  byte;

    /// bitwise access to register PRM_S32
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S32 bitfield

    /// register _SMED0_PRM_S32 reset value
    #define sfr_SMED0_PRM_S32_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S32;


  /** Timer configuration register (CFG at 0x551b) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TIM_NUM0            : 1;      // bit 1
      BITS   TIM_NUM1            : 1;      // bit 2
      BITS   TIM_UPD0            : 1;      // bit 3
      BITS   TIM_UPD1            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _SMED0_CFG reset value
    #define sfr_SMED0_CFG_RESET_VALUE   ((uint8_t) 0x00)

  } CFG;


  /** Dump counter lsb register (DMP_L at 0x551c) */
  union {

    /// bytewise access to DMP_L
    uint8_t  byte;

    /// bitwise access to register DMP_L
    struct {
      BITS   CNT_LO0             : 1;      // bit 0
      BITS   CNT_LO1             : 1;      // bit 1
      BITS   CNT_LO2             : 1;      // bit 2
      BITS   CNT_LO3             : 1;      // bit 3
      BITS   CNT_LO4             : 1;      // bit 4
      BITS   CNT_LO5             : 1;      // bit 5
      BITS   CNT_LO6             : 1;      // bit 6
      BITS   CNT_LO7             : 1;      // bit 7
    };  // DMP_L bitfield

    /// register _SMED0_DMP_L reset value
    #define sfr_SMED0_DMP_L_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_L;


  /** Dump counter msb register (DMP_H at 0x551d) */
  union {

    /// bytewise access to DMP_H
    uint8_t  byte;

    /// bitwise access to register DMP_H
    struct {
      BITS   CNT_LH0             : 1;      // bit 0
      BITS   CNT_LH1             : 1;      // bit 1
      BITS   CNT_LH2             : 1;      // bit 2
      BITS   CNT_LH3             : 1;      // bit 3
      BITS   CNT_LH4             : 1;      // bit 4
      BITS   CNT_LH5             : 1;      // bit 5
      BITS   CNT_LH6             : 1;      // bit 6
      BITS   CNT_LH7             : 1;      // bit 7
    };  // DMP_H bitfield

    /// register _SMED0_DMP_H reset value
    #define sfr_SMED0_DMP_H_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_H;


  /** General status register (GSTS at 0x551e) */
  union {

    /// bytewise access to GSTS
    uint8_t  byte;

    /// bitwise access to register GSTS
    struct {
      BITS   EX0_DUMP            : 1;      // bit 0
      BITS   EX1_DUMP            : 1;      // bit 1
      BITS   EX2_DUMP            : 1;      // bit 2
      BITS   CNT_FLAG            : 1;      // bit 3
      BITS   DMP_LK0             : 1;      // bit 4
      BITS   DMP_LK1             : 1;      // bit 5
      BITS   EVENT_OV            : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // GSTS bitfield

    /// register _SMED0_GSTS reset value
    #define sfr_SMED0_GSTS_RESET_VALUE   ((uint8_t) 0x00)

  } GSTS;


  /** Interrupt status register (ISR at 0x551f) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS   CNT_OVER            : 1;      // bit 0
      BITS   EXT0_INT            : 1;      // bit 1
      BITS   EXT1_INT            : 1;      // bit 2
      BITS   EXT2_INT            : 1;      // bit 3
      BITS   STA_S0_IT           : 1;      // bit 4
      BITS   STA_S1_IT           : 1;      // bit 5
      BITS   STA_S2_IT           : 1;      // bit 6
      BITS   STA_S3_IT           : 1;      // bit 7
    };  // ISR bitfield

    /// register _SMED0_ISR reset value
    #define sfr_SMED0_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /** Interrupt mask register (IMR at 0x5520) */
  union {

    /// bytewise access to IMR
    uint8_t  byte;

    /// bitwise access to register IMR
    struct {
      BITS   CNT_OV_R            : 1;      // bit 0
      BITS   IT_EXT0             : 1;      // bit 1
      BITS   IT_EXT1             : 1;      // bit 2
      BITS   IT_EXT2             : 1;      // bit 3
      BITS   IT_STA_S0           : 1;      // bit 4
      BITS   IT_STA_S1           : 1;      // bit 5
      BITS   IT_STA_S2           : 1;      // bit 6
      BITS   IT_STA_S3           : 1;      // bit 7
    };  // IMR bitfield

    /// register _SMED0_IMR reset value
    #define sfr_SMED0_IMR_RESET_VALUE   ((uint8_t) 0x00)

  } IMR;


  /** External event control register (ISEL at 0x5521) */
  union {

    /// bytewise access to ISEL
    uint8_t  byte;

    /// bitwise access to register ISEL
    struct {
      BITS   INPUT0_EN           : 1;      // bit 0
      BITS   INPUT1_EN           : 1;      // bit 1
      BITS   INPUT2_EN           : 1;      // bit 2
      BITS   INPUT_LAT           : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // ISEL bitfield

    /// register _SMED0_ISEL reset value
    #define sfr_SMED0_ISEL_RESET_VALUE   ((uint8_t) 0x00)

  } ISEL;


  /** Dump enable register (DMP at 0x5522) */
  union {

    /// bytewise access to DMP
    uint8_t  byte;

    /// bitwise access to register DMP
    struct {
      BITS   DMPE_EX0            : 1;      // bit 0
      BITS   DMPE_EX1            : 1;      // bit 1
      BITS   DMPE_EX2            : 1;      // bit 2
      BITS   DMP_EVER            : 1;      // bit 3
      BITS   CPL_IT_GE           : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // DMP bitfield

    /// register _SMED0_DMP reset value
    #define sfr_SMED0_DMP_RESET_VALUE   ((uint8_t) 0x00)

  } DMP;


  /** FSM status register (FSM_STS at 0x5523) */
  union {

    /// bytewise access to FSM_STS
    uint8_t  byte;

    /// bitwise access to register FSM_STS
    struct {
      BITS   FSM0                : 1;      // bit 0
      BITS   FSM1                : 1;      // bit 1
      BITS   FSM2                : 1;      // bit 2
      BITS   PWM                 : 1;      // bit 3
      BITS   EVINP0              : 1;      // bit 4
      BITS   EVINP1              : 1;      // bit 5
      BITS   EVINP2              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FSM_STS bitfield

    /// register _SMED0_FSM_STS reset value
    #define sfr_SMED0_FSM_STS_RESET_VALUE   ((uint8_t) 0x00)

  } FSM_STS;

} SMED0_t;

/// access to SMED0 SFR registers
#define sfr_SMED0   (*((SMED0_t*) 0x5500))


//------------------------
// Module SMED1
//------------------------

/** struct containing SMED1 module registers */
typedef struct {

  /** Control register (CTR at 0x5540) */
  union {

    /// bytewise access to CTR
    uint8_t  byte;

    /// bitwise access to register CTR
    struct {
      BITS   START_CNT           : 1;      // bit 0
      BITS   FSM_ENA             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CTR bitfield

    /// register _SMED1_CTR reset value
    #define sfr_SMED1_CTR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR;


  /** Control timer register (CTR_TMR at 0x5541) */
  union {

    /// bytewise access to CTR_TMR
    uint8_t  byte;

    /// bitwise access to register CTR_TMR
    struct {
      BITS   TIME_T0_VAL         : 1;      // bit 0
      BITS   TIME_T1_VAL         : 1;      // bit 1
      BITS   TIME_T2_VAL         : 1;      // bit 2
      BITS   TIME_T3_VAL         : 1;      // bit 3
      BITS   DITHER_VAL          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CTR_TMR bitfield

    /// register _SMED1_CTR_TMR reset value
    #define sfr_SMED1_CTR_TMR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_TMR;


  /** Control input register (CTR_INP at 0x5542) */
  union {

    /// bytewise access to CTR_INP
    uint8_t  byte;

    /// bitwise access to register CTR_INP
    struct {
      BITS   RS_INSIG0           : 1;      // bit 0
      BITS   RS_INSIG1           : 1;      // bit 1
      BITS   RS_INSIG2           : 1;      // bit 2
      BITS   RAIS_EN             : 1;      // bit 3
      BITS   EL_INSIG0           : 1;      // bit 4
      BITS   EL_INSIG1           : 1;      // bit 5
      BITS   EL_INSIG2           : 1;      // bit 6
      BITS   EL_EN               : 1;      // bit 7
    };  // CTR_INP bitfield

    /// register _SMED1_CTR_INP reset value
    #define sfr_SMED1_CTR_INP_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_INP;


  /** Dithering register (CTR_DTHR at 0x5543) */
  union {

    /// bytewise access to CTR_DTHR
    uint8_t  byte;

    /// bitwise access to register CTR_DTHR
    struct {
      BITS   DITH0               : 1;      // bit 0
      BITS   DITH1               : 1;      // bit 1
      BITS   DITH2               : 1;      // bit 2
      BITS   DITH3               : 1;      // bit 3
      BITS   DITH4               : 1;      // bit 4
      BITS   DITH5               : 1;      // bit 5
      BITS   DITH6               : 1;      // bit 6
      BITS   DITH7               : 1;      // bit 7
    };  // CTR_DTHR bitfield

    /// register _SMED1_CTR_DTHR reset value
    #define sfr_SMED1_CTR_DTHR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_DTHR;


  /** Time T0 lsb register (TMR_T0L at 0x5544) */
  union {

    /// bytewise access to TMR_T0L
    uint8_t  byte;

    /// bitwise access to register TMR_T0L
    struct {
      BITS   TIM_T0L0            : 1;      // bit 0
      BITS   TIM_T0L1            : 1;      // bit 1
      BITS   TIM_T0L2            : 1;      // bit 2
      BITS   TIM_T0L3            : 1;      // bit 3
      BITS   TIM_T0L4            : 1;      // bit 4
      BITS   TIM_T0L5            : 1;      // bit 5
      BITS   TIM_T0L6            : 1;      // bit 6
      BITS   TIM_T0L7            : 1;      // bit 7
    };  // TMR_T0L bitfield

    /// register _SMED1_TMR_T0L reset value
    #define sfr_SMED1_TMR_T0L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0L;


  /** Time T0 msb register (TMR_T0H at 0x5545) */
  union {

    /// bytewise access to TMR_T0H
    uint8_t  byte;

    /// bitwise access to register TMR_T0H
    struct {
      BITS   TIM_T0H0            : 1;      // bit 0
      BITS   TIM_T0H1            : 1;      // bit 1
      BITS   TIM_T0H2            : 1;      // bit 2
      BITS   TIM_T0H3            : 1;      // bit 3
      BITS   TIM_T0H4            : 1;      // bit 4
      BITS   TIM_T0H5            : 1;      // bit 5
      BITS   TIM_T0H6            : 1;      // bit 6
      BITS   TIM_T0H7            : 1;      // bit 7
    };  // TMR_T0H bitfield

    /// register _SMED1_TMR_T0H reset value
    #define sfr_SMED1_TMR_T0H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0H;


  /** Time T1 lsb register (TMR_T1L at 0x5546) */
  union {

    /// bytewise access to TMR_T1L
    uint8_t  byte;

    /// bitwise access to register TMR_T1L
    struct {
      BITS   TIM_T1L0            : 1;      // bit 0
      BITS   TIM_T1L1            : 1;      // bit 1
      BITS   TIM_T1L2            : 1;      // bit 2
      BITS   TIM_T1L3            : 1;      // bit 3
      BITS   TIM_T1L4            : 1;      // bit 4
      BITS   TIM_T1L5            : 1;      // bit 5
      BITS   TIM_T1L6            : 1;      // bit 6
      BITS   TIM_T1L7            : 1;      // bit 7
    };  // TMR_T1L bitfield

    /// register _SMED1_TMR_T1L reset value
    #define sfr_SMED1_TMR_T1L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1L;


  /** Time T1 msb register (TMR_T1H at 0x5547) */
  union {

    /// bytewise access to TMR_T1H
    uint8_t  byte;

    /// bitwise access to register TMR_T1H
    struct {
      BITS   TIM_T1H0            : 1;      // bit 0
      BITS   TIM_T1H1            : 1;      // bit 1
      BITS   TIM_T1H2            : 1;      // bit 2
      BITS   TIM_T1H3            : 1;      // bit 3
      BITS   TIM_T1H4            : 1;      // bit 4
      BITS   TIM_T1H5            : 1;      // bit 5
      BITS   TIM_T1H6            : 1;      // bit 6
      BITS   TIM_T1H7            : 1;      // bit 7
    };  // TMR_T1H bitfield

    /// register _SMED1_TMR_T1H reset value
    #define sfr_SMED1_TMR_T1H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1H;


  /** Time T2 lsb register (TMR_T2L at 0x5548) */
  union {

    /// bytewise access to TMR_T2L
    uint8_t  byte;

    /// bitwise access to register TMR_T2L
    struct {
      BITS   TIM_T2L0            : 1;      // bit 0
      BITS   TIM_T2L1            : 1;      // bit 1
      BITS   TIM_T2L2            : 1;      // bit 2
      BITS   TIM_T2L3            : 1;      // bit 3
      BITS   TIM_T2L4            : 1;      // bit 4
      BITS   TIM_T2L5            : 1;      // bit 5
      BITS   TIM_T2L6            : 1;      // bit 6
      BITS   TIM_T2L7            : 1;      // bit 7
    };  // TMR_T2L bitfield

    /// register _SMED1_TMR_T2L reset value
    #define sfr_SMED1_TMR_T2L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2L;


  /** Time T2 msb register (TMR_T2H at 0x5549) */
  union {

    /// bytewise access to TMR_T2H
    uint8_t  byte;

    /// bitwise access to register TMR_T2H
    struct {
      BITS   TIM_T2H0            : 1;      // bit 0
      BITS   TIM_T2H1            : 1;      // bit 1
      BITS   TIM_T2H2            : 1;      // bit 2
      BITS   TIM_T2H3            : 1;      // bit 3
      BITS   TIM_T2H4            : 1;      // bit 4
      BITS   TIM_T2H5            : 1;      // bit 5
      BITS   TIM_T2H6            : 1;      // bit 6
      BITS   TIM_T2H7            : 1;      // bit 7
    };  // TMR_T2H bitfield

    /// register _SMED1_TMR_T2H reset value
    #define sfr_SMED1_TMR_T2H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2H;


  /** Time T3 lsb register (TMR_T3L at 0x554a) */
  union {

    /// bytewise access to TMR_T3L
    uint8_t  byte;

    /// bitwise access to register TMR_T3L
    struct {
      BITS   TIM_T3L0            : 1;      // bit 0
      BITS   TIM_T3L1            : 1;      // bit 1
      BITS   TIM_T3L2            : 1;      // bit 2
      BITS   TIM_T3L3            : 1;      // bit 3
      BITS   TIM_T3L4            : 1;      // bit 4
      BITS   TIM_T3L5            : 1;      // bit 5
      BITS   TIM_T3L6            : 1;      // bit 6
      BITS   TIM_T3L7            : 1;      // bit 7
    };  // TMR_T3L bitfield

    /// register _SMED1_TMR_T3L reset value
    #define sfr_SMED1_TMR_T3L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3L;


  /** Time T3 msb register (TMR_T3H at 0x554b) */
  union {

    /// bytewise access to TMR_T3H
    uint8_t  byte;

    /// bitwise access to register TMR_T3H
    struct {
      BITS   TIM_T3H0            : 1;      // bit 0
      BITS   TIM_T3H1            : 1;      // bit 1
      BITS   TIM_T3H2            : 1;      // bit 2
      BITS   TIM_T3H3            : 1;      // bit 3
      BITS   TIM_T3H4            : 1;      // bit 4
      BITS   TIM_T3H5            : 1;      // bit 5
      BITS   TIM_T3H6            : 1;      // bit 6
      BITS   TIM_T3H7            : 1;      // bit 7
    };  // TMR_T3H bitfield

    /// register _SMED1_TMR_T3H reset value
    #define sfr_SMED1_TMR_T3H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3H;


  /** Parameter 0 IDLE register (PRM_ID0 at 0x554c) */
  union {

    /// bytewise access to PRM_ID0
    uint8_t  byte;

    /// bitwise access to register PRM_ID0
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_ID0 bitfield

    /// register _SMED1_PRM_ID0 reset value
    #define sfr_SMED1_PRM_ID0_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID0;


  /** Parameter 1 IDLE register (PRM_ID1 at 0x554d) */
  union {

    /// bytewise access to PRM_ID1
    uint8_t  byte;

    /// bitwise access to register PRM_ID1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_ID1 bitfield

    /// register _SMED1_PRM_ID1 reset value
    #define sfr_SMED1_PRM_ID1_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID1;


  /** Parameter 2 IDLE register (PRM_ID2 at 0x554e) */
  union {

    /// bytewise access to PRM_ID2
    uint8_t  byte;

    /// bitwise access to register PRM_ID2
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_ID2 bitfield

    /// register _SMED1_PRM_ID2 reset value
    #define sfr_SMED1_PRM_ID2_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID2;


  /** Parameter 0 S0 register (PRM_S00 at 0x554f) */
  union {

    /// bytewise access to PRM_S00
    uint8_t  byte;

    /// bitwise access to register PRM_S00
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S00 bitfield

    /// register _SMED1_PRM_S00 reset value
    #define sfr_SMED1_PRM_S00_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S00;


  /** Parameter 1 S0 register (PRM_S01 at 0x5550) */
  union {

    /// bytewise access to PRM_S01
    uint8_t  byte;

    /// bitwise access to register PRM_S01
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S01 bitfield

    /// register _SMED1_PRM_S01 reset value
    #define sfr_SMED1_PRM_S01_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S01;


  /** Parameter 2 S0 register (PRM_S02 at 0x5551) */
  union {

    /// bytewise access to PRM_S02
    uint8_t  byte;

    /// bitwise access to register PRM_S02
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S02 bitfield

    /// register _SMED1_PRM_S02 reset value
    #define sfr_SMED1_PRM_S02_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S02;


  /** Parameter 0 S1 register (PRM_S10 at 0x5552) */
  union {

    /// bytewise access to PRM_S10
    uint8_t  byte;

    /// bitwise access to register PRM_S10
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S10 bitfield

    /// register _SMED1_PRM_S10 reset value
    #define sfr_SMED1_PRM_S10_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S10;


  /** Parameter 1 S1 register (PRM_S11 at 0x5553) */
  union {

    /// bytewise access to PRM_S11
    uint8_t  byte;

    /// bitwise access to register PRM_S11
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S11 bitfield

    /// register _SMED1_PRM_S11 reset value
    #define sfr_SMED1_PRM_S11_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S11;


  /** Parameter 2 S1 register (PRM_S12 at 0x5554) */
  union {

    /// bytewise access to PRM_S12
    uint8_t  byte;

    /// bitwise access to register PRM_S12
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S12 bitfield

    /// register _SMED1_PRM_S12 reset value
    #define sfr_SMED1_PRM_S12_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S12;


  /** Parameter 0 S2 register (PRM_S20 at 0x5555) */
  union {

    /// bytewise access to PRM_S20
    uint8_t  byte;

    /// bitwise access to register PRM_S20
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S20 bitfield

    /// register _SMED1_PRM_S20 reset value
    #define sfr_SMED1_PRM_S20_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S20;


  /** Parameter 1 S2 register (PRM_S21 at 0x5556) */
  union {

    /// bytewise access to PRM_S21
    uint8_t  byte;

    /// bitwise access to register PRM_S21
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S21 bitfield

    /// register _SMED1_PRM_S21 reset value
    #define sfr_SMED1_PRM_S21_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S21;


  /** Parameter 2 S2 register (PRM_S22 at 0x5557) */
  union {

    /// bytewise access to PRM_S22
    uint8_t  byte;

    /// bitwise access to register PRM_S22
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S22 bitfield

    /// register _SMED1_PRM_S22 reset value
    #define sfr_SMED1_PRM_S22_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S22;


  /** Parameter 0 S3 register (PRM_S30 at 0x5558) */
  union {

    /// bytewise access to PRM_S30
    uint8_t  byte;

    /// bitwise access to register PRM_S30
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S30 bitfield

    /// register _SMED1_PRM_S30 reset value
    #define sfr_SMED1_PRM_S30_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S30;


  /** Parameter 1 S3 register (PRM_S31 at 0x5559) */
  union {

    /// bytewise access to PRM_S31
    uint8_t  byte;

    /// bitwise access to register PRM_S31
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S31 bitfield

    /// register _SMED1_PRM_S31 reset value
    #define sfr_SMED1_PRM_S31_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S31;


  /** Parameter 2 S3 register (PRM_S32 at 0x555a) */
  union {

    /// bytewise access to PRM_S32
    uint8_t  byte;

    /// bitwise access to register PRM_S32
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S32 bitfield

    /// register _SMED1_PRM_S32 reset value
    #define sfr_SMED1_PRM_S32_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S32;


  /** Timer configuration register (CFG at 0x555b) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TIM_NUM0            : 1;      // bit 1
      BITS   TIM_NUM1            : 1;      // bit 2
      BITS   TIM_UPD0            : 1;      // bit 3
      BITS   TIM_UPD1            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _SMED1_CFG reset value
    #define sfr_SMED1_CFG_RESET_VALUE   ((uint8_t) 0x00)

  } CFG;


  /** Dump counter lsb register (DMP_L at 0x555c) */
  union {

    /// bytewise access to DMP_L
    uint8_t  byte;

    /// bitwise access to register DMP_L
    struct {
      BITS   CNT_LO0             : 1;      // bit 0
      BITS   CNT_LO1             : 1;      // bit 1
      BITS   CNT_LO2             : 1;      // bit 2
      BITS   CNT_LO3             : 1;      // bit 3
      BITS   CNT_LO4             : 1;      // bit 4
      BITS   CNT_LO5             : 1;      // bit 5
      BITS   CNT_LO6             : 1;      // bit 6
      BITS   CNT_LO7             : 1;      // bit 7
    };  // DMP_L bitfield

    /// register _SMED1_DMP_L reset value
    #define sfr_SMED1_DMP_L_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_L;


  /** Dump counter msb register (DMP_H at 0x555d) */
  union {

    /// bytewise access to DMP_H
    uint8_t  byte;

    /// bitwise access to register DMP_H
    struct {
      BITS   CNT_LH0             : 1;      // bit 0
      BITS   CNT_LH1             : 1;      // bit 1
      BITS   CNT_LH2             : 1;      // bit 2
      BITS   CNT_LH3             : 1;      // bit 3
      BITS   CNT_LH4             : 1;      // bit 4
      BITS   CNT_LH5             : 1;      // bit 5
      BITS   CNT_LH6             : 1;      // bit 6
      BITS   CNT_LH7             : 1;      // bit 7
    };  // DMP_H bitfield

    /// register _SMED1_DMP_H reset value
    #define sfr_SMED1_DMP_H_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_H;


  /** General status register (GSTS at 0x555e) */
  union {

    /// bytewise access to GSTS
    uint8_t  byte;

    /// bitwise access to register GSTS
    struct {
      BITS   EX0_DUMP            : 1;      // bit 0
      BITS   EX1_DUMP            : 1;      // bit 1
      BITS   EX2_DUMP            : 1;      // bit 2
      BITS   CNT_FLAG            : 1;      // bit 3
      BITS   DMP_LK0             : 1;      // bit 4
      BITS   DMP_LK1             : 1;      // bit 5
      BITS   EVENT_OV            : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // GSTS bitfield

    /// register _SMED1_GSTS reset value
    #define sfr_SMED1_GSTS_RESET_VALUE   ((uint8_t) 0x00)

  } GSTS;


  /** Interrupt status register (ISR at 0x555f) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS   CNT_OVER            : 1;      // bit 0
      BITS   EXT0_INT            : 1;      // bit 1
      BITS   EXT1_INT            : 1;      // bit 2
      BITS   EXT2_INT            : 1;      // bit 3
      BITS   STA_S0_IT           : 1;      // bit 4
      BITS   STA_S1_IT           : 1;      // bit 5
      BITS   STA_S2_IT           : 1;      // bit 6
      BITS   STA_S3_IT           : 1;      // bit 7
    };  // ISR bitfield

    /// register _SMED1_ISR reset value
    #define sfr_SMED1_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /** Interrupt mask register (IMR at 0x5560) */
  union {

    /// bytewise access to IMR
    uint8_t  byte;

    /// bitwise access to register IMR
    struct {
      BITS   CNT_OV_R            : 1;      // bit 0
      BITS   IT_EXT0             : 1;      // bit 1
      BITS   IT_EXT1             : 1;      // bit 2
      BITS   IT_EXT2             : 1;      // bit 3
      BITS   IT_STA_S0           : 1;      // bit 4
      BITS   IT_STA_S1           : 1;      // bit 5
      BITS   IT_STA_S2           : 1;      // bit 6
      BITS   IT_STA_S3           : 1;      // bit 7
    };  // IMR bitfield

    /// register _SMED1_IMR reset value
    #define sfr_SMED1_IMR_RESET_VALUE   ((uint8_t) 0x00)

  } IMR;


  /** External event control register (ISEL at 0x5561) */
  union {

    /// bytewise access to ISEL
    uint8_t  byte;

    /// bitwise access to register ISEL
    struct {
      BITS   INPUT0_EN           : 1;      // bit 0
      BITS   INPUT1_EN           : 1;      // bit 1
      BITS   INPUT2_EN           : 1;      // bit 2
      BITS   INPUT_LAT           : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // ISEL bitfield

    /// register _SMED1_ISEL reset value
    #define sfr_SMED1_ISEL_RESET_VALUE   ((uint8_t) 0x00)

  } ISEL;


  /** Dump enable register (DMP at 0x5562) */
  union {

    /// bytewise access to DMP
    uint8_t  byte;

    /// bitwise access to register DMP
    struct {
      BITS   DMPE_EX0            : 1;      // bit 0
      BITS   DMPE_EX1            : 1;      // bit 1
      BITS   DMPE_EX2            : 1;      // bit 2
      BITS   DMP_EVER            : 1;      // bit 3
      BITS   CPL_IT_GE           : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // DMP bitfield

    /// register _SMED1_DMP reset value
    #define sfr_SMED1_DMP_RESET_VALUE   ((uint8_t) 0x00)

  } DMP;


  /** FSM status register (FSM_STS at 0x5563) */
  union {

    /// bytewise access to FSM_STS
    uint8_t  byte;

    /// bitwise access to register FSM_STS
    struct {
      BITS   FSM0                : 1;      // bit 0
      BITS   FSM1                : 1;      // bit 1
      BITS   FSM2                : 1;      // bit 2
      BITS   PWM                 : 1;      // bit 3
      BITS   EVINP0              : 1;      // bit 4
      BITS   EVINP1              : 1;      // bit 5
      BITS   EVINP2              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FSM_STS bitfield

    /// register _SMED1_FSM_STS reset value
    #define sfr_SMED1_FSM_STS_RESET_VALUE   ((uint8_t) 0x00)

  } FSM_STS;

} SMED1_t;

/// access to SMED1 SFR registers
#define sfr_SMED1   (*((SMED1_t*) 0x5540))


//------------------------
// Module SMED2
//------------------------

/** struct containing SMED2 module registers */
typedef struct {

  /** Control register (CTR at 0x5580) */
  union {

    /// bytewise access to CTR
    uint8_t  byte;

    /// bitwise access to register CTR
    struct {
      BITS   START_CNT           : 1;      // bit 0
      BITS   FSM_ENA             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CTR bitfield

    /// register _SMED2_CTR reset value
    #define sfr_SMED2_CTR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR;


  /** Control timer register (CTR_TMR at 0x5581) */
  union {

    /// bytewise access to CTR_TMR
    uint8_t  byte;

    /// bitwise access to register CTR_TMR
    struct {
      BITS   TIME_T0_VAL         : 1;      // bit 0
      BITS   TIME_T1_VAL         : 1;      // bit 1
      BITS   TIME_T2_VAL         : 1;      // bit 2
      BITS   TIME_T3_VAL         : 1;      // bit 3
      BITS   DITHER_VAL          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CTR_TMR bitfield

    /// register _SMED2_CTR_TMR reset value
    #define sfr_SMED2_CTR_TMR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_TMR;


  /** Control input register (CTR_INP at 0x5582) */
  union {

    /// bytewise access to CTR_INP
    uint8_t  byte;

    /// bitwise access to register CTR_INP
    struct {
      BITS   RS_INSIG0           : 1;      // bit 0
      BITS   RS_INSIG1           : 1;      // bit 1
      BITS   RS_INSIG2           : 1;      // bit 2
      BITS   RAIS_EN             : 1;      // bit 3
      BITS   EL_INSIG0           : 1;      // bit 4
      BITS   EL_INSIG1           : 1;      // bit 5
      BITS   EL_INSIG2           : 1;      // bit 6
      BITS   EL_EN               : 1;      // bit 7
    };  // CTR_INP bitfield

    /// register _SMED2_CTR_INP reset value
    #define sfr_SMED2_CTR_INP_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_INP;


  /** Dithering register (CTR_DTHR at 0x5583) */
  union {

    /// bytewise access to CTR_DTHR
    uint8_t  byte;

    /// bitwise access to register CTR_DTHR
    struct {
      BITS   DITH0               : 1;      // bit 0
      BITS   DITH1               : 1;      // bit 1
      BITS   DITH2               : 1;      // bit 2
      BITS   DITH3               : 1;      // bit 3
      BITS   DITH4               : 1;      // bit 4
      BITS   DITH5               : 1;      // bit 5
      BITS   DITH6               : 1;      // bit 6
      BITS   DITH7               : 1;      // bit 7
    };  // CTR_DTHR bitfield

    /// register _SMED2_CTR_DTHR reset value
    #define sfr_SMED2_CTR_DTHR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_DTHR;


  /** Time T0 lsb register (TMR_T0L at 0x5584) */
  union {

    /// bytewise access to TMR_T0L
    uint8_t  byte;

    /// bitwise access to register TMR_T0L
    struct {
      BITS   TIM_T0L0            : 1;      // bit 0
      BITS   TIM_T0L1            : 1;      // bit 1
      BITS   TIM_T0L2            : 1;      // bit 2
      BITS   TIM_T0L3            : 1;      // bit 3
      BITS   TIM_T0L4            : 1;      // bit 4
      BITS   TIM_T0L5            : 1;      // bit 5
      BITS   TIM_T0L6            : 1;      // bit 6
      BITS   TIM_T0L7            : 1;      // bit 7
    };  // TMR_T0L bitfield

    /// register _SMED2_TMR_T0L reset value
    #define sfr_SMED2_TMR_T0L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0L;


  /** Time T0 msb register (TMR_T0H at 0x5585) */
  union {

    /// bytewise access to TMR_T0H
    uint8_t  byte;

    /// bitwise access to register TMR_T0H
    struct {
      BITS   TIM_T0H0            : 1;      // bit 0
      BITS   TIM_T0H1            : 1;      // bit 1
      BITS   TIM_T0H2            : 1;      // bit 2
      BITS   TIM_T0H3            : 1;      // bit 3
      BITS   TIM_T0H4            : 1;      // bit 4
      BITS   TIM_T0H5            : 1;      // bit 5
      BITS   TIM_T0H6            : 1;      // bit 6
      BITS   TIM_T0H7            : 1;      // bit 7
    };  // TMR_T0H bitfield

    /// register _SMED2_TMR_T0H reset value
    #define sfr_SMED2_TMR_T0H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0H;


  /** Time T1 lsb register (TMR_T1L at 0x5586) */
  union {

    /// bytewise access to TMR_T1L
    uint8_t  byte;

    /// bitwise access to register TMR_T1L
    struct {
      BITS   TIM_T1L0            : 1;      // bit 0
      BITS   TIM_T1L1            : 1;      // bit 1
      BITS   TIM_T1L2            : 1;      // bit 2
      BITS   TIM_T1L3            : 1;      // bit 3
      BITS   TIM_T1L4            : 1;      // bit 4
      BITS   TIM_T1L5            : 1;      // bit 5
      BITS   TIM_T1L6            : 1;      // bit 6
      BITS   TIM_T1L7            : 1;      // bit 7
    };  // TMR_T1L bitfield

    /// register _SMED2_TMR_T1L reset value
    #define sfr_SMED2_TMR_T1L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1L;


  /** Time T1 msb register (TMR_T1H at 0x5587) */
  union {

    /// bytewise access to TMR_T1H
    uint8_t  byte;

    /// bitwise access to register TMR_T1H
    struct {
      BITS   TIM_T1H0            : 1;      // bit 0
      BITS   TIM_T1H1            : 1;      // bit 1
      BITS   TIM_T1H2            : 1;      // bit 2
      BITS   TIM_T1H3            : 1;      // bit 3
      BITS   TIM_T1H4            : 1;      // bit 4
      BITS   TIM_T1H5            : 1;      // bit 5
      BITS   TIM_T1H6            : 1;      // bit 6
      BITS   TIM_T1H7            : 1;      // bit 7
    };  // TMR_T1H bitfield

    /// register _SMED2_TMR_T1H reset value
    #define sfr_SMED2_TMR_T1H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1H;


  /** Time T2 lsb register (TMR_T2L at 0x5588) */
  union {

    /// bytewise access to TMR_T2L
    uint8_t  byte;

    /// bitwise access to register TMR_T2L
    struct {
      BITS   TIM_T2L0            : 1;      // bit 0
      BITS   TIM_T2L1            : 1;      // bit 1
      BITS   TIM_T2L2            : 1;      // bit 2
      BITS   TIM_T2L3            : 1;      // bit 3
      BITS   TIM_T2L4            : 1;      // bit 4
      BITS   TIM_T2L5            : 1;      // bit 5
      BITS   TIM_T2L6            : 1;      // bit 6
      BITS   TIM_T2L7            : 1;      // bit 7
    };  // TMR_T2L bitfield

    /// register _SMED2_TMR_T2L reset value
    #define sfr_SMED2_TMR_T2L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2L;


  /** Time T2 msb register (TMR_T2H at 0x5589) */
  union {

    /// bytewise access to TMR_T2H
    uint8_t  byte;

    /// bitwise access to register TMR_T2H
    struct {
      BITS   TIM_T2H0            : 1;      // bit 0
      BITS   TIM_T2H1            : 1;      // bit 1
      BITS   TIM_T2H2            : 1;      // bit 2
      BITS   TIM_T2H3            : 1;      // bit 3
      BITS   TIM_T2H4            : 1;      // bit 4
      BITS   TIM_T2H5            : 1;      // bit 5
      BITS   TIM_T2H6            : 1;      // bit 6
      BITS   TIM_T2H7            : 1;      // bit 7
    };  // TMR_T2H bitfield

    /// register _SMED2_TMR_T2H reset value
    #define sfr_SMED2_TMR_T2H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2H;


  /** Time T3 lsb register (TMR_T3L at 0x558a) */
  union {

    /// bytewise access to TMR_T3L
    uint8_t  byte;

    /// bitwise access to register TMR_T3L
    struct {
      BITS   TIM_T3L0            : 1;      // bit 0
      BITS   TIM_T3L1            : 1;      // bit 1
      BITS   TIM_T3L2            : 1;      // bit 2
      BITS   TIM_T3L3            : 1;      // bit 3
      BITS   TIM_T3L4            : 1;      // bit 4
      BITS   TIM_T3L5            : 1;      // bit 5
      BITS   TIM_T3L6            : 1;      // bit 6
      BITS   TIM_T3L7            : 1;      // bit 7
    };  // TMR_T3L bitfield

    /// register _SMED2_TMR_T3L reset value
    #define sfr_SMED2_TMR_T3L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3L;


  /** Time T3 msb register (TMR_T3H at 0x558b) */
  union {

    /// bytewise access to TMR_T3H
    uint8_t  byte;

    /// bitwise access to register TMR_T3H
    struct {
      BITS   TIM_T3H0            : 1;      // bit 0
      BITS   TIM_T3H1            : 1;      // bit 1
      BITS   TIM_T3H2            : 1;      // bit 2
      BITS   TIM_T3H3            : 1;      // bit 3
      BITS   TIM_T3H4            : 1;      // bit 4
      BITS   TIM_T3H5            : 1;      // bit 5
      BITS   TIM_T3H6            : 1;      // bit 6
      BITS   TIM_T3H7            : 1;      // bit 7
    };  // TMR_T3H bitfield

    /// register _SMED2_TMR_T3H reset value
    #define sfr_SMED2_TMR_T3H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3H;


  /** Parameter 0 IDLE register (PRM_ID0 at 0x558c) */
  union {

    /// bytewise access to PRM_ID0
    uint8_t  byte;

    /// bitwise access to register PRM_ID0
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_ID0 bitfield

    /// register _SMED2_PRM_ID0 reset value
    #define sfr_SMED2_PRM_ID0_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID0;


  /** Parameter 1 IDLE register (PRM_ID1 at 0x558d) */
  union {

    /// bytewise access to PRM_ID1
    uint8_t  byte;

    /// bitwise access to register PRM_ID1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_ID1 bitfield

    /// register _SMED2_PRM_ID1 reset value
    #define sfr_SMED2_PRM_ID1_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID1;


  /** Parameter 2 IDLE register (PRM_ID2 at 0x558e) */
  union {

    /// bytewise access to PRM_ID2
    uint8_t  byte;

    /// bitwise access to register PRM_ID2
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_ID2 bitfield

    /// register _SMED2_PRM_ID2 reset value
    #define sfr_SMED2_PRM_ID2_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID2;


  /** Parameter 0 S0 register (PRM_S00 at 0x558f) */
  union {

    /// bytewise access to PRM_S00
    uint8_t  byte;

    /// bitwise access to register PRM_S00
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S00 bitfield

    /// register _SMED2_PRM_S00 reset value
    #define sfr_SMED2_PRM_S00_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S00;


  /** Parameter 1 S0 register (PRM_S01 at 0x5590) */
  union {

    /// bytewise access to PRM_S01
    uint8_t  byte;

    /// bitwise access to register PRM_S01
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S01 bitfield

    /// register _SMED2_PRM_S01 reset value
    #define sfr_SMED2_PRM_S01_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S01;


  /** Parameter 2 S0 register (PRM_S02 at 0x5591) */
  union {

    /// bytewise access to PRM_S02
    uint8_t  byte;

    /// bitwise access to register PRM_S02
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S02 bitfield

    /// register _SMED2_PRM_S02 reset value
    #define sfr_SMED2_PRM_S02_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S02;


  /** Parameter 0 S1 register (PRM_S10 at 0x5592) */
  union {

    /// bytewise access to PRM_S10
    uint8_t  byte;

    /// bitwise access to register PRM_S10
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S10 bitfield

    /// register _SMED2_PRM_S10 reset value
    #define sfr_SMED2_PRM_S10_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S10;


  /** Parameter 1 S1 register (PRM_S11 at 0x5593) */
  union {

    /// bytewise access to PRM_S11
    uint8_t  byte;

    /// bitwise access to register PRM_S11
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S11 bitfield

    /// register _SMED2_PRM_S11 reset value
    #define sfr_SMED2_PRM_S11_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S11;


  /** Parameter 2 S1 register (PRM_S12 at 0x5594) */
  union {

    /// bytewise access to PRM_S12
    uint8_t  byte;

    /// bitwise access to register PRM_S12
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S12 bitfield

    /// register _SMED2_PRM_S12 reset value
    #define sfr_SMED2_PRM_S12_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S12;


  /** Parameter 0 S2 register (PRM_S20 at 0x5595) */
  union {

    /// bytewise access to PRM_S20
    uint8_t  byte;

    /// bitwise access to register PRM_S20
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S20 bitfield

    /// register _SMED2_PRM_S20 reset value
    #define sfr_SMED2_PRM_S20_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S20;


  /** Parameter 1 S2 register (PRM_S21 at 0x5596) */
  union {

    /// bytewise access to PRM_S21
    uint8_t  byte;

    /// bitwise access to register PRM_S21
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S21 bitfield

    /// register _SMED2_PRM_S21 reset value
    #define sfr_SMED2_PRM_S21_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S21;


  /** Parameter 2 S2 register (PRM_S22 at 0x5597) */
  union {

    /// bytewise access to PRM_S22
    uint8_t  byte;

    /// bitwise access to register PRM_S22
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S22 bitfield

    /// register _SMED2_PRM_S22 reset value
    #define sfr_SMED2_PRM_S22_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S22;


  /** Parameter 0 S3 register (PRM_S30 at 0x5598) */
  union {

    /// bytewise access to PRM_S30
    uint8_t  byte;

    /// bitwise access to register PRM_S30
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S30 bitfield

    /// register _SMED2_PRM_S30 reset value
    #define sfr_SMED2_PRM_S30_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S30;


  /** Parameter 1 S3 register (PRM_S31 at 0x5599) */
  union {

    /// bytewise access to PRM_S31
    uint8_t  byte;

    /// bitwise access to register PRM_S31
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S31 bitfield

    /// register _SMED2_PRM_S31 reset value
    #define sfr_SMED2_PRM_S31_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S31;


  /** Parameter 2 S3 register (PRM_S32 at 0x559a) */
  union {

    /// bytewise access to PRM_S32
    uint8_t  byte;

    /// bitwise access to register PRM_S32
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S32 bitfield

    /// register _SMED2_PRM_S32 reset value
    #define sfr_SMED2_PRM_S32_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S32;


  /** Timer configuration register (CFG at 0x559b) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TIM_NUM0            : 1;      // bit 1
      BITS   TIM_NUM1            : 1;      // bit 2
      BITS   TIM_UPD0            : 1;      // bit 3
      BITS   TIM_UPD1            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _SMED2_CFG reset value
    #define sfr_SMED2_CFG_RESET_VALUE   ((uint8_t) 0x00)

  } CFG;


  /** Dump counter lsb register (DMP_L at 0x559c) */
  union {

    /// bytewise access to DMP_L
    uint8_t  byte;

    /// bitwise access to register DMP_L
    struct {
      BITS   CNT_LO0             : 1;      // bit 0
      BITS   CNT_LO1             : 1;      // bit 1
      BITS   CNT_LO2             : 1;      // bit 2
      BITS   CNT_LO3             : 1;      // bit 3
      BITS   CNT_LO4             : 1;      // bit 4
      BITS   CNT_LO5             : 1;      // bit 5
      BITS   CNT_LO6             : 1;      // bit 6
      BITS   CNT_LO7             : 1;      // bit 7
    };  // DMP_L bitfield

    /// register _SMED2_DMP_L reset value
    #define sfr_SMED2_DMP_L_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_L;


  /** Dump counter msb register (DMP_H at 0x559d) */
  union {

    /// bytewise access to DMP_H
    uint8_t  byte;

    /// bitwise access to register DMP_H
    struct {
      BITS   CNT_LH0             : 1;      // bit 0
      BITS   CNT_LH1             : 1;      // bit 1
      BITS   CNT_LH2             : 1;      // bit 2
      BITS   CNT_LH3             : 1;      // bit 3
      BITS   CNT_LH4             : 1;      // bit 4
      BITS   CNT_LH5             : 1;      // bit 5
      BITS   CNT_LH6             : 1;      // bit 6
      BITS   CNT_LH7             : 1;      // bit 7
    };  // DMP_H bitfield

    /// register _SMED2_DMP_H reset value
    #define sfr_SMED2_DMP_H_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_H;


  /** General status register (GSTS at 0x559e) */
  union {

    /// bytewise access to GSTS
    uint8_t  byte;

    /// bitwise access to register GSTS
    struct {
      BITS   EX0_DUMP            : 1;      // bit 0
      BITS   EX1_DUMP            : 1;      // bit 1
      BITS   EX2_DUMP            : 1;      // bit 2
      BITS   CNT_FLAG            : 1;      // bit 3
      BITS   DMP_LK0             : 1;      // bit 4
      BITS   DMP_LK1             : 1;      // bit 5
      BITS   EVENT_OV            : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // GSTS bitfield

    /// register _SMED2_GSTS reset value
    #define sfr_SMED2_GSTS_RESET_VALUE   ((uint8_t) 0x00)

  } GSTS;


  /** Interrupt status register (ISR at 0x559f) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS   CNT_OVER            : 1;      // bit 0
      BITS   EXT0_INT            : 1;      // bit 1
      BITS   EXT1_INT            : 1;      // bit 2
      BITS   EXT2_INT            : 1;      // bit 3
      BITS   STA_S0_IT           : 1;      // bit 4
      BITS   STA_S1_IT           : 1;      // bit 5
      BITS   STA_S2_IT           : 1;      // bit 6
      BITS   STA_S3_IT           : 1;      // bit 7
    };  // ISR bitfield

    /// register _SMED2_ISR reset value
    #define sfr_SMED2_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /** Interrupt mask register (IMR at 0x55a0) */
  union {

    /// bytewise access to IMR
    uint8_t  byte;

    /// bitwise access to register IMR
    struct {
      BITS   CNT_OV_R            : 1;      // bit 0
      BITS   IT_EXT0             : 1;      // bit 1
      BITS   IT_EXT1             : 1;      // bit 2
      BITS   IT_EXT2             : 1;      // bit 3
      BITS   IT_STA_S0           : 1;      // bit 4
      BITS   IT_STA_S1           : 1;      // bit 5
      BITS   IT_STA_S2           : 1;      // bit 6
      BITS   IT_STA_S3           : 1;      // bit 7
    };  // IMR bitfield

    /// register _SMED2_IMR reset value
    #define sfr_SMED2_IMR_RESET_VALUE   ((uint8_t) 0x00)

  } IMR;


  /** External event control register (ISEL at 0x55a1) */
  union {

    /// bytewise access to ISEL
    uint8_t  byte;

    /// bitwise access to register ISEL
    struct {
      BITS   INPUT0_EN           : 1;      // bit 0
      BITS   INPUT1_EN           : 1;      // bit 1
      BITS   INPUT2_EN           : 1;      // bit 2
      BITS   INPUT_LAT           : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // ISEL bitfield

    /// register _SMED2_ISEL reset value
    #define sfr_SMED2_ISEL_RESET_VALUE   ((uint8_t) 0x00)

  } ISEL;


  /** Dump enable register (DMP at 0x55a2) */
  union {

    /// bytewise access to DMP
    uint8_t  byte;

    /// bitwise access to register DMP
    struct {
      BITS   DMPE_EX0            : 1;      // bit 0
      BITS   DMPE_EX1            : 1;      // bit 1
      BITS   DMPE_EX2            : 1;      // bit 2
      BITS   DMP_EVER            : 1;      // bit 3
      BITS   CPL_IT_GE           : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // DMP bitfield

    /// register _SMED2_DMP reset value
    #define sfr_SMED2_DMP_RESET_VALUE   ((uint8_t) 0x00)

  } DMP;


  /** FSM status register (FSM_STS at 0x55a3) */
  union {

    /// bytewise access to FSM_STS
    uint8_t  byte;

    /// bitwise access to register FSM_STS
    struct {
      BITS   FSM0                : 1;      // bit 0
      BITS   FSM1                : 1;      // bit 1
      BITS   FSM2                : 1;      // bit 2
      BITS   PWM                 : 1;      // bit 3
      BITS   EVINP0              : 1;      // bit 4
      BITS   EVINP1              : 1;      // bit 5
      BITS   EVINP2              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FSM_STS bitfield

    /// register _SMED2_FSM_STS reset value
    #define sfr_SMED2_FSM_STS_RESET_VALUE   ((uint8_t) 0x00)

  } FSM_STS;

} SMED2_t;

/// access to SMED2 SFR registers
#define sfr_SMED2   (*((SMED2_t*) 0x5580))


//------------------------
// Module SMED3
//------------------------

/** struct containing SMED3 module registers */
typedef struct {

  /** Control register (CTR at 0x55c0) */
  union {

    /// bytewise access to CTR
    uint8_t  byte;

    /// bitwise access to register CTR
    struct {
      BITS   START_CNT           : 1;      // bit 0
      BITS   FSM_ENA             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CTR bitfield

    /// register _SMED3_CTR reset value
    #define sfr_SMED3_CTR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR;


  /** Control timer register (CTR_TMR at 0x55c1) */
  union {

    /// bytewise access to CTR_TMR
    uint8_t  byte;

    /// bitwise access to register CTR_TMR
    struct {
      BITS   TIME_T0_VAL         : 1;      // bit 0
      BITS   TIME_T1_VAL         : 1;      // bit 1
      BITS   TIME_T2_VAL         : 1;      // bit 2
      BITS   TIME_T3_VAL         : 1;      // bit 3
      BITS   DITHER_VAL          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CTR_TMR bitfield

    /// register _SMED3_CTR_TMR reset value
    #define sfr_SMED3_CTR_TMR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_TMR;


  /** Control input register (CTR_INP at 0x55c2) */
  union {

    /// bytewise access to CTR_INP
    uint8_t  byte;

    /// bitwise access to register CTR_INP
    struct {
      BITS   RS_INSIG0           : 1;      // bit 0
      BITS   RS_INSIG1           : 1;      // bit 1
      BITS   RS_INSIG2           : 1;      // bit 2
      BITS   RAIS_EN             : 1;      // bit 3
      BITS   EL_INSIG0           : 1;      // bit 4
      BITS   EL_INSIG1           : 1;      // bit 5
      BITS   EL_INSIG2           : 1;      // bit 6
      BITS   EL_EN               : 1;      // bit 7
    };  // CTR_INP bitfield

    /// register _SMED3_CTR_INP reset value
    #define sfr_SMED3_CTR_INP_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_INP;


  /** Dithering register (CTR_DTHR at 0x55c3) */
  union {

    /// bytewise access to CTR_DTHR
    uint8_t  byte;

    /// bitwise access to register CTR_DTHR
    struct {
      BITS   DITH0               : 1;      // bit 0
      BITS   DITH1               : 1;      // bit 1
      BITS   DITH2               : 1;      // bit 2
      BITS   DITH3               : 1;      // bit 3
      BITS   DITH4               : 1;      // bit 4
      BITS   DITH5               : 1;      // bit 5
      BITS   DITH6               : 1;      // bit 6
      BITS   DITH7               : 1;      // bit 7
    };  // CTR_DTHR bitfield

    /// register _SMED3_CTR_DTHR reset value
    #define sfr_SMED3_CTR_DTHR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_DTHR;


  /** Time T0 lsb register (TMR_T0L at 0x55c4) */
  union {

    /// bytewise access to TMR_T0L
    uint8_t  byte;

    /// bitwise access to register TMR_T0L
    struct {
      BITS   TIM_T0L0            : 1;      // bit 0
      BITS   TIM_T0L1            : 1;      // bit 1
      BITS   TIM_T0L2            : 1;      // bit 2
      BITS   TIM_T0L3            : 1;      // bit 3
      BITS   TIM_T0L4            : 1;      // bit 4
      BITS   TIM_T0L5            : 1;      // bit 5
      BITS   TIM_T0L6            : 1;      // bit 6
      BITS   TIM_T0L7            : 1;      // bit 7
    };  // TMR_T0L bitfield

    /// register _SMED3_TMR_T0L reset value
    #define sfr_SMED3_TMR_T0L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0L;


  /** Time T0 msb register (TMR_T0H at 0x55c5) */
  union {

    /// bytewise access to TMR_T0H
    uint8_t  byte;

    /// bitwise access to register TMR_T0H
    struct {
      BITS   TIM_T0H0            : 1;      // bit 0
      BITS   TIM_T0H1            : 1;      // bit 1
      BITS   TIM_T0H2            : 1;      // bit 2
      BITS   TIM_T0H3            : 1;      // bit 3
      BITS   TIM_T0H4            : 1;      // bit 4
      BITS   TIM_T0H5            : 1;      // bit 5
      BITS   TIM_T0H6            : 1;      // bit 6
      BITS   TIM_T0H7            : 1;      // bit 7
    };  // TMR_T0H bitfield

    /// register _SMED3_TMR_T0H reset value
    #define sfr_SMED3_TMR_T0H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0H;


  /** Time T1 lsb register (TMR_T1L at 0x55c6) */
  union {

    /// bytewise access to TMR_T1L
    uint8_t  byte;

    /// bitwise access to register TMR_T1L
    struct {
      BITS   TIM_T1L0            : 1;      // bit 0
      BITS   TIM_T1L1            : 1;      // bit 1
      BITS   TIM_T1L2            : 1;      // bit 2
      BITS   TIM_T1L3            : 1;      // bit 3
      BITS   TIM_T1L4            : 1;      // bit 4
      BITS   TIM_T1L5            : 1;      // bit 5
      BITS   TIM_T1L6            : 1;      // bit 6
      BITS   TIM_T1L7            : 1;      // bit 7
    };  // TMR_T1L bitfield

    /// register _SMED3_TMR_T1L reset value
    #define sfr_SMED3_TMR_T1L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1L;


  /** Time T1 msb register (TMR_T1H at 0x55c7) */
  union {

    /// bytewise access to TMR_T1H
    uint8_t  byte;

    /// bitwise access to register TMR_T1H
    struct {
      BITS   TIM_T1H0            : 1;      // bit 0
      BITS   TIM_T1H1            : 1;      // bit 1
      BITS   TIM_T1H2            : 1;      // bit 2
      BITS   TIM_T1H3            : 1;      // bit 3
      BITS   TIM_T1H4            : 1;      // bit 4
      BITS   TIM_T1H5            : 1;      // bit 5
      BITS   TIM_T1H6            : 1;      // bit 6
      BITS   TIM_T1H7            : 1;      // bit 7
    };  // TMR_T1H bitfield

    /// register _SMED3_TMR_T1H reset value
    #define sfr_SMED3_TMR_T1H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1H;


  /** Time T2 lsb register (TMR_T2L at 0x55c8) */
  union {

    /// bytewise access to TMR_T2L
    uint8_t  byte;

    /// bitwise access to register TMR_T2L
    struct {
      BITS   TIM_T2L0            : 1;      // bit 0
      BITS   TIM_T2L1            : 1;      // bit 1
      BITS   TIM_T2L2            : 1;      // bit 2
      BITS   TIM_T2L3            : 1;      // bit 3
      BITS   TIM_T2L4            : 1;      // bit 4
      BITS   TIM_T2L5            : 1;      // bit 5
      BITS   TIM_T2L6            : 1;      // bit 6
      BITS   TIM_T2L7            : 1;      // bit 7
    };  // TMR_T2L bitfield

    /// register _SMED3_TMR_T2L reset value
    #define sfr_SMED3_TMR_T2L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2L;


  /** Time T2 msb register (TMR_T2H at 0x55c9) */
  union {

    /// bytewise access to TMR_T2H
    uint8_t  byte;

    /// bitwise access to register TMR_T2H
    struct {
      BITS   TIM_T2H0            : 1;      // bit 0
      BITS   TIM_T2H1            : 1;      // bit 1
      BITS   TIM_T2H2            : 1;      // bit 2
      BITS   TIM_T2H3            : 1;      // bit 3
      BITS   TIM_T2H4            : 1;      // bit 4
      BITS   TIM_T2H5            : 1;      // bit 5
      BITS   TIM_T2H6            : 1;      // bit 6
      BITS   TIM_T2H7            : 1;      // bit 7
    };  // TMR_T2H bitfield

    /// register _SMED3_TMR_T2H reset value
    #define sfr_SMED3_TMR_T2H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2H;


  /** Time T3 lsb register (TMR_T3L at 0x55ca) */
  union {

    /// bytewise access to TMR_T3L
    uint8_t  byte;

    /// bitwise access to register TMR_T3L
    struct {
      BITS   TIM_T3L0            : 1;      // bit 0
      BITS   TIM_T3L1            : 1;      // bit 1
      BITS   TIM_T3L2            : 1;      // bit 2
      BITS   TIM_T3L3            : 1;      // bit 3
      BITS   TIM_T3L4            : 1;      // bit 4
      BITS   TIM_T3L5            : 1;      // bit 5
      BITS   TIM_T3L6            : 1;      // bit 6
      BITS   TIM_T3L7            : 1;      // bit 7
    };  // TMR_T3L bitfield

    /// register _SMED3_TMR_T3L reset value
    #define sfr_SMED3_TMR_T3L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3L;


  /** Time T3 msb register (TMR_T3H at 0x55cb) */
  union {

    /// bytewise access to TMR_T3H
    uint8_t  byte;

    /// bitwise access to register TMR_T3H
    struct {
      BITS   TIM_T3H0            : 1;      // bit 0
      BITS   TIM_T3H1            : 1;      // bit 1
      BITS   TIM_T3H2            : 1;      // bit 2
      BITS   TIM_T3H3            : 1;      // bit 3
      BITS   TIM_T3H4            : 1;      // bit 4
      BITS   TIM_T3H5            : 1;      // bit 5
      BITS   TIM_T3H6            : 1;      // bit 6
      BITS   TIM_T3H7            : 1;      // bit 7
    };  // TMR_T3H bitfield

    /// register _SMED3_TMR_T3H reset value
    #define sfr_SMED3_TMR_T3H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3H;


  /** Parameter 0 IDLE register (PRM_ID0 at 0x55cc) */
  union {

    /// bytewise access to PRM_ID0
    uint8_t  byte;

    /// bitwise access to register PRM_ID0
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_ID0 bitfield

    /// register _SMED3_PRM_ID0 reset value
    #define sfr_SMED3_PRM_ID0_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID0;


  /** Parameter 1 IDLE register (PRM_ID1 at 0x55cd) */
  union {

    /// bytewise access to PRM_ID1
    uint8_t  byte;

    /// bitwise access to register PRM_ID1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_ID1 bitfield

    /// register _SMED3_PRM_ID1 reset value
    #define sfr_SMED3_PRM_ID1_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID1;


  /** Parameter 2 IDLE register (PRM_ID2 at 0x55ce) */
  union {

    /// bytewise access to PRM_ID2
    uint8_t  byte;

    /// bitwise access to register PRM_ID2
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_ID2 bitfield

    /// register _SMED3_PRM_ID2 reset value
    #define sfr_SMED3_PRM_ID2_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID2;


  /** Parameter 0 S0 register (PRM_S00 at 0x55cf) */
  union {

    /// bytewise access to PRM_S00
    uint8_t  byte;

    /// bitwise access to register PRM_S00
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S00 bitfield

    /// register _SMED3_PRM_S00 reset value
    #define sfr_SMED3_PRM_S00_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S00;


  /** Parameter 1 S0 register (PRM_S01 at 0x55d0) */
  union {

    /// bytewise access to PRM_S01
    uint8_t  byte;

    /// bitwise access to register PRM_S01
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S01 bitfield

    /// register _SMED3_PRM_S01 reset value
    #define sfr_SMED3_PRM_S01_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S01;


  /** Parameter 2 S0 register (PRM_S02 at 0x55d1) */
  union {

    /// bytewise access to PRM_S02
    uint8_t  byte;

    /// bitwise access to register PRM_S02
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S02 bitfield

    /// register _SMED3_PRM_S02 reset value
    #define sfr_SMED3_PRM_S02_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S02;


  /** Parameter 0 S1 register (PRM_S10 at 0x55d2) */
  union {

    /// bytewise access to PRM_S10
    uint8_t  byte;

    /// bitwise access to register PRM_S10
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S10 bitfield

    /// register _SMED3_PRM_S10 reset value
    #define sfr_SMED3_PRM_S10_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S10;


  /** Parameter 1 S1 register (PRM_S11 at 0x55d3) */
  union {

    /// bytewise access to PRM_S11
    uint8_t  byte;

    /// bitwise access to register PRM_S11
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S11 bitfield

    /// register _SMED3_PRM_S11 reset value
    #define sfr_SMED3_PRM_S11_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S11;


  /** Parameter 2 S1 register (PRM_S12 at 0x55d4) */
  union {

    /// bytewise access to PRM_S12
    uint8_t  byte;

    /// bitwise access to register PRM_S12
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S12 bitfield

    /// register _SMED3_PRM_S12 reset value
    #define sfr_SMED3_PRM_S12_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S12;


  /** Parameter 0 S2 register (PRM_S20 at 0x55d5) */
  union {

    /// bytewise access to PRM_S20
    uint8_t  byte;

    /// bitwise access to register PRM_S20
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S20 bitfield

    /// register _SMED3_PRM_S20 reset value
    #define sfr_SMED3_PRM_S20_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S20;


  /** Parameter 1 S2 register (PRM_S21 at 0x55d6) */
  union {

    /// bytewise access to PRM_S21
    uint8_t  byte;

    /// bitwise access to register PRM_S21
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S21 bitfield

    /// register _SMED3_PRM_S21 reset value
    #define sfr_SMED3_PRM_S21_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S21;


  /** Parameter 2 S2 register (PRM_S22 at 0x55d7) */
  union {

    /// bytewise access to PRM_S22
    uint8_t  byte;

    /// bitwise access to register PRM_S22
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S22 bitfield

    /// register _SMED3_PRM_S22 reset value
    #define sfr_SMED3_PRM_S22_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S22;


  /** Parameter 0 S3 register (PRM_S30 at 0x55d8) */
  union {

    /// bytewise access to PRM_S30
    uint8_t  byte;

    /// bitwise access to register PRM_S30
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S30 bitfield

    /// register _SMED3_PRM_S30 reset value
    #define sfr_SMED3_PRM_S30_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S30;


  /** Parameter 1 S3 register (PRM_S31 at 0x55d9) */
  union {

    /// bytewise access to PRM_S31
    uint8_t  byte;

    /// bitwise access to register PRM_S31
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S31 bitfield

    /// register _SMED3_PRM_S31 reset value
    #define sfr_SMED3_PRM_S31_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S31;


  /** Parameter 2 S3 register (PRM_S32 at 0x55da) */
  union {

    /// bytewise access to PRM_S32
    uint8_t  byte;

    /// bitwise access to register PRM_S32
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S32 bitfield

    /// register _SMED3_PRM_S32 reset value
    #define sfr_SMED3_PRM_S32_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S32;


  /** Timer configuration register (CFG at 0x55db) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TIM_NUM0            : 1;      // bit 1
      BITS   TIM_NUM1            : 1;      // bit 2
      BITS   TIM_UPD0            : 1;      // bit 3
      BITS   TIM_UPD1            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _SMED3_CFG reset value
    #define sfr_SMED3_CFG_RESET_VALUE   ((uint8_t) 0x00)

  } CFG;


  /** Dump counter lsb register (DMP_L at 0x55dc) */
  union {

    /// bytewise access to DMP_L
    uint8_t  byte;

    /// bitwise access to register DMP_L
    struct {
      BITS   CNT_LO0             : 1;      // bit 0
      BITS   CNT_LO1             : 1;      // bit 1
      BITS   CNT_LO2             : 1;      // bit 2
      BITS   CNT_LO3             : 1;      // bit 3
      BITS   CNT_LO4             : 1;      // bit 4
      BITS   CNT_LO5             : 1;      // bit 5
      BITS   CNT_LO6             : 1;      // bit 6
      BITS   CNT_LO7             : 1;      // bit 7
    };  // DMP_L bitfield

    /// register _SMED3_DMP_L reset value
    #define sfr_SMED3_DMP_L_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_L;


  /** Dump counter msb register (DMP_H at 0x55dd) */
  union {

    /// bytewise access to DMP_H
    uint8_t  byte;

    /// bitwise access to register DMP_H
    struct {
      BITS   CNT_LH0             : 1;      // bit 0
      BITS   CNT_LH1             : 1;      // bit 1
      BITS   CNT_LH2             : 1;      // bit 2
      BITS   CNT_LH3             : 1;      // bit 3
      BITS   CNT_LH4             : 1;      // bit 4
      BITS   CNT_LH5             : 1;      // bit 5
      BITS   CNT_LH6             : 1;      // bit 6
      BITS   CNT_LH7             : 1;      // bit 7
    };  // DMP_H bitfield

    /// register _SMED3_DMP_H reset value
    #define sfr_SMED3_DMP_H_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_H;


  /** General status register (GSTS at 0x55de) */
  union {

    /// bytewise access to GSTS
    uint8_t  byte;

    /// bitwise access to register GSTS
    struct {
      BITS   EX0_DUMP            : 1;      // bit 0
      BITS   EX1_DUMP            : 1;      // bit 1
      BITS   EX2_DUMP            : 1;      // bit 2
      BITS   CNT_FLAG            : 1;      // bit 3
      BITS   DMP_LK0             : 1;      // bit 4
      BITS   DMP_LK1             : 1;      // bit 5
      BITS   EVENT_OV            : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // GSTS bitfield

    /// register _SMED3_GSTS reset value
    #define sfr_SMED3_GSTS_RESET_VALUE   ((uint8_t) 0x00)

  } GSTS;


  /** Interrupt status register (ISR at 0x55df) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS   CNT_OVER            : 1;      // bit 0
      BITS   EXT0_INT            : 1;      // bit 1
      BITS   EXT1_INT            : 1;      // bit 2
      BITS   EXT2_INT            : 1;      // bit 3
      BITS   STA_S0_IT           : 1;      // bit 4
      BITS   STA_S1_IT           : 1;      // bit 5
      BITS   STA_S2_IT           : 1;      // bit 6
      BITS   STA_S3_IT           : 1;      // bit 7
    };  // ISR bitfield

    /// register _SMED3_ISR reset value
    #define sfr_SMED3_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /** Interrupt mask register (IMR at 0x55e0) */
  union {

    /// bytewise access to IMR
    uint8_t  byte;

    /// bitwise access to register IMR
    struct {
      BITS   CNT_OV_R            : 1;      // bit 0
      BITS   IT_EXT0             : 1;      // bit 1
      BITS   IT_EXT1             : 1;      // bit 2
      BITS   IT_EXT2             : 1;      // bit 3
      BITS   IT_STA_S0           : 1;      // bit 4
      BITS   IT_STA_S1           : 1;      // bit 5
      BITS   IT_STA_S2           : 1;      // bit 6
      BITS   IT_STA_S3           : 1;      // bit 7
    };  // IMR bitfield

    /// register _SMED3_IMR reset value
    #define sfr_SMED3_IMR_RESET_VALUE   ((uint8_t) 0x00)

  } IMR;


  /** External event control register (ISEL at 0x55e1) */
  union {

    /// bytewise access to ISEL
    uint8_t  byte;

    /// bitwise access to register ISEL
    struct {
      BITS   INPUT0_EN           : 1;      // bit 0
      BITS   INPUT1_EN           : 1;      // bit 1
      BITS   INPUT2_EN           : 1;      // bit 2
      BITS   INPUT_LAT           : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // ISEL bitfield

    /// register _SMED3_ISEL reset value
    #define sfr_SMED3_ISEL_RESET_VALUE   ((uint8_t) 0x00)

  } ISEL;


  /** Dump enable register (DMP at 0x55e2) */
  union {

    /// bytewise access to DMP
    uint8_t  byte;

    /// bitwise access to register DMP
    struct {
      BITS   DMPE_EX0            : 1;      // bit 0
      BITS   DMPE_EX1            : 1;      // bit 1
      BITS   DMPE_EX2            : 1;      // bit 2
      BITS   DMP_EVER            : 1;      // bit 3
      BITS   CPL_IT_GE           : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // DMP bitfield

    /// register _SMED3_DMP reset value
    #define sfr_SMED3_DMP_RESET_VALUE   ((uint8_t) 0x00)

  } DMP;


  /** FSM status register (FSM_STS at 0x55e3) */
  union {

    /// bytewise access to FSM_STS
    uint8_t  byte;

    /// bitwise access to register FSM_STS
    struct {
      BITS   FSM0                : 1;      // bit 0
      BITS   FSM1                : 1;      // bit 1
      BITS   FSM2                : 1;      // bit 2
      BITS   PWM                 : 1;      // bit 3
      BITS   EVINP0              : 1;      // bit 4
      BITS   EVINP1              : 1;      // bit 5
      BITS   EVINP2              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FSM_STS bitfield

    /// register _SMED3_FSM_STS reset value
    #define sfr_SMED3_FSM_STS_RESET_VALUE   ((uint8_t) 0x00)

  } FSM_STS;

} SMED3_t;

/// access to SMED3 SFR registers
#define sfr_SMED3   (*((SMED3_t*) 0x55c0))


//------------------------
// Module SMED4
//------------------------

/** struct containing SMED4 module registers */
typedef struct {

  /** Control register (CTR at 0x5600) */
  union {

    /// bytewise access to CTR
    uint8_t  byte;

    /// bitwise access to register CTR
    struct {
      BITS   START_CNT           : 1;      // bit 0
      BITS   FSM_ENA             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CTR bitfield

    /// register _SMED4_CTR reset value
    #define sfr_SMED4_CTR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR;


  /** Control timer register (CTR_TMR at 0x5601) */
  union {

    /// bytewise access to CTR_TMR
    uint8_t  byte;

    /// bitwise access to register CTR_TMR
    struct {
      BITS   TIME_T0_VAL         : 1;      // bit 0
      BITS   TIME_T1_VAL         : 1;      // bit 1
      BITS   TIME_T2_VAL         : 1;      // bit 2
      BITS   TIME_T3_VAL         : 1;      // bit 3
      BITS   DITHER_VAL          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CTR_TMR bitfield

    /// register _SMED4_CTR_TMR reset value
    #define sfr_SMED4_CTR_TMR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_TMR;


  /** Control input register (CTR_INP at 0x5602) */
  union {

    /// bytewise access to CTR_INP
    uint8_t  byte;

    /// bitwise access to register CTR_INP
    struct {
      BITS   RS_INSIG0           : 1;      // bit 0
      BITS   RS_INSIG1           : 1;      // bit 1
      BITS   RS_INSIG2           : 1;      // bit 2
      BITS   RAIS_EN             : 1;      // bit 3
      BITS   EL_INSIG0           : 1;      // bit 4
      BITS   EL_INSIG1           : 1;      // bit 5
      BITS   EL_INSIG2           : 1;      // bit 6
      BITS   EL_EN               : 1;      // bit 7
    };  // CTR_INP bitfield

    /// register _SMED4_CTR_INP reset value
    #define sfr_SMED4_CTR_INP_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_INP;


  /** Dithering register (CTR_DTHR at 0x5603) */
  union {

    /// bytewise access to CTR_DTHR
    uint8_t  byte;

    /// bitwise access to register CTR_DTHR
    struct {
      BITS   DITH0               : 1;      // bit 0
      BITS   DITH1               : 1;      // bit 1
      BITS   DITH2               : 1;      // bit 2
      BITS   DITH3               : 1;      // bit 3
      BITS   DITH4               : 1;      // bit 4
      BITS   DITH5               : 1;      // bit 5
      BITS   DITH6               : 1;      // bit 6
      BITS   DITH7               : 1;      // bit 7
    };  // CTR_DTHR bitfield

    /// register _SMED4_CTR_DTHR reset value
    #define sfr_SMED4_CTR_DTHR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_DTHR;


  /** Time T0 lsb register (TMR_T0L at 0x5604) */
  union {

    /// bytewise access to TMR_T0L
    uint8_t  byte;

    /// bitwise access to register TMR_T0L
    struct {
      BITS   TIM_T0L0            : 1;      // bit 0
      BITS   TIM_T0L1            : 1;      // bit 1
      BITS   TIM_T0L2            : 1;      // bit 2
      BITS   TIM_T0L3            : 1;      // bit 3
      BITS   TIM_T0L4            : 1;      // bit 4
      BITS   TIM_T0L5            : 1;      // bit 5
      BITS   TIM_T0L6            : 1;      // bit 6
      BITS   TIM_T0L7            : 1;      // bit 7
    };  // TMR_T0L bitfield

    /// register _SMED4_TMR_T0L reset value
    #define sfr_SMED4_TMR_T0L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0L;


  /** Time T0 msb register (TMR_T0H at 0x5605) */
  union {

    /// bytewise access to TMR_T0H
    uint8_t  byte;

    /// bitwise access to register TMR_T0H
    struct {
      BITS   TIM_T0H0            : 1;      // bit 0
      BITS   TIM_T0H1            : 1;      // bit 1
      BITS   TIM_T0H2            : 1;      // bit 2
      BITS   TIM_T0H3            : 1;      // bit 3
      BITS   TIM_T0H4            : 1;      // bit 4
      BITS   TIM_T0H5            : 1;      // bit 5
      BITS   TIM_T0H6            : 1;      // bit 6
      BITS   TIM_T0H7            : 1;      // bit 7
    };  // TMR_T0H bitfield

    /// register _SMED4_TMR_T0H reset value
    #define sfr_SMED4_TMR_T0H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0H;


  /** Time T1 lsb register (TMR_T1L at 0x5606) */
  union {

    /// bytewise access to TMR_T1L
    uint8_t  byte;

    /// bitwise access to register TMR_T1L
    struct {
      BITS   TIM_T1L0            : 1;      // bit 0
      BITS   TIM_T1L1            : 1;      // bit 1
      BITS   TIM_T1L2            : 1;      // bit 2
      BITS   TIM_T1L3            : 1;      // bit 3
      BITS   TIM_T1L4            : 1;      // bit 4
      BITS   TIM_T1L5            : 1;      // bit 5
      BITS   TIM_T1L6            : 1;      // bit 6
      BITS   TIM_T1L7            : 1;      // bit 7
    };  // TMR_T1L bitfield

    /// register _SMED4_TMR_T1L reset value
    #define sfr_SMED4_TMR_T1L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1L;


  /** Time T1 msb register (TMR_T1H at 0x5607) */
  union {

    /// bytewise access to TMR_T1H
    uint8_t  byte;

    /// bitwise access to register TMR_T1H
    struct {
      BITS   TIM_T1H0            : 1;      // bit 0
      BITS   TIM_T1H1            : 1;      // bit 1
      BITS   TIM_T1H2            : 1;      // bit 2
      BITS   TIM_T1H3            : 1;      // bit 3
      BITS   TIM_T1H4            : 1;      // bit 4
      BITS   TIM_T1H5            : 1;      // bit 5
      BITS   TIM_T1H6            : 1;      // bit 6
      BITS   TIM_T1H7            : 1;      // bit 7
    };  // TMR_T1H bitfield

    /// register _SMED4_TMR_T1H reset value
    #define sfr_SMED4_TMR_T1H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1H;


  /** Time T2 lsb register (TMR_T2L at 0x5608) */
  union {

    /// bytewise access to TMR_T2L
    uint8_t  byte;

    /// bitwise access to register TMR_T2L
    struct {
      BITS   TIM_T2L0            : 1;      // bit 0
      BITS   TIM_T2L1            : 1;      // bit 1
      BITS   TIM_T2L2            : 1;      // bit 2
      BITS   TIM_T2L3            : 1;      // bit 3
      BITS   TIM_T2L4            : 1;      // bit 4
      BITS   TIM_T2L5            : 1;      // bit 5
      BITS   TIM_T2L6            : 1;      // bit 6
      BITS   TIM_T2L7            : 1;      // bit 7
    };  // TMR_T2L bitfield

    /// register _SMED4_TMR_T2L reset value
    #define sfr_SMED4_TMR_T2L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2L;


  /** Time T2 msb register (TMR_T2H at 0x5609) */
  union {

    /// bytewise access to TMR_T2H
    uint8_t  byte;

    /// bitwise access to register TMR_T2H
    struct {
      BITS   TIM_T2H0            : 1;      // bit 0
      BITS   TIM_T2H1            : 1;      // bit 1
      BITS   TIM_T2H2            : 1;      // bit 2
      BITS   TIM_T2H3            : 1;      // bit 3
      BITS   TIM_T2H4            : 1;      // bit 4
      BITS   TIM_T2H5            : 1;      // bit 5
      BITS   TIM_T2H6            : 1;      // bit 6
      BITS   TIM_T2H7            : 1;      // bit 7
    };  // TMR_T2H bitfield

    /// register _SMED4_TMR_T2H reset value
    #define sfr_SMED4_TMR_T2H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2H;


  /** Time T3 lsb register (TMR_T3L at 0x560a) */
  union {

    /// bytewise access to TMR_T3L
    uint8_t  byte;

    /// bitwise access to register TMR_T3L
    struct {
      BITS   TIM_T3L0            : 1;      // bit 0
      BITS   TIM_T3L1            : 1;      // bit 1
      BITS   TIM_T3L2            : 1;      // bit 2
      BITS   TIM_T3L3            : 1;      // bit 3
      BITS   TIM_T3L4            : 1;      // bit 4
      BITS   TIM_T3L5            : 1;      // bit 5
      BITS   TIM_T3L6            : 1;      // bit 6
      BITS   TIM_T3L7            : 1;      // bit 7
    };  // TMR_T3L bitfield

    /// register _SMED4_TMR_T3L reset value
    #define sfr_SMED4_TMR_T3L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3L;


  /** Time T3 msb register (TMR_T3H at 0x560b) */
  union {

    /// bytewise access to TMR_T3H
    uint8_t  byte;

    /// bitwise access to register TMR_T3H
    struct {
      BITS   TIM_T3H0            : 1;      // bit 0
      BITS   TIM_T3H1            : 1;      // bit 1
      BITS   TIM_T3H2            : 1;      // bit 2
      BITS   TIM_T3H3            : 1;      // bit 3
      BITS   TIM_T3H4            : 1;      // bit 4
      BITS   TIM_T3H5            : 1;      // bit 5
      BITS   TIM_T3H6            : 1;      // bit 6
      BITS   TIM_T3H7            : 1;      // bit 7
    };  // TMR_T3H bitfield

    /// register _SMED4_TMR_T3H reset value
    #define sfr_SMED4_TMR_T3H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3H;


  /** Parameter 0 IDLE register (PRM_ID0 at 0x560c) */
  union {

    /// bytewise access to PRM_ID0
    uint8_t  byte;

    /// bitwise access to register PRM_ID0
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_ID0 bitfield

    /// register _SMED4_PRM_ID0 reset value
    #define sfr_SMED4_PRM_ID0_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID0;


  /** Parameter 1 IDLE register (PRM_ID1 at 0x560d) */
  union {

    /// bytewise access to PRM_ID1
    uint8_t  byte;

    /// bitwise access to register PRM_ID1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_ID1 bitfield

    /// register _SMED4_PRM_ID1 reset value
    #define sfr_SMED4_PRM_ID1_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID1;


  /** Parameter 2 IDLE register (PRM_ID2 at 0x560e) */
  union {

    /// bytewise access to PRM_ID2
    uint8_t  byte;

    /// bitwise access to register PRM_ID2
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_ID2 bitfield

    /// register _SMED4_PRM_ID2 reset value
    #define sfr_SMED4_PRM_ID2_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID2;


  /** Parameter 0 S0 register (PRM_S00 at 0x560f) */
  union {

    /// bytewise access to PRM_S00
    uint8_t  byte;

    /// bitwise access to register PRM_S00
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S00 bitfield

    /// register _SMED4_PRM_S00 reset value
    #define sfr_SMED4_PRM_S00_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S00;


  /** Parameter 1 S0 register (PRM_S01 at 0x5610) */
  union {

    /// bytewise access to PRM_S01
    uint8_t  byte;

    /// bitwise access to register PRM_S01
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S01 bitfield

    /// register _SMED4_PRM_S01 reset value
    #define sfr_SMED4_PRM_S01_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S01;


  /** Parameter 2 S0 register (PRM_S02 at 0x5611) */
  union {

    /// bytewise access to PRM_S02
    uint8_t  byte;

    /// bitwise access to register PRM_S02
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S02 bitfield

    /// register _SMED4_PRM_S02 reset value
    #define sfr_SMED4_PRM_S02_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S02;


  /** Parameter 0 S1 register (PRM_S10 at 0x5612) */
  union {

    /// bytewise access to PRM_S10
    uint8_t  byte;

    /// bitwise access to register PRM_S10
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S10 bitfield

    /// register _SMED4_PRM_S10 reset value
    #define sfr_SMED4_PRM_S10_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S10;


  /** Parameter 1 S1 register (PRM_S11 at 0x5613) */
  union {

    /// bytewise access to PRM_S11
    uint8_t  byte;

    /// bitwise access to register PRM_S11
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S11 bitfield

    /// register _SMED4_PRM_S11 reset value
    #define sfr_SMED4_PRM_S11_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S11;


  /** Parameter 2 S1 register (PRM_S12 at 0x5614) */
  union {

    /// bytewise access to PRM_S12
    uint8_t  byte;

    /// bitwise access to register PRM_S12
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S12 bitfield

    /// register _SMED4_PRM_S12 reset value
    #define sfr_SMED4_PRM_S12_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S12;


  /** Parameter 0 S2 register (PRM_S20 at 0x5615) */
  union {

    /// bytewise access to PRM_S20
    uint8_t  byte;

    /// bitwise access to register PRM_S20
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S20 bitfield

    /// register _SMED4_PRM_S20 reset value
    #define sfr_SMED4_PRM_S20_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S20;


  /** Parameter 1 S2 register (PRM_S21 at 0x5616) */
  union {

    /// bytewise access to PRM_S21
    uint8_t  byte;

    /// bitwise access to register PRM_S21
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S21 bitfield

    /// register _SMED4_PRM_S21 reset value
    #define sfr_SMED4_PRM_S21_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S21;


  /** Parameter 2 S2 register (PRM_S22 at 0x5617) */
  union {

    /// bytewise access to PRM_S22
    uint8_t  byte;

    /// bitwise access to register PRM_S22
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S22 bitfield

    /// register _SMED4_PRM_S22 reset value
    #define sfr_SMED4_PRM_S22_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S22;


  /** Parameter 0 S3 register (PRM_S30 at 0x5618) */
  union {

    /// bytewise access to PRM_S30
    uint8_t  byte;

    /// bitwise access to register PRM_S30
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S30 bitfield

    /// register _SMED4_PRM_S30 reset value
    #define sfr_SMED4_PRM_S30_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S30;


  /** Parameter 1 S3 register (PRM_S31 at 0x5619) */
  union {

    /// bytewise access to PRM_S31
    uint8_t  byte;

    /// bitwise access to register PRM_S31
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S31 bitfield

    /// register _SMED4_PRM_S31 reset value
    #define sfr_SMED4_PRM_S31_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S31;


  /** Parameter 2 S3 register (PRM_S32 at 0x561a) */
  union {

    /// bytewise access to PRM_S32
    uint8_t  byte;

    /// bitwise access to register PRM_S32
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S32 bitfield

    /// register _SMED4_PRM_S32 reset value
    #define sfr_SMED4_PRM_S32_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S32;


  /** Timer configuration register (CFG at 0x561b) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TIM_NUM0            : 1;      // bit 1
      BITS   TIM_NUM1            : 1;      // bit 2
      BITS   TIM_UPD0            : 1;      // bit 3
      BITS   TIM_UPD1            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _SMED4_CFG reset value
    #define sfr_SMED4_CFG_RESET_VALUE   ((uint8_t) 0x00)

  } CFG;


  /** Dump counter lsb register (DMP_L at 0x561c) */
  union {

    /// bytewise access to DMP_L
    uint8_t  byte;

    /// bitwise access to register DMP_L
    struct {
      BITS   CNT_LO0             : 1;      // bit 0
      BITS   CNT_LO1             : 1;      // bit 1
      BITS   CNT_LO2             : 1;      // bit 2
      BITS   CNT_LO3             : 1;      // bit 3
      BITS   CNT_LO4             : 1;      // bit 4
      BITS   CNT_LO5             : 1;      // bit 5
      BITS   CNT_LO6             : 1;      // bit 6
      BITS   CNT_LO7             : 1;      // bit 7
    };  // DMP_L bitfield

    /// register _SMED4_DMP_L reset value
    #define sfr_SMED4_DMP_L_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_L;


  /** Dump counter msb register (DMP_H at 0x561d) */
  union {

    /// bytewise access to DMP_H
    uint8_t  byte;

    /// bitwise access to register DMP_H
    struct {
      BITS   CNT_LH0             : 1;      // bit 0
      BITS   CNT_LH1             : 1;      // bit 1
      BITS   CNT_LH2             : 1;      // bit 2
      BITS   CNT_LH3             : 1;      // bit 3
      BITS   CNT_LH4             : 1;      // bit 4
      BITS   CNT_LH5             : 1;      // bit 5
      BITS   CNT_LH6             : 1;      // bit 6
      BITS   CNT_LH7             : 1;      // bit 7
    };  // DMP_H bitfield

    /// register _SMED4_DMP_H reset value
    #define sfr_SMED4_DMP_H_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_H;


  /** General status register (GSTS at 0x561e) */
  union {

    /// bytewise access to GSTS
    uint8_t  byte;

    /// bitwise access to register GSTS
    struct {
      BITS   EX0_DUMP            : 1;      // bit 0
      BITS   EX1_DUMP            : 1;      // bit 1
      BITS   EX2_DUMP            : 1;      // bit 2
      BITS   CNT_FLAG            : 1;      // bit 3
      BITS   DMP_LK0             : 1;      // bit 4
      BITS   DMP_LK1             : 1;      // bit 5
      BITS   EVENT_OV            : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // GSTS bitfield

    /// register _SMED4_GSTS reset value
    #define sfr_SMED4_GSTS_RESET_VALUE   ((uint8_t) 0x00)

  } GSTS;


  /** Interrupt status register (ISR at 0x561f) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS   CNT_OVER            : 1;      // bit 0
      BITS   EXT0_INT            : 1;      // bit 1
      BITS   EXT1_INT            : 1;      // bit 2
      BITS   EXT2_INT            : 1;      // bit 3
      BITS   STA_S0_IT           : 1;      // bit 4
      BITS   STA_S1_IT           : 1;      // bit 5
      BITS   STA_S2_IT           : 1;      // bit 6
      BITS   STA_S3_IT           : 1;      // bit 7
    };  // ISR bitfield

    /// register _SMED4_ISR reset value
    #define sfr_SMED4_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /** Interrupt mask register (IMR at 0x5620) */
  union {

    /// bytewise access to IMR
    uint8_t  byte;

    /// bitwise access to register IMR
    struct {
      BITS   CNT_OV_R            : 1;      // bit 0
      BITS   IT_EXT0             : 1;      // bit 1
      BITS   IT_EXT1             : 1;      // bit 2
      BITS   IT_EXT2             : 1;      // bit 3
      BITS   IT_STA_S0           : 1;      // bit 4
      BITS   IT_STA_S1           : 1;      // bit 5
      BITS   IT_STA_S2           : 1;      // bit 6
      BITS   IT_STA_S3           : 1;      // bit 7
    };  // IMR bitfield

    /// register _SMED4_IMR reset value
    #define sfr_SMED4_IMR_RESET_VALUE   ((uint8_t) 0x00)

  } IMR;


  /** External event control register (ISEL at 0x5621) */
  union {

    /// bytewise access to ISEL
    uint8_t  byte;

    /// bitwise access to register ISEL
    struct {
      BITS   INPUT0_EN           : 1;      // bit 0
      BITS   INPUT1_EN           : 1;      // bit 1
      BITS   INPUT2_EN           : 1;      // bit 2
      BITS   INPUT_LAT           : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // ISEL bitfield

    /// register _SMED4_ISEL reset value
    #define sfr_SMED4_ISEL_RESET_VALUE   ((uint8_t) 0x00)

  } ISEL;


  /** Dump enable register (DMP at 0x5622) */
  union {

    /// bytewise access to DMP
    uint8_t  byte;

    /// bitwise access to register DMP
    struct {
      BITS   DMPE_EX0            : 1;      // bit 0
      BITS   DMPE_EX1            : 1;      // bit 1
      BITS   DMPE_EX2            : 1;      // bit 2
      BITS   DMP_EVER            : 1;      // bit 3
      BITS   CPL_IT_GE           : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // DMP bitfield

    /// register _SMED4_DMP reset value
    #define sfr_SMED4_DMP_RESET_VALUE   ((uint8_t) 0x00)

  } DMP;


  /** FSM status register (FSM_STS at 0x5623) */
  union {

    /// bytewise access to FSM_STS
    uint8_t  byte;

    /// bitwise access to register FSM_STS
    struct {
      BITS   FSM0                : 1;      // bit 0
      BITS   FSM1                : 1;      // bit 1
      BITS   FSM2                : 1;      // bit 2
      BITS   PWM                 : 1;      // bit 3
      BITS   EVINP0              : 1;      // bit 4
      BITS   EVINP1              : 1;      // bit 5
      BITS   EVINP2              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FSM_STS bitfield

    /// register _SMED4_FSM_STS reset value
    #define sfr_SMED4_FSM_STS_RESET_VALUE   ((uint8_t) 0x00)

  } FSM_STS;

} SMED4_t;

/// access to SMED4 SFR registers
#define sfr_SMED4   (*((SMED4_t*) 0x5600))


//------------------------
// Module SMED5
//------------------------

/** struct containing SMED5 module registers */
typedef struct {

  /** Control register (CTR at 0x5640) */
  union {

    /// bytewise access to CTR
    uint8_t  byte;

    /// bitwise access to register CTR
    struct {
      BITS   START_CNT           : 1;      // bit 0
      BITS   FSM_ENA             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CTR bitfield

    /// register _SMED5_CTR reset value
    #define sfr_SMED5_CTR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR;


  /** Control timer register (CTR_TMR at 0x5641) */
  union {

    /// bytewise access to CTR_TMR
    uint8_t  byte;

    /// bitwise access to register CTR_TMR
    struct {
      BITS   TIME_T0_VAL         : 1;      // bit 0
      BITS   TIME_T1_VAL         : 1;      // bit 1
      BITS   TIME_T2_VAL         : 1;      // bit 2
      BITS   TIME_T3_VAL         : 1;      // bit 3
      BITS   DITHER_VAL          : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CTR_TMR bitfield

    /// register _SMED5_CTR_TMR reset value
    #define sfr_SMED5_CTR_TMR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_TMR;


  /** Control input register (CTR_INP at 0x5642) */
  union {

    /// bytewise access to CTR_INP
    uint8_t  byte;

    /// bitwise access to register CTR_INP
    struct {
      BITS   RS_INSIG0           : 1;      // bit 0
      BITS   RS_INSIG1           : 1;      // bit 1
      BITS   RS_INSIG2           : 1;      // bit 2
      BITS   RAIS_EN             : 1;      // bit 3
      BITS   EL_INSIG0           : 1;      // bit 4
      BITS   EL_INSIG1           : 1;      // bit 5
      BITS   EL_INSIG2           : 1;      // bit 6
      BITS   EL_EN               : 1;      // bit 7
    };  // CTR_INP bitfield

    /// register _SMED5_CTR_INP reset value
    #define sfr_SMED5_CTR_INP_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_INP;


  /** Dithering register (CTR_DTHR at 0x5643) */
  union {

    /// bytewise access to CTR_DTHR
    uint8_t  byte;

    /// bitwise access to register CTR_DTHR
    struct {
      BITS   DITH0               : 1;      // bit 0
      BITS   DITH1               : 1;      // bit 1
      BITS   DITH2               : 1;      // bit 2
      BITS   DITH3               : 1;      // bit 3
      BITS   DITH4               : 1;      // bit 4
      BITS   DITH5               : 1;      // bit 5
      BITS   DITH6               : 1;      // bit 6
      BITS   DITH7               : 1;      // bit 7
    };  // CTR_DTHR bitfield

    /// register _SMED5_CTR_DTHR reset value
    #define sfr_SMED5_CTR_DTHR_RESET_VALUE   ((uint8_t) 0x00)

  } CTR_DTHR;


  /** Time T0 lsb register (TMR_T0L at 0x5644) */
  union {

    /// bytewise access to TMR_T0L
    uint8_t  byte;

    /// bitwise access to register TMR_T0L
    struct {
      BITS   TIM_T0L0            : 1;      // bit 0
      BITS   TIM_T0L1            : 1;      // bit 1
      BITS   TIM_T0L2            : 1;      // bit 2
      BITS   TIM_T0L3            : 1;      // bit 3
      BITS   TIM_T0L4            : 1;      // bit 4
      BITS   TIM_T0L5            : 1;      // bit 5
      BITS   TIM_T0L6            : 1;      // bit 6
      BITS   TIM_T0L7            : 1;      // bit 7
    };  // TMR_T0L bitfield

    /// register _SMED5_TMR_T0L reset value
    #define sfr_SMED5_TMR_T0L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0L;


  /** Time T0 msb register (TMR_T0H at 0x5645) */
  union {

    /// bytewise access to TMR_T0H
    uint8_t  byte;

    /// bitwise access to register TMR_T0H
    struct {
      BITS   TIM_T0H0            : 1;      // bit 0
      BITS   TIM_T0H1            : 1;      // bit 1
      BITS   TIM_T0H2            : 1;      // bit 2
      BITS   TIM_T0H3            : 1;      // bit 3
      BITS   TIM_T0H4            : 1;      // bit 4
      BITS   TIM_T0H5            : 1;      // bit 5
      BITS   TIM_T0H6            : 1;      // bit 6
      BITS   TIM_T0H7            : 1;      // bit 7
    };  // TMR_T0H bitfield

    /// register _SMED5_TMR_T0H reset value
    #define sfr_SMED5_TMR_T0H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T0H;


  /** Time T1 lsb register (TMR_T1L at 0x5646) */
  union {

    /// bytewise access to TMR_T1L
    uint8_t  byte;

    /// bitwise access to register TMR_T1L
    struct {
      BITS   TIM_T1L0            : 1;      // bit 0
      BITS   TIM_T1L1            : 1;      // bit 1
      BITS   TIM_T1L2            : 1;      // bit 2
      BITS   TIM_T1L3            : 1;      // bit 3
      BITS   TIM_T1L4            : 1;      // bit 4
      BITS   TIM_T1L5            : 1;      // bit 5
      BITS   TIM_T1L6            : 1;      // bit 6
      BITS   TIM_T1L7            : 1;      // bit 7
    };  // TMR_T1L bitfield

    /// register _SMED5_TMR_T1L reset value
    #define sfr_SMED5_TMR_T1L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1L;


  /** Time T1 msb register (TMR_T1H at 0x5647) */
  union {

    /// bytewise access to TMR_T1H
    uint8_t  byte;

    /// bitwise access to register TMR_T1H
    struct {
      BITS   TIM_T1H0            : 1;      // bit 0
      BITS   TIM_T1H1            : 1;      // bit 1
      BITS   TIM_T1H2            : 1;      // bit 2
      BITS   TIM_T1H3            : 1;      // bit 3
      BITS   TIM_T1H4            : 1;      // bit 4
      BITS   TIM_T1H5            : 1;      // bit 5
      BITS   TIM_T1H6            : 1;      // bit 6
      BITS   TIM_T1H7            : 1;      // bit 7
    };  // TMR_T1H bitfield

    /// register _SMED5_TMR_T1H reset value
    #define sfr_SMED5_TMR_T1H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T1H;


  /** Time T2 lsb register (TMR_T2L at 0x5648) */
  union {

    /// bytewise access to TMR_T2L
    uint8_t  byte;

    /// bitwise access to register TMR_T2L
    struct {
      BITS   TIM_T2L0            : 1;      // bit 0
      BITS   TIM_T2L1            : 1;      // bit 1
      BITS   TIM_T2L2            : 1;      // bit 2
      BITS   TIM_T2L3            : 1;      // bit 3
      BITS   TIM_T2L4            : 1;      // bit 4
      BITS   TIM_T2L5            : 1;      // bit 5
      BITS   TIM_T2L6            : 1;      // bit 6
      BITS   TIM_T2L7            : 1;      // bit 7
    };  // TMR_T2L bitfield

    /// register _SMED5_TMR_T2L reset value
    #define sfr_SMED5_TMR_T2L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2L;


  /** Time T2 msb register (TMR_T2H at 0x5649) */
  union {

    /// bytewise access to TMR_T2H
    uint8_t  byte;

    /// bitwise access to register TMR_T2H
    struct {
      BITS   TIM_T2H0            : 1;      // bit 0
      BITS   TIM_T2H1            : 1;      // bit 1
      BITS   TIM_T2H2            : 1;      // bit 2
      BITS   TIM_T2H3            : 1;      // bit 3
      BITS   TIM_T2H4            : 1;      // bit 4
      BITS   TIM_T2H5            : 1;      // bit 5
      BITS   TIM_T2H6            : 1;      // bit 6
      BITS   TIM_T2H7            : 1;      // bit 7
    };  // TMR_T2H bitfield

    /// register _SMED5_TMR_T2H reset value
    #define sfr_SMED5_TMR_T2H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T2H;


  /** Time T3 lsb register (TMR_T3L at 0x564a) */
  union {

    /// bytewise access to TMR_T3L
    uint8_t  byte;

    /// bitwise access to register TMR_T3L
    struct {
      BITS   TIM_T3L0            : 1;      // bit 0
      BITS   TIM_T3L1            : 1;      // bit 1
      BITS   TIM_T3L2            : 1;      // bit 2
      BITS   TIM_T3L3            : 1;      // bit 3
      BITS   TIM_T3L4            : 1;      // bit 4
      BITS   TIM_T3L5            : 1;      // bit 5
      BITS   TIM_T3L6            : 1;      // bit 6
      BITS   TIM_T3L7            : 1;      // bit 7
    };  // TMR_T3L bitfield

    /// register _SMED5_TMR_T3L reset value
    #define sfr_SMED5_TMR_T3L_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3L;


  /** Time T3 msb register (TMR_T3H at 0x564b) */
  union {

    /// bytewise access to TMR_T3H
    uint8_t  byte;

    /// bitwise access to register TMR_T3H
    struct {
      BITS   TIM_T3H0            : 1;      // bit 0
      BITS   TIM_T3H1            : 1;      // bit 1
      BITS   TIM_T3H2            : 1;      // bit 2
      BITS   TIM_T3H3            : 1;      // bit 3
      BITS   TIM_T3H4            : 1;      // bit 4
      BITS   TIM_T3H5            : 1;      // bit 5
      BITS   TIM_T3H6            : 1;      // bit 6
      BITS   TIM_T3H7            : 1;      // bit 7
    };  // TMR_T3H bitfield

    /// register _SMED5_TMR_T3H reset value
    #define sfr_SMED5_TMR_T3H_RESET_VALUE   ((uint8_t) 0x00)

  } TMR_T3H;


  /** Parameter 0 IDLE register (PRM_ID0 at 0x564c) */
  union {

    /// bytewise access to PRM_ID0
    uint8_t  byte;

    /// bitwise access to register PRM_ID0
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_ID0 bitfield

    /// register _SMED5_PRM_ID0 reset value
    #define sfr_SMED5_PRM_ID0_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID0;


  /** Parameter 1 IDLE register (PRM_ID1 at 0x564d) */
  union {

    /// bytewise access to PRM_ID1
    uint8_t  byte;

    /// bitwise access to register PRM_ID1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_ID1 bitfield

    /// register _SMED5_PRM_ID1 reset value
    #define sfr_SMED5_PRM_ID1_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID1;


  /** Parameter 2 IDLE register (PRM_ID2 at 0x564e) */
  union {

    /// bytewise access to PRM_ID2
    uint8_t  byte;

    /// bitwise access to register PRM_ID2
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_ID2 bitfield

    /// register _SMED5_PRM_ID2 reset value
    #define sfr_SMED5_PRM_ID2_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_ID2;


  /** Parameter 0 S0 register (PRM_S00 at 0x564f) */
  union {

    /// bytewise access to PRM_S00
    uint8_t  byte;

    /// bitwise access to register PRM_S00
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S00 bitfield

    /// register _SMED5_PRM_S00 reset value
    #define sfr_SMED5_PRM_S00_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S00;


  /** Parameter 1 S0 register (PRM_S01 at 0x5650) */
  union {

    /// bytewise access to PRM_S01
    uint8_t  byte;

    /// bitwise access to register PRM_S01
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S01 bitfield

    /// register _SMED5_PRM_S01 reset value
    #define sfr_SMED5_PRM_S01_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S01;


  /** Parameter 2 S0 register (PRM_S02 at 0x5651) */
  union {

    /// bytewise access to PRM_S02
    uint8_t  byte;

    /// bitwise access to register PRM_S02
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S02 bitfield

    /// register _SMED5_PRM_S02 reset value
    #define sfr_SMED5_PRM_S02_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S02;


  /** Parameter 0 S1 register (PRM_S10 at 0x5652) */
  union {

    /// bytewise access to PRM_S10
    uint8_t  byte;

    /// bitwise access to register PRM_S10
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S10 bitfield

    /// register _SMED5_PRM_S10 reset value
    #define sfr_SMED5_PRM_S10_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S10;


  /** Parameter 1 S1 register (PRM_S11 at 0x5653) */
  union {

    /// bytewise access to PRM_S11
    uint8_t  byte;

    /// bitwise access to register PRM_S11
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S11 bitfield

    /// register _SMED5_PRM_S11 reset value
    #define sfr_SMED5_PRM_S11_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S11;


  /** Parameter 2 S1 register (PRM_S12 at 0x5654) */
  union {

    /// bytewise access to PRM_S12
    uint8_t  byte;

    /// bitwise access to register PRM_S12
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S12 bitfield

    /// register _SMED5_PRM_S12 reset value
    #define sfr_SMED5_PRM_S12_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S12;


  /** Parameter 0 S2 register (PRM_S20 at 0x5655) */
  union {

    /// bytewise access to PRM_S20
    uint8_t  byte;

    /// bitwise access to register PRM_S20
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S20 bitfield

    /// register _SMED5_PRM_S20 reset value
    #define sfr_SMED5_PRM_S20_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S20;


  /** Parameter 1 S2 register (PRM_S21 at 0x5656) */
  union {

    /// bytewise access to PRM_S21
    uint8_t  byte;

    /// bitwise access to register PRM_S21
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S21 bitfield

    /// register _SMED5_PRM_S21 reset value
    #define sfr_SMED5_PRM_S21_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S21;


  /** Parameter 2 S2 register (PRM_S22 at 0x5657) */
  union {

    /// bytewise access to PRM_S22
    uint8_t  byte;

    /// bitwise access to register PRM_S22
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S22 bitfield

    /// register _SMED5_PRM_S22 reset value
    #define sfr_SMED5_PRM_S22_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S22;


  /** Parameter 0 S3 register (PRM_S30 at 0x5658) */
  union {

    /// bytewise access to PRM_S30
    uint8_t  byte;

    /// bitwise access to register PRM_S30
    struct {
      BITS   NX_STAT0            : 1;      // bit 0
      BITS   NX_STAT1            : 1;      // bit 1
      BITS   EDGE0               : 1;      // bit 2
      BITS   EDGE1               : 1;      // bit 3
      BITS   CNT_RSTE            : 1;      // bit 4
      BITS   PULS_EDG            : 1;      // bit 5
      BITS   HOLD_JMP            : 1;      // bit 6
      BITS   AND_OR              : 1;      // bit 7
    };  // PRM_S30 bitfield

    /// register _SMED5_PRM_S30 reset value
    #define sfr_SMED5_PRM_S30_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S30;


  /** Parameter 1 S3 register (PRM_S31 at 0x5659) */
  union {

    /// bytewise access to PRM_S31
    uint8_t  byte;

    /// bitwise access to register PRM_S31
    struct {
      BITS                       : 2;      // 2 bits
      BITS   CEDGE0              : 1;      // bit 2
      BITS   CEDGE1              : 1;      // bit 3
      BITS   CNT_RSTC            : 1;      // bit 4
      BITS   PULS_CMP            : 1;      // bit 5
      BITS   HOLD_EXIT           : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // PRM_S31 bitfield

    /// register _SMED5_PRM_S31 reset value
    #define sfr_SMED5_PRM_S31_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S31;


  /** Parameter 2 S3 register (PRM_S32 at 0x565a) */
  union {

    /// bytewise access to PRM_S32
    uint8_t  byte;

    /// bitwise access to register PRM_S32
    struct {
      BITS   LATCH_RS            : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // PRM_S32 bitfield

    /// register _SMED5_PRM_S32 reset value
    #define sfr_SMED5_PRM_S32_RESET_VALUE   ((uint8_t) 0x00)

  } PRM_S32;


  /** Timer configuration register (CFG at 0x565b) */
  union {

    /// bytewise access to CFG
    uint8_t  byte;

    /// bitwise access to register CFG
    struct {
      BITS                       : 1;      // 1 bit
      BITS   TIM_NUM0            : 1;      // bit 1
      BITS   TIM_NUM1            : 1;      // bit 2
      BITS   TIM_UPD0            : 1;      // bit 3
      BITS   TIM_UPD1            : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // CFG bitfield

    /// register _SMED5_CFG reset value
    #define sfr_SMED5_CFG_RESET_VALUE   ((uint8_t) 0x00)

  } CFG;


  /** Dump counter lsb register (DMP_L at 0x565c) */
  union {

    /// bytewise access to DMP_L
    uint8_t  byte;

    /// bitwise access to register DMP_L
    struct {
      BITS   CNT_LO0             : 1;      // bit 0
      BITS   CNT_LO1             : 1;      // bit 1
      BITS   CNT_LO2             : 1;      // bit 2
      BITS   CNT_LO3             : 1;      // bit 3
      BITS   CNT_LO4             : 1;      // bit 4
      BITS   CNT_LO5             : 1;      // bit 5
      BITS   CNT_LO6             : 1;      // bit 6
      BITS   CNT_LO7             : 1;      // bit 7
    };  // DMP_L bitfield

    /// register _SMED5_DMP_L reset value
    #define sfr_SMED5_DMP_L_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_L;


  /** Dump counter msb register (DMP_H at 0x565d) */
  union {

    /// bytewise access to DMP_H
    uint8_t  byte;

    /// bitwise access to register DMP_H
    struct {
      BITS   CNT_LH0             : 1;      // bit 0
      BITS   CNT_LH1             : 1;      // bit 1
      BITS   CNT_LH2             : 1;      // bit 2
      BITS   CNT_LH3             : 1;      // bit 3
      BITS   CNT_LH4             : 1;      // bit 4
      BITS   CNT_LH5             : 1;      // bit 5
      BITS   CNT_LH6             : 1;      // bit 6
      BITS   CNT_LH7             : 1;      // bit 7
    };  // DMP_H bitfield

    /// register _SMED5_DMP_H reset value
    #define sfr_SMED5_DMP_H_RESET_VALUE   ((uint8_t) 0x00)

  } DMP_H;


  /** General status register (GSTS at 0x565e) */
  union {

    /// bytewise access to GSTS
    uint8_t  byte;

    /// bitwise access to register GSTS
    struct {
      BITS   EX0_DUMP            : 1;      // bit 0
      BITS   EX1_DUMP            : 1;      // bit 1
      BITS   EX2_DUMP            : 1;      // bit 2
      BITS   CNT_FLAG            : 1;      // bit 3
      BITS   DMP_LK0             : 1;      // bit 4
      BITS   DMP_LK1             : 1;      // bit 5
      BITS   EVENT_OV            : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // GSTS bitfield

    /// register _SMED5_GSTS reset value
    #define sfr_SMED5_GSTS_RESET_VALUE   ((uint8_t) 0x00)

  } GSTS;


  /** Interrupt status register (ISR at 0x565f) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS   CNT_OVER            : 1;      // bit 0
      BITS   EXT0_INT            : 1;      // bit 1
      BITS   EXT1_INT            : 1;      // bit 2
      BITS   EXT2_INT            : 1;      // bit 3
      BITS   STA_S0_IT           : 1;      // bit 4
      BITS   STA_S1_IT           : 1;      // bit 5
      BITS   STA_S2_IT           : 1;      // bit 6
      BITS   STA_S3_IT           : 1;      // bit 7
    };  // ISR bitfield

    /// register _SMED5_ISR reset value
    #define sfr_SMED5_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /** Interrupt mask register (IMR at 0x5660) */
  union {

    /// bytewise access to IMR
    uint8_t  byte;

    /// bitwise access to register IMR
    struct {
      BITS   CNT_OV_R            : 1;      // bit 0
      BITS   IT_EXT0             : 1;      // bit 1
      BITS   IT_EXT1             : 1;      // bit 2
      BITS   IT_EXT2             : 1;      // bit 3
      BITS   IT_STA_S0           : 1;      // bit 4
      BITS   IT_STA_S1           : 1;      // bit 5
      BITS   IT_STA_S2           : 1;      // bit 6
      BITS   IT_STA_S3           : 1;      // bit 7
    };  // IMR bitfield

    /// register _SMED5_IMR reset value
    #define sfr_SMED5_IMR_RESET_VALUE   ((uint8_t) 0x00)

  } IMR;


  /** External event control register (ISEL at 0x5661) */
  union {

    /// bytewise access to ISEL
    uint8_t  byte;

    /// bitwise access to register ISEL
    struct {
      BITS   INPUT0_EN           : 1;      // bit 0
      BITS   INPUT1_EN           : 1;      // bit 1
      BITS   INPUT2_EN           : 1;      // bit 2
      BITS   INPUT_LAT           : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // ISEL bitfield

    /// register _SMED5_ISEL reset value
    #define sfr_SMED5_ISEL_RESET_VALUE   ((uint8_t) 0x00)

  } ISEL;


  /** Dump enable register (DMP at 0x5662) */
  union {

    /// bytewise access to DMP
    uint8_t  byte;

    /// bitwise access to register DMP
    struct {
      BITS   DMPE_EX0            : 1;      // bit 0
      BITS   DMPE_EX1            : 1;      // bit 1
      BITS   DMPE_EX2            : 1;      // bit 2
      BITS   DMP_EVER            : 1;      // bit 3
      BITS   CPL_IT_GE           : 1;      // bit 4
      BITS                       : 3;      // 3 bits
    };  // DMP bitfield

    /// register _SMED5_DMP reset value
    #define sfr_SMED5_DMP_RESET_VALUE   ((uint8_t) 0x00)

  } DMP;


  /** FSM status register (FSM_STS at 0x5663) */
  union {

    /// bytewise access to FSM_STS
    uint8_t  byte;

    /// bitwise access to register FSM_STS
    struct {
      BITS   FSM0                : 1;      // bit 0
      BITS   FSM1                : 1;      // bit 1
      BITS   FSM2                : 1;      // bit 2
      BITS   PWM                 : 1;      // bit 3
      BITS   EVINP0              : 1;      // bit 4
      BITS   EVINP1              : 1;      // bit 5
      BITS   EVINP2              : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // FSM_STS bitfield

    /// register _SMED5_FSM_STS reset value
    #define sfr_SMED5_FSM_STS_RESET_VALUE   ((uint8_t) 0x00)

  } FSM_STS;

} SMED5_t;

/// access to SMED5 SFR registers
#define sfr_SMED5   (*((SMED5_t*) 0x5640))


//------------------------
// Module STMR
//------------------------

/** struct containing STMR module registers */
typedef struct {

  /** Control register 1 (STMR_CR1 at 0x5340) */
  union {

    /// bytewise access to STMR_CR1
    uint8_t  byte;

    /// bitwise access to register STMR_CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS                       : 3;      // 3 bits
      BITS   ARPE                : 1;      // bit 7
    };  // STMR_CR1 bitfield

    /// register _STMR_STMR_CR1 reset value
    #define sfr_STMR_STMR_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_CR1;


  /** Interrupt enable register (STMR_IER at 0x5341) */
  union {

    /// bytewise access to STMR_IER
    uint8_t  byte;

    /// bitwise access to register STMR_IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // STMR_IER bitfield

    /// register _STMR_STMR_IER reset value
    #define sfr_STMR_STMR_IER_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_IER;


  /** Status register (STMR_SR1 at 0x5342) */
  union {

    /// bytewise access to STMR_SR1
    uint8_t  byte;

    /// bitwise access to register STMR_SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // STMR_SR1 bitfield

    /// register _STMR_STMR_SR1 reset value
    #define sfr_STMR_STMR_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_SR1;


  /** Event generation register (STMR_EGR at 0x5343) */
  union {

    /// bytewise access to STMR_EGR
    uint8_t  byte;

    /// bitwise access to register STMR_EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS                       : 7;      // 7 bits
    };  // STMR_EGR bitfield

    /// register _STMR_STMR_EGR reset value
    #define sfr_STMR_STMR_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_EGR;


  /** Counter high register (STMR_CNTH at 0x5344) */
  union {

    /// bytewise access to STMR_CNTH
    uint8_t  byte;

    /// bitwise access to register STMR_CNTH
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // STMR_CNTH bitfield

    /// register _STMR_STMR_CNTH reset value
    #define sfr_STMR_STMR_CNTH_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_CNTH;


  /** Counter low register (STMR_CNTL at 0x5345) */
  union {

    /// bytewise access to STMR_CNTL
    uint8_t  byte;

    /// bitwise access to register STMR_CNTL
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // STMR_CNTL bitfield

    /// register _STMR_STMR_CNTL reset value
    #define sfr_STMR_STMR_CNTL_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_CNTL;


  /** Prescaler register (STMR_PSCL at 0x5346) */
  union {

    /// bytewise access to STMR_PSCL
    uint8_t  byte;

    /// bitwise access to register STMR_PSCL
    struct {
      BITS   PSC                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // STMR_PSCL bitfield

    /// register _STMR_STMR_PSCL reset value
    #define sfr_STMR_STMR_PSCL_RESET_VALUE   ((uint8_t) 0x00)

  } STMR_PSCL;


  /** Auto-reload high register (STMR_ARRH at 0x5347) */
  union {

    /// bytewise access to STMR_ARRH
    uint8_t  byte;

    /// bitwise access to register STMR_ARRH
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // STMR_ARRH bitfield

    /// register _STMR_STMR_ARRH reset value
    #define sfr_STMR_STMR_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } STMR_ARRH;


  /** Auto-reload low register (STMR_ARRL at 0x5348) */
  union {

    /// bytewise access to STMR_ARRL
    uint8_t  byte;

    /// bitwise access to register STMR_ARRL
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // STMR_ARRL bitfield

    /// register _STMR_STMR_ARRL reset value
    #define sfr_STMR_STMR_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } STMR_ARRL;

} STMR_t;

/// access to STMR SFR registers
#define sfr_STMR   (*((STMR_t*) 0x5340))


//------------------------
// Module SYS
//------------------------

/** struct containing SYS module registers */
typedef struct {

  /** Global configuration register (CFG_GCR at 0x7f60) */
  union {

    /// bytewise access to CFG_GCR
    uint8_t  byte;

    /// bitwise access to register CFG_GCR
    struct {
      BITS   SWD                 : 1;      // bit 0
      BITS   AL                  : 1;      // bit 1
      BITS                       : 5;      // 5 bits
      BITS   HSIT                : 1;      // bit 7
    };  // CFG_GCR bitfield

    /// register _SYS_CFG_GCR reset value
    #define sfr_SYS_CFG_GCR_RESET_VALUE   ((uint8_t) 0x00)

  } CFG_GCR;


  /// Reserved register (31B)
  uint8_t     Reserved_1[31];


  /** SWIM control status register (SWIM_CSR at 0x7f80) */
  union {

    /// bytewise access to SWIM_CSR
    uint8_t  byte;

    /// bitwise access to register SWIM_CSR
    struct {
      BITS   SWIM_PRI            : 1;      // bit 0
      BITS   OBL                 : 1;      // bit 1
      BITS   RST                 : 1;      // bit 2
      BITS   OSCOFF              : 1;      // bit 3
      BITS   HS                  : 1;      // bit 4
      BITS   SWIM_DM             : 1;      // bit 5
      BITS   HALT                : 1;      // bit 6
      BITS   SAFE_MASK           : 1;      // bit 7
    };  // SWIM_CSR bitfield

    /// register _SYS_SWIM_CSR reset value
    #define sfr_SYS_SWIM_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } SWIM_CSR;

} SYS_t;

/// access to SYS SFR registers
#define sfr_SYS   (*((SYS_t*) 0x7f60))


//------------------------
// Module UART
//------------------------

/** struct containing UART module registers */
typedef struct {

  /** UART status register (SR at 0x5230) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   PE                  : 1;      // bit 0
      BITS   FE                  : 1;      // bit 1
      BITS   NF                  : 1;      // bit 2
      BITS   OR                  : 1;      // bit 3
      BITS   IDLE                : 1;      // bit 4
      BITS   RXNE                : 1;      // bit 5
      BITS   TC                  : 1;      // bit 6
      BITS   TXE                 : 1;      // bit 7
    };  // SR bitfield

    /// register _UART_SR reset value
    #define sfr_UART_SR_RESET_VALUE   ((uint8_t) 0xC0)

  } SR;


  /** UART data register (DR at 0x5231) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _UART_DR reset value
    #define sfr_UART_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** UART baud rate register 1 (BRR1 at 0x5232) */
  union {

    /// bytewise access to BRR1
    uint8_t  byte;

    /// bitwise access to register BRR1
    struct {
      BITS   UART_DIV_L          : 8;      // bits 0-7
    };  // BRR1 bitfield

    /// register _UART_BRR1 reset value
    #define sfr_UART_BRR1_RESET_VALUE   ((uint8_t) 0x00)

  } BRR1;


  /** UART baud rate register 2 (BRR2 at 0x5233) */
  union {

    /// bytewise access to BRR2
    uint8_t  byte;

    /// bitwise access to register BRR2
    struct {
      BITS   UART_DIV_F          : 4;      // bits 0-3
      BITS   UART_DIV_M          : 4;      // bits 4-7
    };  // BRR2 bitfield

    /// register _UART_BRR2 reset value
    #define sfr_UART_BRR2_RESET_VALUE   ((uint8_t) 0x00)

  } BRR2;


  /** UART control register 1 (CR1 at 0x5234) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   PIEN                : 1;      // bit 0
      BITS   PS                  : 1;      // bit 1
      BITS   PCEN                : 1;      // bit 2
      BITS   WAKE                : 1;      // bit 3
      BITS   M                   : 1;      // bit 4
      BITS   UARTD               : 1;      // bit 5
      BITS   T8                  : 1;      // bit 6
      BITS   R8                  : 1;      // bit 7
    };  // CR1 bitfield

    /// register _UART_CR1 reset value
    #define sfr_UART_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** UART control register 2 (CR2 at 0x5235) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   SBK                 : 1;      // bit 0
      BITS   RWU                 : 1;      // bit 1
      BITS   REN                 : 1;      // bit 2
      BITS   TEN                 : 1;      // bit 3
      BITS   ILIEN               : 1;      // bit 4
      BITS   RIEN                : 1;      // bit 5
      BITS   TCIEN               : 1;      // bit 6
      BITS   TIEN                : 1;      // bit 7
    };  // CR2 bitfield

    /// register _UART_CR2 reset value
    #define sfr_UART_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** UART control register 3 (CR3 at 0x5236) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS                       : 4;      // 4 bits
      BITS   STOP                : 2;      // bits 4-5
      BITS                       : 2;      // 2 bits
    };  // CR3 bitfield

    /// register _UART_CR3 reset value
    #define sfr_UART_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** UART control register 4 (CR4 at 0x5237) */
  union {

    /// bytewise access to CR4
    uint8_t  byte;

    /// bitwise access to register CR4
    struct {
      BITS   ADD                 : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // CR4 bitfield

    /// register _UART_CR4 reset value
    #define sfr_UART_CR4_RESET_VALUE   ((uint8_t) 0x00)

  } CR4;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** UART guard time register (GTR at 0x5239) */
  union {

    /// bytewise access to GTR
    uint8_t  byte;

    /// bitwise access to register GTR
    struct {
      BITS   GT                  : 8;      // bits 0-7
    };  // GTR bitfield

    /// register _UART_GTR reset value
    #define sfr_UART_GTR_RESET_VALUE   ((uint8_t) 0x00)

  } GTR;


  /** UART prescaler register (PSCR at 0x523a) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 8;      // bits 0-7
    };  // PSCR bitfield

    /// register _UART_PSCR reset value
    #define sfr_UART_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;

} UART_t;

/// access to UART SFR registers
#define sfr_UART   (*((UART_t*) 0x5230))


//------------------------
// Module WDG
//------------------------

/** struct containing WDG module registers */
typedef struct {

  /** WWDG control register (WWDG_CR at 0x50d1) */
  union {

    /// bytewise access to WWDG_CR
    uint8_t  byte;

    /// bitwise access to register WWDG_CR
    struct {
      BITS   T0                  : 1;      // bit 0
      BITS   T1                  : 1;      // bit 1
      BITS   T2                  : 1;      // bit 2
      BITS   T3                  : 1;      // bit 3
      BITS   T4                  : 1;      // bit 4
      BITS   T5                  : 1;      // bit 5
      BITS   T6                  : 1;      // bit 6
      BITS   WDGA                : 1;      // bit 7
    };  // WWDG_CR bitfield

    /// register _WDG_WWDG_CR reset value
    #define sfr_WDG_WWDG_CR_RESET_VALUE   ((uint8_t) 0x7F)

  } WWDG_CR;


  /** WWDR window register (WWDG_WR at 0x50d2) */
  union {

    /// bytewise access to WWDG_WR
    uint8_t  byte;

    /// bitwise access to register WWDG_WR
    struct {
      BITS   W0                  : 1;      // bit 0
      BITS   W1                  : 1;      // bit 1
      BITS   W2                  : 1;      // bit 2
      BITS   W3                  : 1;      // bit 3
      BITS   W4                  : 1;      // bit 4
      BITS   W5                  : 1;      // bit 5
      BITS   W6                  : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // WWDG_WR bitfield

    /// register _WDG_WWDG_WR reset value
    #define sfr_WDG_WWDG_WR_RESET_VALUE   ((uint8_t) 0x7F)

  } WWDG_WR;


  /// Reserved register (13B)
  uint8_t     Reserved_1[13];


  /** IWDG key register (IWDG_KR at 0x50e0) */
  union {

    /// bytewise access to IWDG_KR
    uint8_t  byte;

    /// bitwise access to register IWDG_KR
    struct {
      BITS   KEY                 : 8;      // bits 0-7
    };  // IWDG_KR bitfield

    /// register _WDG_IWDG_KR reset value
    #define sfr_WDG_IWDG_KR_RESET_VALUE   ((uint8_t) 0x00)

  } IWDG_KR;


  /** IWDG prescaler register (IWDG_PR at 0x50e1) */
  union {

    /// bytewise access to IWDG_PR
    uint8_t  byte;

    /// bitwise access to register IWDG_PR
    struct {
      BITS   PR                  : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // IWDG_PR bitfield

    /// register _WDG_IWDG_PR reset value
    #define sfr_WDG_IWDG_PR_RESET_VALUE   ((uint8_t) 0x00)

  } IWDG_PR;


  /** IWDG reload register (IWDG_RLR at 0x50e2) */
  union {

    /// bytewise access to IWDG_RLR
    uint8_t  byte;

    /// bitwise access to register IWDG_RLR
    struct {
      BITS   RL                  : 8;      // bits 0-7
    };  // IWDG_RLR bitfield

    /// register _WDG_IWDG_RLR reset value
    #define sfr_WDG_IWDG_RLR_RESET_VALUE   ((uint8_t) 0xFF)

  } IWDG_RLR;

} WDG_t;

/// access to WDG SFR registers
#define sfr_WDG   (*((WDG_t*) 0x50d1))


// undefine local macros
#undef  BITS

// required for C++
#ifdef __cplusplus
  }   // extern "C"
#endif

/*-------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-------------------------------------------------------------------------*/
#endif // STLUX385A_H
