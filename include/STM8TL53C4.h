/*-------------------------------------------------------------------------

  STM8TL53C4.h - Device Declarations

  STM8T without ROM bootloader

  Copyright (C) 2020, Georg Icking-Konert

  8-bit, ultra-low-power, touch-sensing MCUs with 16-Kbyte Flash and proximity detection 

  datasheet: https://www.st.com/resource/en/datasheet/stm8tl53c4.pdf
  reference: RM0312 https://www.st.com/content/ccc/resource/technical/document/reference_manual/b7/ef/a7/83/e9/1c/4e/6a/DM00040094.pdf/files/DM00040094.pdf/jcr:content/translations/en.DM00040094.pdf

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
#ifndef STM8TL53C4_H
#define STM8TL53C4_H

// DEVICE NAME
#define DEVICE_STM8TL53C4

// DEVICE FAMILY
#define FAMILY_STM8T

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
#define RAM_ADDR_END                  0x000FFF
#define RAM_SIZE                      4096


// OPTION
#define OPTION_ADDR_START             0x004800
#define OPTION_ADDR_END               0x0048FF
#define OPTION_SIZE                   256


// ID
#define ID_ADDR_START                 0x004925
#define ID_ADDR_END                   0x004930
#define ID_SIZE                       12


// SFR1
#define SFR1_ADDR_START               0x005000
#define SFR1_ADDR_END                 0x0057FF
#define SFR1_SIZE                     2048


// SFR2
#define SFR2_ADDR_START               0x007F00
#define SFR2_ADDR_END                 0x007FFF
#define SFR2_SIZE                     256


// FLASH
#define FLASH_ADDR_START              0x008000
#define FLASH_ADDR_END                0x00BFFF
#define FLASH_SIZE                    16384


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
#define _FLASH_EOP_VECTOR_                       1          ///< FLASH_EOP interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.EOP, priority: ITC_SPR1.VECT1SPR
#define _FLASH_WR_PG_DIS_VECTOR_                 1          ///< FLASH_WR_PG_DIS interrupt vector: enable: FLASH_CR1.IE, pending: FLASH_IAPSR.WR_PG_DIS, priority: ITC_SPR1.VECT1SPR
#define _PXS_EOC_VECTOR_                         2          ///< PXS_EOC interrupt vector: enable: PXS_CR2.EOCITEN, pending: PXS_ISR.EOCF, priority: ITC_SPR1.VECT2SPR
#define _PXS_FCC_VECTOR_                         2          ///< PXS_FCC interrupt vector: enable: PXS_CR2.FCCITEN, pending: PXS_ISR.FCCF, priority: ITC_SPR1.VECT2SPR
#define _AWU_VECTOR_                             4          ///< AWU interrupt vector: enable: AWU_CSR.AWUEN, pending: AWU_CSR.AWUF, priority: ITC_SPR2.VECT4SPR
#define _EXTIB_VECTOR_                           6          ///< EXTIB interrupt vector: enable: EXTI_CR3.PBIS, pending: EXTI_SR2.PBF, priority: ITC_SPR2.VECT6SPR
#define _EXTID_VECTOR_                           7          ///< EXTID interrupt vector: enable: EXTI_CR3.PDIS, pending: EXTI_SR2.PDF, priority: ITC_SPR2.VECT7SPR
#define _EXTI0_VECTOR_                           8          ///< EXTI0 interrupt vector: enable: EXTI_CR1.P0IS, pending: EXTI_SR1.P0F, priority: ITC_SPR3.VECT8SPR
#define _EXTI1_VECTOR_                           9          ///< EXTI1 interrupt vector: enable: EXTI_CR1.P1IS, pending: EXTI_SR1.P1F, priority: ITC_SPR3.VECT9SPR
#define _EXTI2_VECTOR_                           10         ///< EXTI2 interrupt vector: enable: EXTI_CR1.P2IS, pending: EXTI_SR1.P2F, priority: ITC_SPR3.VECT10SPR
#define _EXTI3_VECTOR_                           11         ///< EXTI3 interrupt vector: enable: EXTI_CR1.P3IS, pending: EXTI_SR1.P3F, priority: ITC_SPR3.VECT11SPR
#define _EXTI4_VECTOR_                           12         ///< EXTI4 interrupt vector: enable: EXTI_CR2.P4IS, pending: EXTI_SR1.P4F, priority: ITC_SPR4.VECT12SPR
#define _EXTI5_VECTOR_                           13         ///< EXTI5 interrupt vector: enable: EXTI_CR2.P5IS, pending: EXTI_SR1.P5F, priority: ITC_SPR4.VECT13SPR
#define _EXTI6_VECTOR_                           14         ///< EXTI6 interrupt vector: enable: EXTI_CR2.P6IS, pending: EXTI_SR1.P6F, priority: ITC_SPR4.VECT14SPR
#define _EXTI7_VECTOR_                           15         ///< EXTI7 interrupt vector: enable: EXTI_CR2.P7IS, pending: EXTI_SR1.P7F, priority: ITC_SPR4.VECT15SPR
#define _TIM2_BREAK_VECTOR_                      19         ///< TIM2_BREAK interrupt vector: enable: TIM2_IER.BIE, pending: TIM2_SR1.BIF, priority: ITC_SPR5.VECT19SPR
#define _TIM2_TRIGGER_VECTOR_                    19         ///< TIM2_TRIGGER interrupt vector: enable: TIM2_IER.TIE, pending: TIM2_SR1.TIF, priority: ITC_SPR5.VECT19SPR
#define _TIM2_UPDATE_VECTOR_                     19         ///< TIM2_UPDATE interrupt vector: enable: TIM2_IER.UIE, pending: TIM2_SR1.UIF, priority: ITC_SPR5.VECT19SPR
#define _TIM2_CAPCOM_CC1IF_VECTOR_               20         ///< TIM2_CAPCOM_CC1IF interrupt vector: enable: TIM2_IER.CC1IE, pending: TIM2_SR1.CC1IF, priority: ITC_SPR6.VECT20SPR
#define _TIM2_CAPCOM_CC2IF_VECTOR_               20         ///< TIM2_CAPCOM_CC2IF interrupt vector: enable: TIM2_IER.CC2IE, pending: TIM2_SR1.CC2IF, priority: ITC_SPR6.VECT20SPR
#define _TIM3_BREAK_VECTOR_                      21         ///< TIM3_BREAK interrupt vector: enable: TIM3_IER.BIE, pending: TIM3_SR1.BIF, priority: ITC_SPR5.VECT21SPR
#define _TIM3_TRIGGER_VECTOR_                    21         ///< TIM3_TRIGGER interrupt vector: enable: TIM3_IER.TIE, pending: TIM3_SR1.TIF, priority: ITC_SPR5.VECT21SPR
#define _TIM3_UPDATE_VECTOR_                     21         ///< TIM3_UPDATE interrupt vector: enable: TIM3_IER.UIE, pending: TIM3_SR1.UIF, priority: ITC_SPR5.VECT21SPR
#define _TIM3_CAPCOM_CC1IF_VECTOR_               22         ///< TIM3_CAPCOM_CC1IF interrupt vector: enable: TIM3_IER.CC1IE, pending: TIM3_SR1.CC1IF, priority: ITC_SPR6.VECT22SPR
#define _TIM3_CAPCOM_CC2IF_VECTOR_               22         ///< TIM3_CAPCOM_CC2IF interrupt vector: enable: TIM3_IER.CC2IE, pending: TIM3_SR1.CC2IF, priority: ITC_SPR6.VECT22SPR
#define _TIM4_TRIGGER_VECTOR_                    25         ///< TIM4_TRIGGER interrupt vector: enable: TIM4_IER.TIE, pending: TIM4_SR1.TIF, priority: ITC_SPR7.VECT25SPR
#define _TIM4_UPDATE_VECTOR_                     25         ///< TIM4_UPDATE interrupt vector: enable: TIM4_IER.UIE, pending: TIM4_SR1.UIF, priority: ITC_SPR7.VECT25SPR
#define _SPI_MODF_VECTOR_                        26         ///< SPI_MODF interrupt vector: enable: SPI_ICR.ERRIE, pending: SPI_SR.MODF, priority: ITC_SPR7.VECT26SPR
#define _SPI_OVR_VECTOR_                         26         ///< SPI_OVR interrupt vector: enable: SPI_ICR.ERRIE, pending: SPI_SR.OVR, priority: ITC_SPR7.VECT26SPR
#define _SPI_RXNE_VECTOR_                        26         ///< SPI_RXNE interrupt vector: enable: SPI_ICR.RXIE, pending: SPI_SR.RXNE, priority: ITC_SPR7.VECT26SPR
#define _SPI_TXE_VECTOR_                         26         ///< SPI_TXE interrupt vector: enable: SPI_ICR.TXIE, pending: SPI_SR.TXE, priority: ITC_SPR7.VECT26SPR
#define _SPI_WKUP_VECTOR_                        26         ///< SPI_WKUP interrupt vector: enable: SPI_ICR.WKIE, pending: SPI_SR.WKUP, priority: ITC_SPR7.VECT26SPR
#define _USART_T_TC_VECTOR_                      27         ///< USART_T_TC interrupt vector: enable: USART_CR2.TCIEN, pending: USART_SR.TC, priority: ITC_SPR7.VECT27SPR
#define _USART_T_TXE_VECTOR_                     27         ///< USART_T_TXE interrupt vector: enable: USART_CR2.TIEN, pending: USART_SR.TXE, priority: ITC_SPR7.VECT27SPR
#define _USART_R_IDLE_VECTOR_                    28         ///< USART_R_IDLE interrupt vector: enable: USART_CR2.ILIEN, pending: USART_SR.IDLE, priority: ITC_SPR7.VECT28SPR
#define _USART_R_OR_VECTOR_                      28         ///< USART_R_OR interrupt vector: enable: USART_CR2.RIEN, pending: USART_SR.OR, priority: ITC_SPR7.VECT28SPR
#define _USART_R_PE_VECTOR_                      28         ///< USART_R_PE interrupt vector: enable: USART_CR1.PIEN, pending: USART_SR.PE, priority: ITC_SPR7.VECT28SPR
#define _USART_R_RXNE_VECTOR_                    28         ///< USART_R_RXNE interrupt vector: enable: USART_CR2.RIEN, pending: USART_SR.RXNE, priority: ITC_SPR7.VECT28SPR
#define _I2C_ADD10_VECTOR_                       29         ///< I2C_ADD10 interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.ADD10, priority: ITC_SPR8.VECT29SPR
#define _I2C_ADDR_VECTOR_                        29         ///< I2C_ADDR interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.ADDR, priority: ITC_SPR8.VECT29SPR
#define _I2C_AF_VECTOR_                          29         ///< I2C_AF interrupt vector: enable: I2C_ITR.ITERREN, pending: I2C_SR2.AF, priority: ITC_SPR8.VECT29SPR
#define _I2C_ARLO_VECTOR_                        29         ///< I2C_ARLO interrupt vector: enable: I2C_ITR.ITERREN, pending: I2C_SR2.ARLO, priority: ITC_SPR8.VECT29SPR
#define _I2C_BERR_VECTOR_                        29         ///< I2C_BERR interrupt vector: enable: I2C_ITR.ITERREN, pending: I2C_SR2.BERR, priority: ITC_SPR8.VECT29SPR
#define _I2C_BTF_VECTOR_                         29         ///< I2C_BTF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.BTF, priority: ITC_SPR8.VECT29SPR
#define _I2C_OVR_VECTOR_                         29         ///< I2C_OVR interrupt vector: enable: I2C_ITR.ITERREN, pending: I2C_SR2.OVR, priority: ITC_SPR8.VECT29SPR
#define _I2C_RXNE_VECTOR_                        29         ///< I2C_RXNE interrupt vector: enable: I2C_ITR.ITBUFEN, pending: I2C_SR1.RXNE, priority: ITC_SPR8.VECT29SPR
#define _I2C_SB_VECTOR_                          29         ///< I2C_SB interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.SB, priority: ITC_SPR8.VECT29SPR
#define _I2C_STOPF_VECTOR_                       29         ///< I2C_STOPF interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR1.STOPF, priority: ITC_SPR8.VECT29SPR
#define _I2C_TXE_VECTOR_                         29         ///< I2C_TXE interrupt vector: enable: I2C_ITR.ITBUFEN, pending: I2C_SR1.TXE, priority: ITC_SPR8.VECT29SPR
#define _I2C_WUFH_VECTOR_                        29         ///< I2C_WUFH interrupt vector: enable: I2C_ITR.ITEVTEN, pending: I2C_SR2.WUFH, priority: ITC_SPR8.VECT29SPR


/*-------------------------------------------------------------------------
  DEFINITION OF STM8 PERIPHERAL REGISTERS
-------------------------------------------------------------------------*/

//------------------------
// Module AWU
//------------------------

/** struct containing AWU module registers */
typedef struct {

  /** AWU control/status register (CSR at 0x50f0) */
  union {

    /// bytewise access to CSR
    uint8_t  byte;

    /// bitwise access to register CSR
    struct {
      BITS   MSR                 : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   AWUEN               : 1;      // bit 4
      BITS   AWUF                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CSR bitfield

    /// register _AWU_CSR reset value
    #define sfr_AWU_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSR;


  /** AWU asynchronous prescaler buffer register (APR at 0x50f1) */
  union {

    /// bytewise access to APR
    uint8_t  byte;

    /// bitwise access to register APR
    struct {
      BITS   APR                 : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // APR bitfield

    /// register _AWU_APR reset value
    #define sfr_AWU_APR_RESET_VALUE   ((uint8_t) 0x3F)

  } APR;


  /** AWU timebase selection register (TBR at 0x50f2) */
  union {

    /// bytewise access to TBR
    uint8_t  byte;

    /// bitwise access to register TBR
    struct {
      BITS   AWUTB               : 6;      // bits 0-5
      BITS                       : 2;      // 2 bits
    };  // TBR bitfield

    /// register _AWU_TBR reset value
    #define sfr_AWU_TBR_RESET_VALUE   ((uint8_t) 0x00)

  } TBR;

} AWU_t;

/// access to AWU SFR registers
#define sfr_AWU   (*((AWU_t*) 0x50f0))


//------------------------
// Module BEEP
//------------------------

/** struct containing BEEP module registers */
typedef struct {

  /** BEEP control/status register (CSR at 0x50f3) */
  union {

    /// bytewise access to CSR
    uint8_t  byte;

    /// bitwise access to register CSR
    struct {
      BITS   BEEPDIV             : 5;      // bits 0-4
      BITS   BEEPEN              : 1;      // bit 5
      BITS   BEEPSEL             : 2;      // bits 6-7
    };  // CSR bitfield

    /// register _BEEP_CSR reset value
    #define sfr_BEEP_CSR_RESET_VALUE   ((uint8_t) 0x1F)

  } CSR;

} BEEP_t;

/// access to BEEP SFR registers
#define sfr_BEEP   (*((BEEP_t*) 0x50f3))


//------------------------
// Module CLK
//------------------------

/** struct containing CLK module registers */
typedef struct {

  /** Clock divider register (CKDIVR at 0x50c0) */
  union {

    /// bytewise access to CKDIVR
    uint8_t  byte;

    /// bitwise access to register CKDIVR
    struct {
      BITS   HSIDIV              : 2;      // bits 0-1
      BITS                       : 6;      // 6 bits
    };  // CKDIVR bitfield

    /// register _CLK_CKDIVR reset value
    #define sfr_CLK_CKDIVR_RESET_VALUE   ((uint8_t) 0x03)

  } CKDIVR;


  /// Reserved register (2B)
  uint8_t     Reserved_1[2];


  /** Peripheral clock gating register 1 (PCKENR1 at 0x50c3) */
  union {

    /// bytewise access to PCKENR1
    uint8_t  byte;

    /// bitwise access to register PCKENR1
    struct {
      BITS   PCKENR1             : 8;      // bits 0-7
    };  // PCKENR1 bitfield

    /// register _CLK_PCKENR1 reset value
    #define sfr_CLK_PCKENR1_RESET_VALUE   ((uint8_t) 0x00)

  } PCKENR1;


  /** Peripheral clock gating register 2 (PCKENR2 at 0x50c4) */
  union {

    /// bytewise access to PCKENR2
    uint8_t  byte;

    /// bitwise access to register PCKENR2
    struct {
      BITS   PCKENR2             : 8;      // bits 0-7
    };  // PCKENR2 bitfield

    /// register _CLK_PCKENR2 reset value
    #define sfr_CLK_PCKENR2_RESET_VALUE   ((uint8_t) 0x00)

  } PCKENR2;


  /** Configurable clock control register (CCOR at 0x50c5) */
  union {

    /// bytewise access to CCOR
    uint8_t  byte;

    /// bitwise access to register CCOR
    struct {
      BITS   CC0EN               : 1;      // bit 0
      BITS   CCOSEL              : 2;      // bits 1-2
      BITS                       : 5;      // 5 bits
    };  // CCOR bitfield

    /// register _CLK_CCOR reset value
    #define sfr_CLK_CCOR_RESET_VALUE   ((uint8_t) 0x00)

  } CCOR;

} CLK_t;

/// access to CLK SFR registers
#define sfr_CLK   (*((CLK_t*) 0x50c0))


//------------------------
// Module CPU
//------------------------

/** struct containing CPU module registers */
typedef struct {

  /** Accumulator (A at 0x7f00) */
  union {

    /// bytewise access to A
    uint8_t  byte;

    /// skip bitwise access to register A

    /// register _CPU_A reset value
    #define sfr_CPU_A_RESET_VALUE   ((uint8_t) 0x00)

  } A;


  /** Program counter extended (PCE at 0x7f01) */
  union {

    /// bytewise access to PCE
    uint8_t  byte;

    /// skip bitwise access to register PCE

    /// register _CPU_PCE reset value
    #define sfr_CPU_PCE_RESET_VALUE   ((uint8_t) 0x00)

  } PCE;


  /** Program counter high (PCH at 0x7f02) */
  union {

    /// bytewise access to PCH
    uint8_t  byte;

    /// skip bitwise access to register PCH

    /// register _CPU_PCH reset value
    #define sfr_CPU_PCH_RESET_VALUE   ((uint8_t) 0x80)

  } PCH;


  /** Program counter low (PCL at 0x7f03) */
  union {

    /// bytewise access to PCL
    uint8_t  byte;

    /// skip bitwise access to register PCL

    /// register _CPU_PCL reset value
    #define sfr_CPU_PCL_RESET_VALUE   ((uint8_t) 0x00)

  } PCL;


  /** X index register high (XH at 0x7f04) */
  union {

    /// bytewise access to XH
    uint8_t  byte;

    /// skip bitwise access to register XH

    /// register _CPU_XH reset value
    #define sfr_CPU_XH_RESET_VALUE   ((uint8_t) 0x00)

  } XH;


  /** X index register low (XL at 0x7f05) */
  union {

    /// bytewise access to XL
    uint8_t  byte;

    /// skip bitwise access to register XL

    /// register _CPU_XL reset value
    #define sfr_CPU_XL_RESET_VALUE   ((uint8_t) 0x00)

  } XL;


  /** Y index register high (YH at 0x7f06) */
  union {

    /// bytewise access to YH
    uint8_t  byte;

    /// skip bitwise access to register YH

    /// register _CPU_YH reset value
    #define sfr_CPU_YH_RESET_VALUE   ((uint8_t) 0x00)

  } YH;


  /** Y index register low (YL at 0x7f07) */
  union {

    /// bytewise access to YL
    uint8_t  byte;

    /// skip bitwise access to register YL

    /// register _CPU_YL reset value
    #define sfr_CPU_YL_RESET_VALUE   ((uint8_t) 0x00)

  } YL;


  /** Stack pointer high (SPH at 0x7f08) */
  union {

    /// bytewise access to SPH
    uint8_t  byte;

    /// skip bitwise access to register SPH

    /// register _CPU_SPH reset value
    #define sfr_CPU_SPH_RESET_VALUE   ((uint8_t) 0x05)

  } SPH;


  /** Stack pointer low (SPL at 0x7f09) */
  union {

    /// bytewise access to SPL
    uint8_t  byte;

    /// skip bitwise access to register SPL

    /// register _CPU_SPL reset value
    #define sfr_CPU_SPL_RESET_VALUE   ((uint8_t) 0xFF)

  } SPL;


  /** Condition code register (CCR at 0x7f0a) */
  union {

    /// bytewise access to CCR
    uint8_t  byte;

    /// bitwise access to register CCR
    struct {
      BITS   C                   : 1;      // bit 0
      BITS   Z                   : 1;      // bit 1
      BITS   N                   : 1;      // bit 2
      BITS   I0                  : 1;      // bit 3
      BITS   H                   : 1;      // bit 4
      BITS   I1                  : 1;      // bit 5
      BITS                       : 1;      // 1 bit
      BITS   V                   : 1;      // bit 7
    };  // CCR bitfield

    /// register _CPU_CCR reset value
    #define sfr_CPU_CCR_RESET_VALUE   ((uint8_t) 0x28)

  } CCR;


  /// Reserved register (85B)
  uint8_t     Reserved_1[85];


  /** Global configuration register (CFG_GCR at 0x7f60) */
  union {

    /// bytewise access to CFG_GCR
    uint8_t  byte;

    /// bitwise access to register CFG_GCR
    struct {
      BITS   SWD                 : 1;      // bit 0
      BITS   AL                  : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CFG_GCR bitfield

    /// register _CPU_CFG_GCR reset value
    #define sfr_CPU_CFG_GCR_RESET_VALUE   ((uint8_t) 0x00)

  } CFG_GCR;

} CPU_t;

/// access to CPU SFR registers
#define sfr_CPU   (*((CPU_t*) 0x7f00))


//------------------------
// Module DM
//------------------------

/** struct containing DM module registers */
typedef struct {

  /** Breakpoint 1 register extended byte (BK1RE at 0x7f90) */
  union {

    /// bytewise access to BK1RE
    uint8_t  byte;

    /// skip bitwise access to register BK1RE

    /// register _DM_BK1RE reset value
    #define sfr_DM_BK1RE_RESET_VALUE   ((uint8_t) 0xFF)

  } BK1RE;


  /** Breakpoint 1 register high byte (BK1RH at 0x7f91) */
  union {

    /// bytewise access to BK1RH
    uint8_t  byte;

    /// skip bitwise access to register BK1RH

    /// register _DM_BK1RH reset value
    #define sfr_DM_BK1RH_RESET_VALUE   ((uint8_t) 0xFF)

  } BK1RH;


  /** Breakpoint 1 register low byte (BK1RL at 0x7f92) */
  union {

    /// bytewise access to BK1RL
    uint8_t  byte;

    /// skip bitwise access to register BK1RL

    /// register _DM_BK1RL reset value
    #define sfr_DM_BK1RL_RESET_VALUE   ((uint8_t) 0xFF)

  } BK1RL;


  /** Breakpoint 2 register extended byte (BK2RE at 0x7f93) */
  union {

    /// bytewise access to BK2RE
    uint8_t  byte;

    /// skip bitwise access to register BK2RE

    /// register _DM_BK2RE reset value
    #define sfr_DM_BK2RE_RESET_VALUE   ((uint8_t) 0xFF)

  } BK2RE;


  /** Breakpoint 2 register high byte (BK2RH at 0x7f94) */
  union {

    /// bytewise access to BK2RH
    uint8_t  byte;

    /// skip bitwise access to register BK2RH

    /// register _DM_BK2RH reset value
    #define sfr_DM_BK2RH_RESET_VALUE   ((uint8_t) 0xFF)

  } BK2RH;


  /** Breakpoint 2 register low byte (BK2RL at 0x7f95) */
  union {

    /// bytewise access to BK2RL
    uint8_t  byte;

    /// skip bitwise access to register BK2RL

    /// register _DM_BK2RL reset value
    #define sfr_DM_BK2RL_RESET_VALUE   ((uint8_t) 0xFF)

  } BK2RL;


  /** Debug module control register 1 (CR1 at 0x7f96) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// skip bitwise access to register CR1

    /// register _DM_CR1 reset value
    #define sfr_DM_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Debug module control register 2 (CR2 at 0x7f97) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// skip bitwise access to register CR2

    /// register _DM_CR2 reset value
    #define sfr_DM_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** Debug module control/status register 1 (CSR1 at 0x7f98) */
  union {

    /// bytewise access to CSR1
    uint8_t  byte;

    /// skip bitwise access to register CSR1

    /// register _DM_CSR1 reset value
    #define sfr_DM_CSR1_RESET_VALUE   ((uint8_t) 0x10)

  } CSR1;


  /** Debug module control/status register 2 (CSR2 at 0x7f99) */
  union {

    /// bytewise access to CSR2
    uint8_t  byte;

    /// skip bitwise access to register CSR2

    /// register _DM_CSR2 reset value
    #define sfr_DM_CSR2_RESET_VALUE   ((uint8_t) 0x00)

  } CSR2;


  /** Enable function register (ENFCTR at 0x7f9a) */
  union {

    /// bytewise access to ENFCTR
    uint8_t  byte;

    /// skip bitwise access to register ENFCTR

    /// register _DM_ENFCTR reset value
    #define sfr_DM_ENFCTR_RESET_VALUE   ((uint8_t) 0xFF)

  } ENFCTR;

} DM_t;

/// access to DM SFR registers
#define sfr_DM   (*((DM_t*) 0x7f90))


//------------------------
// Module FLASH
//------------------------

/** struct containing FLASH module registers */
typedef struct {

  /** Flash control register 1 (CR1 at 0x5050) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   FIX                 : 1;      // bit 0
      BITS   IE                  : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // CR1 bitfield

    /// register _FLASH_CR1 reset value
    #define sfr_FLASH_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Flash control register 2 (CR2 at 0x5051) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   PRG                 : 1;      // bit 0
      BITS                       : 3;      // 3 bits
      BITS   FPRG                : 1;      // bit 4
      BITS   ERASE               : 1;      // bit 5
      BITS   WPRG                : 1;      // bit 6
      BITS   OPT                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _FLASH_CR2 reset value
    #define sfr_FLASH_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** Flash Program memory unprotection register (PUKR at 0x5052) */
  union {

    /// bytewise access to PUKR
    uint8_t  byte;

    /// bitwise access to register PUKR
    struct {
      BITS   MASS_PRG            : 8;      // bits 0-7
    };  // PUKR bitfield

    /// register _FLASH_PUKR reset value
    #define sfr_FLASH_PUKR_RESET_VALUE   ((uint8_t) 0x00)

  } PUKR;


  /** Data EEPROM unprotection register (DUKR at 0x5053) */
  union {

    /// bytewise access to DUKR
    uint8_t  byte;

    /// bitwise access to register DUKR
    struct {
      BITS   MASS_DATA           : 8;      // bits 0-7
    };  // DUKR bitfield

    /// register _FLASH_DUKR reset value
    #define sfr_FLASH_DUKR_RESET_VALUE   ((uint8_t) 0x00)

  } DUKR;


  /** Flash in-application programming status register (IAPSR at 0x5054) */
  union {

    /// bytewise access to IAPSR
    uint8_t  byte;

    /// bitwise access to register IAPSR
    struct {
      BITS   WR_PG_DIS           : 1;      // bit 0
      BITS   PUL                 : 1;      // bit 1
      BITS   EOP                 : 1;      // bit 2
      BITS   DUL                 : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // IAPSR bitfield

    /// register _FLASH_IAPSR reset value
    #define sfr_FLASH_IAPSR_RESET_VALUE   ((uint8_t) 0x00)

  } IAPSR;

} FLASH_t;

/// access to FLASH SFR registers
#define sfr_FLASH   (*((FLASH_t*) 0x5050))


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


  /** I2C own address register low (OAR1L at 0x5213) */
  union {

    /// bytewise access to OAR1L
    uint8_t  byte;

    /// bitwise access to register OAR1L
    struct {
      BITS   ADD                 : 8;      // bits 0-7
    };  // OAR1L bitfield

    /// register _I2C_OAR1L reset value
    #define sfr_I2C_OAR1L_RESET_VALUE   ((uint8_t) 0x00)

  } OAR1L;


  /** I2C own address register high (OAR1H at 0x5214) */
  union {

    /// bytewise access to OAR1H
    uint8_t  byte;

    /// bitwise access to register OAR1H
    struct {
      BITS                       : 1;      // 1 bit
      BITS   ADD                 : 2;      // bits 1-2
      BITS                       : 3;      // 3 bits
      BITS   ADDCONF             : 1;      // bit 6
      BITS   ADDMODE             : 1;      // bit 7
    };  // OAR1H bitfield

    /// register _I2C_OAR1H reset value
    #define sfr_I2C_OAR1H_RESET_VALUE   ((uint8_t) 0x00)

  } OAR1H;


  /** I2C own address register 2 (OAR2 at 0x5215) */
  union {

    /// bytewise access to OAR2
    uint8_t  byte;

    /// bitwise access to register OAR2
    struct {
      BITS   ENDUAL              : 1;      // bit 0
      BITS   ADD2                : 7;      // bits 1-7
    };  // OAR2 bitfield

    /// register _I2C_OAR2 reset value
    #define sfr_I2C_OAR2_RESET_VALUE   ((uint8_t) 0x00)

  } OAR2;


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


  /** I2C Clock control register low (CCRL at 0x521b) */
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


  /** I2C Clock control register high (CCRH at 0x521c) */
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
    #define sfr_I2C_TRISER_RESET_VALUE   ((uint8_t) 0x00)

  } TRISER;

} I2C_t;

/// access to I2C SFR registers
#define sfr_I2C   (*((I2C_t*) 0x5210))


//------------------------
// Module ITC_EXTI
//------------------------

/** struct containing ITC_EXTI module registers */
typedef struct {

  /** External interrupt control register 1 (CR1 at 0x50a0) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   P0IS                : 2;      // bits 0-1
      BITS   P1IS                : 2;      // bits 2-3
      BITS   P2IS                : 2;      // bits 4-5
      BITS   P3IS                : 2;      // bits 6-7
    };  // CR1 bitfield

    /// register _ITC_EXTI_CR1 reset value
    #define sfr_ITC_EXTI_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** External interrupt control register 2 (CR2 at 0x50a1) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   P4IS                : 2;      // bits 0-1
      BITS   P5IS                : 2;      // bits 2-3
      BITS   P6IS                : 2;      // bits 4-5
      BITS   P7IS                : 2;      // bits 6-7
    };  // CR2 bitfield

    /// register _ITC_EXTI_CR2 reset value
    #define sfr_ITC_EXTI_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** External interrupt control register 3 (CR3 at 0x50a2) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   PDIS                : 2;      // bits 0-1
      BITS   PBIS                : 2;      // bits 2-3
      BITS                       : 4;      // 4 bits
    };  // CR3 bitfield

    /// register _ITC_EXTI_CR3 reset value
    #define sfr_ITC_EXTI_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** External interrupt status register 1 (SR1 at 0x50a3) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   P0F                 : 1;      // bit 0
      BITS   P1F                 : 1;      // bit 1
      BITS   P2F                 : 1;      // bit 2
      BITS   P3F                 : 1;      // bit 3
      BITS   P4F                 : 1;      // bit 4
      BITS   P5F                 : 1;      // bit 5
      BITS   P6F                 : 1;      // bit 6
      BITS   P7F                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _ITC_EXTI_SR1 reset value
    #define sfr_ITC_EXTI_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** External interrupt status register 2 (SR2 at 0x50a4) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS   PBF                 : 1;      // bit 0
      BITS   PDF                 : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // SR2 bitfield

    /// register _ITC_EXTI_SR2 reset value
    #define sfr_ITC_EXTI_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** External interrupt port select register (CONF at 0x50a5) */
  union {

    /// bytewise access to CONF
    uint8_t  byte;

    /// bitwise access to register CONF
    struct {
      BITS   PBLIS               : 1;      // bit 0
      BITS   PBHIS               : 1;      // bit 1
      BITS   PDLIS               : 1;      // bit 2
      BITS   PDHIS               : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // CONF bitfield

    /// register _ITC_EXTI_CONF reset value
    #define sfr_ITC_EXTI_CONF_RESET_VALUE   ((uint8_t) 0x00)

  } CONF;

} ITC_EXTI_t;

/// access to ITC_EXTI SFR registers
#define sfr_ITC_EXTI   (*((ITC_EXTI_t*) 0x50a0))


//------------------------
// Module ITC_SPR
//------------------------

/** struct containing ITC_SPR module registers */
typedef struct {

  /** Interrupt Software priority register 1 (SPR1 at 0x7f70) */
  union {

    /// bytewise access to SPR1
    uint8_t  byte;

    /// bitwise access to register SPR1
    struct {
      BITS                       : 2;      // 2 bits
      BITS   VECT1SPR            : 2;      // bits 2-3
      BITS                       : 4;      // 4 bits
    };  // SPR1 bitfield

    /// register _ITC_SPR_SPR1 reset value
    #define sfr_ITC_SPR_SPR1_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR1;


  /** Interrupt Software priority register 2 (SPR2 at 0x7f71) */
  union {

    /// bytewise access to SPR2
    uint8_t  byte;

    /// bitwise access to register SPR2
    struct {
      BITS   VECT4SPR            : 2;      // bits 0-1
      BITS                       : 2;      // 2 bits
      BITS   VECT6SPR            : 2;      // bits 4-5
      BITS   VECT7SPR            : 2;      // bits 6-7
    };  // SPR2 bitfield

    /// register _ITC_SPR_SPR2 reset value
    #define sfr_ITC_SPR_SPR2_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR2;


  /** Interrupt Software priority register 3 (SPR3 at 0x7f72) */
  union {

    /// bytewise access to SPR3
    uint8_t  byte;

    /// bitwise access to register SPR3
    struct {
      BITS   VECT8SPR            : 2;      // bits 0-1
      BITS   VECT9SPR            : 2;      // bits 2-3
      BITS   VECT10SPR           : 2;      // bits 4-5
      BITS   VECT11SPR           : 2;      // bits 6-7
    };  // SPR3 bitfield

    /// register _ITC_SPR_SPR3 reset value
    #define sfr_ITC_SPR_SPR3_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR3;


  /** Interrupt Software priority register 4 (SPR4 at 0x7f73) */
  union {

    /// bytewise access to SPR4
    uint8_t  byte;

    /// bitwise access to register SPR4
    struct {
      BITS   VECT12SPR           : 2;      // bits 0-1
      BITS   VECT13SPR           : 2;      // bits 2-3
      BITS   VECT14SPR           : 2;      // bits 4-5
      BITS   VECT15SPR           : 2;      // bits 6-7
    };  // SPR4 bitfield

    /// register _ITC_SPR_SPR4 reset value
    #define sfr_ITC_SPR_SPR4_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR4;


  /** Interrupt Software priority register 5 (SPR5 at 0x7f74) */
  union {

    /// bytewise access to SPR5
    uint8_t  byte;

    /// bitwise access to register SPR5
    struct {
      BITS                       : 4;      // 4 bits
      BITS   VECT18SPR           : 2;      // bits 4-5
      BITS   VECT19SPR           : 2;      // bits 6-7
    };  // SPR5 bitfield

    /// register _ITC_SPR_SPR5 reset value
    #define sfr_ITC_SPR_SPR5_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR5;


  /** Interrupt Software priority register 6 (SPR6 at 0x7f75) */
  union {

    /// bytewise access to SPR6
    uint8_t  byte;

    /// bitwise access to register SPR6
    struct {
      BITS   VECT20SPR           : 2;      // bits 0-1
      BITS   VECT21SPR           : 2;      // bits 2-3
      BITS   VECT22SPR           : 2;      // bits 4-5
      BITS                       : 2;      // 2 bits
    };  // SPR6 bitfield

    /// register _ITC_SPR_SPR6 reset value
    #define sfr_ITC_SPR_SPR6_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR6;


  /** Interrupt Software priority register 7 (SPR7 at 0x7f76) */
  union {

    /// bytewise access to SPR7
    uint8_t  byte;

    /// bitwise access to register SPR7
    struct {
      BITS                       : 2;      // 2 bits
      BITS   VECT25SPR           : 2;      // bits 2-3
      BITS   VECT26SPR           : 2;      // bits 4-5
      BITS   VECT27SPR           : 2;      // bits 6-7
    };  // SPR7 bitfield

    /// register _ITC_SPR_SPR7 reset value
    #define sfr_ITC_SPR_SPR7_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR7;


  /** Interrupt Software priority register 8 (SPR8 at 0x7f77) */
  union {

    /// bytewise access to SPR8
    uint8_t  byte;

    /// bitwise access to register SPR8
    struct {
      BITS   VECT28SPR           : 2;      // bits 0-1
      BITS   VECT29SPR           : 2;      // bits 2-3
      BITS                       : 4;      // 4 bits
    };  // SPR8 bitfield

    /// register _ITC_SPR_SPR8 reset value
    #define sfr_ITC_SPR_SPR8_RESET_VALUE   ((uint8_t) 0xFF)

  } SPR8;

} ITC_SPR_t;

/// access to ITC_SPR SFR registers
#define sfr_ITC_SPR   (*((ITC_SPR_t*) 0x7f70))


//------------------------
// Module OPT
//------------------------

/** struct containing OPT module registers */
typedef struct {

  /** Read-out protection (ROP) (OPT0 at 0x4800) */
  union {

    /// bytewise access to OPT0
    uint8_t  byte;

    /// skip bitwise access to register OPT0

    /// register _OPT_OPT0 reset value
    #define sfr_OPT_OPT0_RESET_VALUE   ((uint8_t) 0xAA)

  } OPT0;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** User boot code (UBC) (OPT1 at 0x4802) */
  union {

    /// bytewise access to OPT1
    uint8_t  byte;

    /// skip bitwise access to register OPT1

    /// register _OPT_OPT1 reset value
    #define sfr_OPT_OPT1_RESET_VALUE   ((uint8_t) 0x00)

  } OPT1;


  /** DATASIZE (OPT2 at 0x4803) */
  union {

    /// bytewise access to OPT2
    uint8_t  byte;

    /// skip bitwise access to register OPT2

    /// register _OPT_OPT2 reset value
    #define sfr_OPT_OPT2_RESET_VALUE   ((uint8_t) 0x00)

  } OPT2;


  /// Reserved register (3B)
  uint8_t     Reserved_2[3];


  /** PCODESIZE (OPT3 at 0x4807) */
  union {

    /// bytewise access to OPT3
    uint8_t  byte;

    /// skip bitwise access to register OPT3

    /// register _OPT_OPT3 reset value
    #define sfr_OPT_OPT3_RESET_VALUE   ((uint8_t) 0x00)

  } OPT3;


  /** Watchdog option (OPT4 at 0x4808) */
  union {

    /// bytewise access to OPT4
    uint8_t  byte;

    /// skip bitwise access to register OPT4

    /// register _OPT_OPT4 reset value
    #define sfr_OPT_OPT4_RESET_VALUE   ((uint8_t) 0x00)

  } OPT4;

} OPT_t;

/// access to OPT SFR registers
#define sfr_OPT   (*((OPT_t*) 0x4800))


//------------------------
// Module PORT
//------------------------

/** struct containing PORTA module registers */
typedef struct {

  /** Port A data output latch register (ODR at 0x5000) */
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


  /** Port A input pin value register (IDR at 0x5001) */
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


  /** Port A data direction register (DDR at 0x5002) */
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


  /** Port A control register 1 (CR1 at 0x5003) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   C10                 : 1;      // bit 0
      BITS   C11                 : 1;      // bit 1
      BITS   C12                 : 1;      // bit 2
      BITS   C13                 : 1;      // bit 3
      BITS   C14                 : 1;      // bit 4
      BITS   C15                 : 1;      // bit 5
      BITS   C16                 : 1;      // bit 6
      BITS   C17                 : 1;      // bit 7
    };  // CR1 bitfield

    /// register _PORT_CR1 reset value
    #define sfr_PORT_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** Port A control register 2 (CR2 at 0x5004) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   C20                 : 1;      // bit 0
      BITS   C21                 : 1;      // bit 1
      BITS   C22                 : 1;      // bit 2
      BITS   C23                 : 1;      // bit 3
      BITS   C24                 : 1;      // bit 4
      BITS   C25                 : 1;      // bit 5
      BITS   C26                 : 1;      // bit 6
      BITS   C27                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _PORT_CR2 reset value
    #define sfr_PORT_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;

} PORT_t;

/// access to PORTA SFR registers
#define sfr_PORTA   (*((PORT_t*) 0x5000))


/// access to PORTB SFR registers
#define sfr_PORTB   (*((PORT_t*) 0x5005))


/// access to PORTD SFR registers
#define sfr_PORTD   (*((PORT_t*) 0x500f))


//------------------------
// Module PXS
//------------------------

/** struct containing PXS module registers */
typedef struct {

  /** ProxSense control register 1 (CR1 at 0x5300) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS                       : 5;      // 5 bits
      BITS   LOW_POWER           : 1;      // bit 5
      BITS   START               : 1;      // bit 6
      BITS   PXSEN               : 1;      // bit 7
    };  // CR1 bitfield

    /// register _PXS_CR1 reset value
    #define sfr_PXS_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** ProxSense control register 2 (CR2 at 0x5301) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   SYNCEDGE            : 1;      // bit 0
      BITS   SYNCEN              : 1;      // bit 1
      BITS   RXCOUPLING          : 1;      // bit 2
      BITS   RXGROUP             : 1;      // bit 3
      BITS                       : 1;      // 1 bit
      BITS   NOISEDETEN          : 1;      // bit 5
      BITS   FCCITEN             : 1;      // bit 6
      BITS   EOCITEN             : 1;      // bit 7
    };  // CR2 bitfield

    /// register _PXS_CR2 reset value
    #define sfr_PXS_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** ProxSense control register 3 (CR3 at 0x5302) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   VTHR                : 4;      // bits 0-3
      BITS   BIAS                : 2;      // bits 4-5
      BITS   STAB                : 2;      // bits 6-7
    };  // CR3 bitfield

    /// register _PXS_CR3 reset value
    #define sfr_PXS_CR3_RESET_VALUE   ((uint8_t) 0x04)

  } CR3;


  /// Reserved register (1B)
  uint8_t     Reserved_1[1];


  /** ProxSense interrupt and status register (ISR at 0x5304) */
  union {

    /// bytewise access to ISR
    uint8_t  byte;

    /// bitwise access to register ISR
    struct {
      BITS                       : 2;      // 2 bits
      BITS   SYNC_OVRF           : 1;      // bit 2
      BITS   SYNCPF              : 1;      // bit 3
      BITS   CIPF                : 1;      // bit 4
      BITS   NOISEDETF           : 1;      // bit 5
      BITS   FCCF                : 1;      // bit 6
      BITS   EOCF                : 1;      // bit 7
    };  // ISR bitfield

    /// register _PXS_ISR reset value
    #define sfr_PXS_ISR_RESET_VALUE   ((uint8_t) 0x00)

  } ISR;


  /// Reserved register (1B)
  uint8_t     Reserved_2[1];


  /** ProxSense clock control register 1 (CKCR1 at 0x5306) */
  union {

    /// bytewise access to CKCR1
    uint8_t  byte;

    /// bitwise access to register CKCR1
    struct {
      BITS   PASSLEN             : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   PREUPLENSC          : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CKCR1 bitfield

    /// register _PXS_CKCR1 reset value
    #define sfr_PXS_CKCR1_RESET_VALUE   ((uint8_t) 0x30)

  } CKCR1;


  /** ProxSense clock control register 2 (CKCR2 at 0x5307) */
  union {

    /// bytewise access to CKCR2
    uint8_t  byte;

    /// bitwise access to register CKCR2
    struct {
      BITS   INCPHASE            : 1;      // bit 0
      BITS   ANADEAD             : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   PRESC               : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CKCR2 bitfield

    /// register _PXS_CKCR2 reset value
    #define sfr_PXS_CKCR2_RESET_VALUE   ((uint8_t) 0x11)

  } CKCR2;


  /** ProxSense receiver enable register high (RXENRH at 0x5308) */
  union {

    /// bytewise access to RXENRH
    uint8_t  byte;

    /// bitwise access to register RXENRH
    struct {
      BITS   RXEN8               : 1;      // bit 0
      BITS   RXEN9               : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // RXENRH bitfield

    /// register _PXS_RXENRH reset value
    #define sfr_PXS_RXENRH_RESET_VALUE   ((uint8_t) 0x00)

  } RXENRH;


  /** ProxSense receiver enable register low (RXENRL at 0x5309) */
  union {

    /// bytewise access to RXENRL
    uint8_t  byte;

    /// bitwise access to register RXENRL
    struct {
      BITS   RXEN0               : 1;      // bit 0
      BITS   RXEN1               : 1;      // bit 1
      BITS   RXEN2               : 1;      // bit 2
      BITS   RXEN3               : 1;      // bit 3
      BITS   RXEN4               : 1;      // bit 4
      BITS   RXEN5               : 1;      // bit 5
      BITS   RXEN6               : 1;      // bit 6
      BITS   RXEN7               : 1;      // bit 7
    };  // RXENRL bitfield

    /// register _PXS_RXENRL reset value
    #define sfr_PXS_RXENRL_RESET_VALUE   ((uint8_t) 0x00)

  } RXENRL;


  /** ProxSense receiver control register 1 high (RXCR1H at 0x530a) */
  union {

    /// bytewise access to RXCR1H
    uint8_t  byte;

    /// bitwise access to register RXCR1H
    struct {
      BITS   RXCR1_8             : 1;      // bit 0
      BITS   RXCR1_9             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // RXCR1H bitfield

    /// register _PXS_RXCR1H reset value
    #define sfr_PXS_RXCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } RXCR1H;


  /** ProxSense receiver control register 1 low (RXCR1L at 0x530b) */
  union {

    /// bytewise access to RXCR1L
    uint8_t  byte;

    /// bitwise access to register RXCR1L
    struct {
      BITS   RXCR1_0             : 1;      // bit 0
      BITS   RXCR1_1             : 1;      // bit 1
      BITS   RXCR1_2             : 1;      // bit 2
      BITS   RXCR1_3             : 1;      // bit 3
      BITS   RXCR1_4             : 1;      // bit 4
      BITS   RXCR1_5             : 1;      // bit 5
      BITS   RXCR1_6             : 1;      // bit 6
      BITS   RXCR1_7             : 1;      // bit 7
    };  // RXCR1L bitfield

    /// register _PXS_RXCR1L reset value
    #define sfr_PXS_RXCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } RXCR1L;


  /** ProxSense receiver control register 2 high (RXCR2H at 0x530c) */
  union {

    /// bytewise access to RXCR2H
    uint8_t  byte;

    /// bitwise access to register RXCR2H
    struct {
      BITS   RXCR2_8             : 1;      // bit 0
      BITS   RXCR2_9             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // RXCR2H bitfield

    /// register _PXS_RXCR2H reset value
    #define sfr_PXS_RXCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } RXCR2H;


  /** ProxSense receiver control register 2 low (RXCR2L at 0x530d) */
  union {

    /// bytewise access to RXCR2L
    uint8_t  byte;

    /// bitwise access to register RXCR2L
    struct {
      BITS   RXCR2_0             : 1;      // bit 0
      BITS   RXCR2_1             : 1;      // bit 1
      BITS   RXCR2_2             : 1;      // bit 2
      BITS   RXCR2_3             : 1;      // bit 3
      BITS   RXCR2_4             : 1;      // bit 4
      BITS   RXCR2_5             : 1;      // bit 5
      BITS   RXCR2_6             : 1;      // bit 6
      BITS   RXCR2_7             : 1;      // bit 7
    };  // RXCR2L bitfield

    /// register _PXS_RXCR2L reset value
    #define sfr_PXS_RXCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } RXCR2L;


  /** ProxSense receiver control register 3 high (RXCR3H at 0x530e) */
  union {

    /// bytewise access to RXCR3H
    uint8_t  byte;

    /// bitwise access to register RXCR3H
    struct {
      BITS   RXCR3_8             : 1;      // bit 0
      BITS   RXCR3_9             : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // RXCR3H bitfield

    /// register _PXS_RXCR3H reset value
    #define sfr_PXS_RXCR3H_RESET_VALUE   ((uint8_t) 0x00)

  } RXCR3H;


  /** ProxSense receiver control register 3 low (RXCR3L at 0x530f) */
  union {

    /// bytewise access to RXCR3L
    uint8_t  byte;

    /// bitwise access to register RXCR3L
    struct {
      BITS   RXCR3_0             : 1;      // bit 0
      BITS   RXCR3_1             : 1;      // bit 1
      BITS   RXCR3_2             : 1;      // bit 2
      BITS   RXCR3_3             : 1;      // bit 3
      BITS   RXCR3_4             : 1;      // bit 4
      BITS   RXCR3_5             : 1;      // bit 5
      BITS   RXCR3_6             : 1;      // bit 6
      BITS   RXCR3_7             : 1;      // bit 7
    };  // RXCR3L bitfield

    /// register _PXS_RXCR3L reset value
    #define sfr_PXS_RXCR3L_RESET_VALUE   ((uint8_t) 0x00)

  } RXCR3L;


  /// Reserved register (2B)
  uint8_t     Reserved_3[2];


  /** ProxSense receiver inactive state register high (RXINSRH at 0x5312) */
  union {

    /// bytewise access to RXINSRH
    uint8_t  byte;

    /// bitwise access to register RXINSRH
    struct {
      BITS   RXINS8              : 1;      // bit 0
      BITS   RXINS9              : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // RXINSRH bitfield

    /// register _PXS_RXINSRH reset value
    #define sfr_PXS_RXINSRH_RESET_VALUE   ((uint8_t) 0x00)

  } RXINSRH;


  /** ProxSense receiver inactive state register low (RXINSRL at 0x5313) */
  union {

    /// bytewise access to RXINSRL
    uint8_t  byte;

    /// bitwise access to register RXINSRL
    struct {
      BITS   RXINS0              : 1;      // bit 0
      BITS   RXINS1              : 1;      // bit 1
      BITS   RXINS2              : 1;      // bit 2
      BITS   RXINS3              : 1;      // bit 3
      BITS   RXINS4              : 1;      // bit 4
      BITS   RXINS5              : 1;      // bit 5
      BITS   RXINS6              : 1;      // bit 6
      BITS   RXINS7              : 1;      // bit 7
    };  // RXINSRL bitfield

    /// register _PXS_RXINSRL reset value
    #define sfr_PXS_RXINSRL_RESET_VALUE   ((uint8_t) 0x00)

  } RXINSRL;


  /// Reserved register (2B)
  uint8_t     Reserved_4[2];


  /** ProxSense transmit enable register high (TXENRH at 0x5316) */
  union {

    /// bytewise access to TXENRH
    uint8_t  byte;

    /// bitwise access to register TXENRH
    struct {
      BITS   TXEN8               : 1;      // bit 0
      BITS   TXEN9               : 1;      // bit 1
      BITS   TXEN10              : 1;      // bit 2
      BITS   TXEN11              : 1;      // bit 3
      BITS   TXEN12              : 1;      // bit 4
      BITS   TXEN13              : 1;      // bit 5
      BITS   TXEN14              : 1;      // bit 6
      BITS   TXEN15              : 1;      // bit 7
    };  // TXENRH bitfield

    /// register _PXS_TXENRH reset value
    #define sfr_PXS_TXENRH_RESET_VALUE   ((uint8_t) 0x00)

  } TXENRH;


  /** ProxSense transmit enable register low (TXENRL at 0x5317) */
  union {

    /// bytewise access to TXENRL
    uint8_t  byte;

    /// bitwise access to register TXENRL
    struct {
      BITS   TXEN0               : 1;      // bit 0
      BITS   TXEN1               : 1;      // bit 1
      BITS   TXEN2               : 1;      // bit 2
      BITS   TXEN3               : 1;      // bit 3
      BITS   TXEN4               : 1;      // bit 4
      BITS   TXEN5               : 1;      // bit 5
      BITS   TXEN6               : 1;      // bit 6
      BITS   TXEN7               : 1;      // bit 7
    };  // TXENRL bitfield

    /// register _PXS_TXENRL reset value
    #define sfr_PXS_TXENRL_RESET_VALUE   ((uint8_t) 0x00)

  } TXENRL;


  /// Reserved register (2B)
  uint8_t     Reserved_5[2];


  /** ProxSense maximum counter value register high (MAXRH at 0x531a) */
  union {

    /// bytewise access to MAXRH
    uint8_t  byte;

    /// bitwise access to register MAXRH
    struct {
      BITS   MAX                 : 8;      // bits 0-7
    };  // MAXRH bitfield

    /// register _PXS_MAXRH reset value
    #define sfr_PXS_MAXRH_RESET_VALUE   ((uint8_t) 0xFF)

  } MAXRH;


  /** ProxSense maximum counter value register low (MAXRL at 0x531b) */
  union {

    /// bytewise access to MAXRL
    uint8_t  byte;

    /// bitwise access to register MAXRL
    struct {
      BITS   MAX                 : 8;      // bits 0-7
    };  // MAXRL bitfield

    /// register _PXS_MAXRL reset value
    #define sfr_PXS_MAXRL_RESET_VALUE   ((uint8_t) 0xFF)

  } MAXRL;


  /** ProxSense maximum counter enable register high (MAXENRH at 0x531c) */
  union {

    /// bytewise access to MAXENRH
    uint8_t  byte;

    /// bitwise access to register MAXENRH
    struct {
      BITS   MAXEN8              : 1;      // bit 0
      BITS   MAXEN9              : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // MAXENRH bitfield

    /// register _PXS_MAXENRH reset value
    #define sfr_PXS_MAXENRH_RESET_VALUE   ((uint8_t) 0x00)

  } MAXENRH;


  /** ProxSense maximum counter enable register low (MAXENRL at 0x531d) */
  union {

    /// bytewise access to MAXENRL
    uint8_t  byte;

    /// bitwise access to register MAXENRL
    struct {
      BITS   MAXEN0              : 1;      // bit 0
      BITS   MAXEN1              : 1;      // bit 1
      BITS   MAXEN2              : 1;      // bit 2
      BITS   MAXEN3              : 1;      // bit 3
      BITS   MAXEN4              : 1;      // bit 4
      BITS   MAXEN5              : 1;      // bit 5
      BITS   MAXEN6              : 1;      // bit 6
      BITS   MAXEN7              : 1;      // bit 7
    };  // MAXENRL bitfield

    /// register _PXS_MAXENRL reset value
    #define sfr_PXS_MAXENRL_RESET_VALUE   ((uint8_t) 0x00)

  } MAXENRL;


  /** ProxSense receiver status register high (RXSRH at 0x531e) */
  union {

    /// bytewise access to RXSRH
    uint8_t  byte;

    /// bitwise access to register RXSRH
    struct {
      BITS   VALID8              : 1;      // bit 0
      BITS   VALID9              : 1;      // bit 1
      BITS                       : 6;      // 6 bits
    };  // RXSRH bitfield

    /// register _PXS_RXSRH reset value
    #define sfr_PXS_RXSRH_RESET_VALUE   ((uint8_t) 0x00)

  } RXSRH;


  /** ProxSense receiver status register low (RXSRL at 0x531f) */
  union {

    /// bytewise access to RXSRL
    uint8_t  byte;

    /// bitwise access to register RXSRL
    struct {
      BITS   VALID0              : 1;      // bit 0
      BITS   VALID1              : 1;      // bit 1
      BITS   VALID2              : 1;      // bit 2
      BITS   VALID3              : 1;      // bit 3
      BITS   VALID4              : 1;      // bit 4
      BITS   VALID5              : 1;      // bit 5
      BITS   VALID6              : 1;      // bit 6
      BITS   VALID7              : 1;      // bit 7
    };  // RXSRL bitfield

    /// register _PXS_RXSRL reset value
    #define sfr_PXS_RXSRL_RESET_VALUE   ((uint8_t) 0x00)

  } RXSRL;


  /** ProxSense counter register receiver channel high (RX0CNTRH at 0x5320) */
  union {

    /// bytewise access to RX0CNTRH
    uint8_t  byte;

    /// bitwise access to register RX0CNTRH
    struct {
      BITS   RX0CNT              : 8;      // bits 0-7
    };  // RX0CNTRH bitfield

    /// register _PXS_RX0CNTRH reset value
    #define sfr_PXS_RX0CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX0CNTRH;


  /** ProxSense counter register receiver channel low (RX0CNTRL at 0x5321) */
  union {

    /// bytewise access to RX0CNTRL
    uint8_t  byte;

    /// bitwise access to register RX0CNTRL
    struct {
      BITS   RX0CNT              : 8;      // bits 0-7
    };  // RX0CNTRL bitfield

    /// register _PXS_RX0CNTRL reset value
    #define sfr_PXS_RX0CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX0CNTRL;


  /** ProxSense counter register receiver channel high (RX1CNTRH at 0x5322) */
  union {

    /// bytewise access to RX1CNTRH
    uint8_t  byte;

    /// bitwise access to register RX1CNTRH
    struct {
      BITS   RX1CNT              : 8;      // bits 0-7
    };  // RX1CNTRH bitfield

    /// register _PXS_RX1CNTRH reset value
    #define sfr_PXS_RX1CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX1CNTRH;


  /** ProxSense counter register receiver channel low (RX1CNTRL at 0x5323) */
  union {

    /// bytewise access to RX1CNTRL
    uint8_t  byte;

    /// bitwise access to register RX1CNTRL
    struct {
      BITS   RX1CNT              : 8;      // bits 0-7
    };  // RX1CNTRL bitfield

    /// register _PXS_RX1CNTRL reset value
    #define sfr_PXS_RX1CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX1CNTRL;


  /** ProxSense counter register receiver channel high (RX2CNTRH at 0x5324) */
  union {

    /// bytewise access to RX2CNTRH
    uint8_t  byte;

    /// bitwise access to register RX2CNTRH
    struct {
      BITS   RX2CNT              : 8;      // bits 0-7
    };  // RX2CNTRH bitfield

    /// register _PXS_RX2CNTRH reset value
    #define sfr_PXS_RX2CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX2CNTRH;


  /** ProxSense counter register receiver channel low (RX2CNTRL at 0x5325) */
  union {

    /// bytewise access to RX2CNTRL
    uint8_t  byte;

    /// bitwise access to register RX2CNTRL
    struct {
      BITS   RX2CNT              : 8;      // bits 0-7
    };  // RX2CNTRL bitfield

    /// register _PXS_RX2CNTRL reset value
    #define sfr_PXS_RX2CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX2CNTRL;


  /** ProxSense counter register receiver channel high (RX3CNTRH at 0x5326) */
  union {

    /// bytewise access to RX3CNTRH
    uint8_t  byte;

    /// bitwise access to register RX3CNTRH
    struct {
      BITS   RX3CNT              : 8;      // bits 0-7
    };  // RX3CNTRH bitfield

    /// register _PXS_RX3CNTRH reset value
    #define sfr_PXS_RX3CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX3CNTRH;


  /** ProxSense counter register receiver channel low (RX3CNTRL at 0x5327) */
  union {

    /// bytewise access to RX3CNTRL
    uint8_t  byte;

    /// bitwise access to register RX3CNTRL
    struct {
      BITS   RX3CNT              : 8;      // bits 0-7
    };  // RX3CNTRL bitfield

    /// register _PXS_RX3CNTRL reset value
    #define sfr_PXS_RX3CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX3CNTRL;


  /** ProxSense counter register receiver channel high (RX4CNTRH at 0x5328) */
  union {

    /// bytewise access to RX4CNTRH
    uint8_t  byte;

    /// bitwise access to register RX4CNTRH
    struct {
      BITS   RX4CNT              : 8;      // bits 0-7
    };  // RX4CNTRH bitfield

    /// register _PXS_RX4CNTRH reset value
    #define sfr_PXS_RX4CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX4CNTRH;


  /** ProxSense counter register receiver channel low (RX4CNTRL at 0x5329) */
  union {

    /// bytewise access to RX4CNTRL
    uint8_t  byte;

    /// bitwise access to register RX4CNTRL
    struct {
      BITS   RX4CNT              : 8;      // bits 0-7
    };  // RX4CNTRL bitfield

    /// register _PXS_RX4CNTRL reset value
    #define sfr_PXS_RX4CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX4CNTRL;


  /** ProxSense counter register receiver channel high (RX5CNTRH at 0x532a) */
  union {

    /// bytewise access to RX5CNTRH
    uint8_t  byte;

    /// bitwise access to register RX5CNTRH
    struct {
      BITS   RX5CNT              : 8;      // bits 0-7
    };  // RX5CNTRH bitfield

    /// register _PXS_RX5CNTRH reset value
    #define sfr_PXS_RX5CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX5CNTRH;


  /** ProxSense counter register receiver channel low (RX5CNTRL at 0x532b) */
  union {

    /// bytewise access to RX5CNTRL
    uint8_t  byte;

    /// bitwise access to register RX5CNTRL
    struct {
      BITS   RX5CNT              : 8;      // bits 0-7
    };  // RX5CNTRL bitfield

    /// register _PXS_RX5CNTRL reset value
    #define sfr_PXS_RX5CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX5CNTRL;


  /** ProxSense counter register receiver channel high (RX6CNTRH at 0x532c) */
  union {

    /// bytewise access to RX6CNTRH
    uint8_t  byte;

    /// bitwise access to register RX6CNTRH
    struct {
      BITS   RX6CNT              : 8;      // bits 0-7
    };  // RX6CNTRH bitfield

    /// register _PXS_RX6CNTRH reset value
    #define sfr_PXS_RX6CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX6CNTRH;


  /** ProxSense counter register receiver channel low (RX6CNTRL at 0x532d) */
  union {

    /// bytewise access to RX6CNTRL
    uint8_t  byte;

    /// bitwise access to register RX6CNTRL
    struct {
      BITS   RX6CNT              : 8;      // bits 0-7
    };  // RX6CNTRL bitfield

    /// register _PXS_RX6CNTRL reset value
    #define sfr_PXS_RX6CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX6CNTRL;


  /** ProxSense counter register receiver channel high (RX7CNTRH at 0x532e) */
  union {

    /// bytewise access to RX7CNTRH
    uint8_t  byte;

    /// bitwise access to register RX7CNTRH
    struct {
      BITS   RX7CNT              : 8;      // bits 0-7
    };  // RX7CNTRH bitfield

    /// register _PXS_RX7CNTRH reset value
    #define sfr_PXS_RX7CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX7CNTRH;


  /** ProxSense counter register receiver channel low (RX7CNTRL at 0x532f) */
  union {

    /// bytewise access to RX7CNTRL
    uint8_t  byte;

    /// bitwise access to register RX7CNTRL
    struct {
      BITS   RX7CNT              : 8;      // bits 0-7
    };  // RX7CNTRL bitfield

    /// register _PXS_RX7CNTRL reset value
    #define sfr_PXS_RX7CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX7CNTRL;


  /** ProxSense counter register receiver channel high (RX8CNTRH at 0x5330) */
  union {

    /// bytewise access to RX8CNTRH
    uint8_t  byte;

    /// bitwise access to register RX8CNTRH
    struct {
      BITS   RX8CNT              : 8;      // bits 0-7
    };  // RX8CNTRH bitfield

    /// register _PXS_RX8CNTRH reset value
    #define sfr_PXS_RX8CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX8CNTRH;


  /** ProxSense counter register receiver channel low (RX8CNTRL at 0x5331) */
  union {

    /// bytewise access to RX8CNTRL
    uint8_t  byte;

    /// bitwise access to register RX8CNTRL
    struct {
      BITS   RX8CNT              : 8;      // bits 0-7
    };  // RX8CNTRL bitfield

    /// register _PXS_RX8CNTRL reset value
    #define sfr_PXS_RX8CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX8CNTRL;


  /** ProxSense counter register receiver channel high (RX9CNTRH at 0x5332) */
  union {

    /// bytewise access to RX9CNTRH
    uint8_t  byte;

    /// bitwise access to register RX9CNTRH
    struct {
      BITS   RX9CNT              : 8;      // bits 0-7
    };  // RX9CNTRH bitfield

    /// register _PXS_RX9CNTRH reset value
    #define sfr_PXS_RX9CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } RX9CNTRH;


  /** ProxSense counter register receiver channel low (RX9CNTRL at 0x5333) */
  union {

    /// bytewise access to RX9CNTRL
    uint8_t  byte;

    /// bitwise access to register RX9CNTRL
    struct {
      BITS   RX9CNT              : 8;      // bits 0-7
    };  // RX9CNTRL bitfield

    /// register _PXS_RX9CNTRL reset value
    #define sfr_PXS_RX9CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } RX9CNTRL;


  /// Reserved register (12B)
  uint8_t     Reserved_6[12];


  /** ProxSense receiver sampling capacitor selection register (RX0CSSELR at 0x5340) */
  union {

    /// bytewise access to RX0CSSELR
    uint8_t  byte;

    /// bitwise access to register RX0CSSELR
    struct {
      BITS   RX0CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX0CSSELR bitfield

    /// register _PXS_RX0CSSELR reset value
    #define sfr_PXS_RX0CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX0CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX1CSSELR at 0x5341) */
  union {

    /// bytewise access to RX1CSSELR
    uint8_t  byte;

    /// bitwise access to register RX1CSSELR
    struct {
      BITS   RX1CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX1CSSELR bitfield

    /// register _PXS_RX1CSSELR reset value
    #define sfr_PXS_RX1CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX1CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX2CSSELR at 0x5342) */
  union {

    /// bytewise access to RX2CSSELR
    uint8_t  byte;

    /// bitwise access to register RX2CSSELR
    struct {
      BITS   RX2CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX2CSSELR bitfield

    /// register _PXS_RX2CSSELR reset value
    #define sfr_PXS_RX2CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX2CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX3CSSELR at 0x5343) */
  union {

    /// bytewise access to RX3CSSELR
    uint8_t  byte;

    /// bitwise access to register RX3CSSELR
    struct {
      BITS   RX3CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX3CSSELR bitfield

    /// register _PXS_RX3CSSELR reset value
    #define sfr_PXS_RX3CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX3CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX4CSSELR at 0x5344) */
  union {

    /// bytewise access to RX4CSSELR
    uint8_t  byte;

    /// bitwise access to register RX4CSSELR
    struct {
      BITS   RX4CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX4CSSELR bitfield

    /// register _PXS_RX4CSSELR reset value
    #define sfr_PXS_RX4CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX4CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX5CSSELR at 0x5345) */
  union {

    /// bytewise access to RX5CSSELR
    uint8_t  byte;

    /// bitwise access to register RX5CSSELR
    struct {
      BITS   RX5CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX5CSSELR bitfield

    /// register _PXS_RX5CSSELR reset value
    #define sfr_PXS_RX5CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX5CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX6CSSELR at 0x5346) */
  union {

    /// bytewise access to RX6CSSELR
    uint8_t  byte;

    /// bitwise access to register RX6CSSELR
    struct {
      BITS   RX6CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX6CSSELR bitfield

    /// register _PXS_RX6CSSELR reset value
    #define sfr_PXS_RX6CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX6CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX7CSSELR at 0x5347) */
  union {

    /// bytewise access to RX7CSSELR
    uint8_t  byte;

    /// bitwise access to register RX7CSSELR
    struct {
      BITS   RX7CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX7CSSELR bitfield

    /// register _PXS_RX7CSSELR reset value
    #define sfr_PXS_RX7CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX7CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX8CSSELR at 0x5348) */
  union {

    /// bytewise access to RX8CSSELR
    uint8_t  byte;

    /// bitwise access to register RX8CSSELR
    struct {
      BITS   RX8CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX8CSSELR bitfield

    /// register _PXS_RX8CSSELR reset value
    #define sfr_PXS_RX8CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX8CSSELR;


  /** ProxSense receiver sampling capacitor selection register (RX9CSSELR at 0x5349) */
  union {

    /// bytewise access to RX9CSSELR
    uint8_t  byte;

    /// bitwise access to register RX9CSSELR
    struct {
      BITS   RX9CSSEL            : 5;      // bits 0-4
      BITS                       : 3;      // 3 bits
    };  // RX9CSSELR bitfield

    /// register _PXS_RX9CSSELR reset value
    #define sfr_PXS_RX9CSSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX9CSSELR;


  /// Reserved register (6B)
  uint8_t     Reserved_7[6];


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX0EPCCSELR at 0x5350) */
  union {

    /// bytewise access to RX0EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX0EPCCSELR
    struct {
      BITS   RX0EPCC             : 8;      // bits 0-7
    };  // RX0EPCCSELR bitfield

    /// register _PXS_RX0EPCCSELR reset value
    #define sfr_PXS_RX0EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX0EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX1EPCCSELR at 0x5351) */
  union {

    /// bytewise access to RX1EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX1EPCCSELR
    struct {
      BITS   RX1EPCC             : 8;      // bits 0-7
    };  // RX1EPCCSELR bitfield

    /// register _PXS_RX1EPCCSELR reset value
    #define sfr_PXS_RX1EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX1EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX2EPCCSELR at 0x5352) */
  union {

    /// bytewise access to RX2EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX2EPCCSELR
    struct {
      BITS   RX2EPCC             : 8;      // bits 0-7
    };  // RX2EPCCSELR bitfield

    /// register _PXS_RX2EPCCSELR reset value
    #define sfr_PXS_RX2EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX2EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX3EPCCSELR at 0x5353) */
  union {

    /// bytewise access to RX3EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX3EPCCSELR
    struct {
      BITS   RX3EPCC             : 8;      // bits 0-7
    };  // RX3EPCCSELR bitfield

    /// register _PXS_RX3EPCCSELR reset value
    #define sfr_PXS_RX3EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX3EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX4EPCCSELR at 0x5354) */
  union {

    /// bytewise access to RX4EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX4EPCCSELR
    struct {
      BITS   RX4EPCC             : 8;      // bits 0-7
    };  // RX4EPCCSELR bitfield

    /// register _PXS_RX4EPCCSELR reset value
    #define sfr_PXS_RX4EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX4EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX5EPCCSELR at 0x5355) */
  union {

    /// bytewise access to RX5EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX5EPCCSELR
    struct {
      BITS   RX5EPCC             : 8;      // bits 0-7
    };  // RX5EPCCSELR bitfield

    /// register _PXS_RX5EPCCSELR reset value
    #define sfr_PXS_RX5EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX5EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX6EPCCSELR at 0x5356) */
  union {

    /// bytewise access to RX6EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX6EPCCSELR
    struct {
      BITS   RX6EPCC             : 8;      // bits 0-7
    };  // RX6EPCCSELR bitfield

    /// register _PXS_RX6EPCCSELR reset value
    #define sfr_PXS_RX6EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX6EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX7EPCCSELR at 0x5357) */
  union {

    /// bytewise access to RX7EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX7EPCCSELR
    struct {
      BITS   RX7EPCC             : 8;      // bits 0-7
    };  // RX7EPCCSELR bitfield

    /// register _PXS_RX7EPCCSELR reset value
    #define sfr_PXS_RX7EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX7EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX8EPCCSELR at 0x5358) */
  union {

    /// bytewise access to RX8EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX8EPCCSELR
    struct {
      BITS   RX8EPCC             : 8;      // bits 0-7
    };  // RX8EPCCSELR bitfield

    /// register _PXS_RX8EPCCSELR reset value
    #define sfr_PXS_RX8EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX8EPCCSELR;


  /** ProxSense receiver electrode parasitic compensation capacitor selection register (RX9EPCCSELR at 0x5359) */
  union {

    /// bytewise access to RX9EPCCSELR
    uint8_t  byte;

    /// bitwise access to register RX9EPCCSELR
    struct {
      BITS   RX9EPCC             : 8;      // bits 0-7
    };  // RX9EPCCSELR bitfield

    /// register _PXS_RX9EPCCSELR reset value
    #define sfr_PXS_RX9EPCCSELR_RESET_VALUE   ((uint8_t) 0x00)

  } RX9EPCCSELR;

} PXS_t;

/// access to PXS SFR registers
#define sfr_PXS   (*((PXS_t*) 0x5300))


//------------------------
// Module RST
//------------------------

/** struct containing RST module registers */
typedef struct {

  /** Reset control register (CR at 0x50b0) */
  union {

    /// bytewise access to CR
    uint8_t  byte;

    /// bitwise access to register CR
    struct {
      BITS   RSTPIN_KEY          : 8;      // bits 0-7
    };  // CR bitfield

    /// register _RST_CR reset value
    #define sfr_RST_CR_RESET_VALUE   ((uint8_t) 0x00)

  } CR;


  /** Reset status register (SR at 0x50b1) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   PORF                : 1;      // bit 0
      BITS   IWDGF               : 1;      // bit 1
      BITS   ILLOPF              : 1;      // bit 2
      BITS   SWIMF               : 1;      // bit 3
      BITS                       : 4;      // 4 bits
    };  // SR bitfield

    /// register _RST_SR reset value
    #define sfr_RST_SR_RESET_VALUE   ((uint8_t) 0x01)

  } SR;

} RST_t;

/// access to RST SFR registers
#define sfr_RST   (*((RST_t*) 0x50b0))


//------------------------
// Module SPI
//------------------------

/** struct containing SPI module registers */
typedef struct {

  /** SPI control register 1 (CR1 at 0x5200) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CPHA                : 1;      // bit 0
      BITS   CPOL                : 1;      // bit 1
      BITS   MSTR                : 1;      // bit 2
      BITS   BR                  : 3;      // bits 3-5
      BITS   SPE                 : 1;      // bit 6
      BITS   LSBFIRST            : 1;      // bit 7
    };  // CR1 bitfield

    /// register _SPI_CR1 reset value
    #define sfr_SPI_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** SPI control register 2 (CR2 at 0x5201) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   SSI                 : 1;      // bit 0
      BITS   SSM                 : 1;      // bit 1
      BITS   RXOnly              : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   BD0E                : 1;      // bit 6
      BITS   BDM                 : 1;      // bit 7
    };  // CR2 bitfield

    /// register _SPI_CR2 reset value
    #define sfr_SPI_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** SPI interrupt control register (ICR at 0x5202) */
  union {

    /// bytewise access to ICR
    uint8_t  byte;

    /// bitwise access to register ICR
    struct {
      BITS                       : 4;      // 4 bits
      BITS   WKIE                : 1;      // bit 4
      BITS   ERRIE               : 1;      // bit 5
      BITS   RXIE                : 1;      // bit 6
      BITS   TXIE                : 1;      // bit 7
    };  // ICR bitfield

    /// register _SPI_ICR reset value
    #define sfr_SPI_ICR_RESET_VALUE   ((uint8_t) 0x00)

  } ICR;


  /** SPI status register (SR at 0x5203) */
  union {

    /// bytewise access to SR
    uint8_t  byte;

    /// bitwise access to register SR
    struct {
      BITS   RXNE                : 1;      // bit 0
      BITS   TXE                 : 1;      // bit 1
      BITS                       : 1;      // 1 bit
      BITS   WKUP                : 1;      // bit 3
      BITS                       : 1;      // 1 bit
      BITS   MODF                : 1;      // bit 5
      BITS   OVR                 : 1;      // bit 6
      BITS   BSY                 : 1;      // bit 7
    };  // SR bitfield

    /// register _SPI_SR reset value
    #define sfr_SPI_SR_RESET_VALUE   ((uint8_t) 0x02)

  } SR;


  /** SPI data register (DR at 0x5204) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _SPI_DR reset value
    #define sfr_SPI_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;

} SPI_t;

/// access to SPI SFR registers
#define sfr_SPI   (*((SPI_t*) 0x5200))


//------------------------
// Module SWIM
//------------------------

/** struct containing SWIM module registers */
typedef struct {

  /** SWIM control status register (CSR at 0x7f80) */
  union {

    /// bytewise access to CSR
    uint8_t  byte;

    /// skip bitwise access to register CSR

    /// register _SWIM_CSR reset value
    #define sfr_SWIM_CSR_RESET_VALUE   ((uint8_t) 0x00)

  } CSR;

} SWIM_t;

/// access to SWIM SFR registers
#define sfr_SWIM   (*((SWIM_t*) 0x7f80))


//------------------------
// Module TIM2
//------------------------

/** struct containing TIM2 module registers */
typedef struct {

  /** TIM2 Control register 1 (CR1 at 0x5250) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS   DIR                 : 1;      // bit 4
      BITS   CMS                 : 2;      // bits 5-6
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM2_CR1 reset value
    #define sfr_TIM2_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM2 Control register 2 (CR2 at 0x5251) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 4;      // 4 bits
      BITS   MMS                 : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _TIM2_CR2 reset value
    #define sfr_TIM2_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM2 Slave Mode Control register (SMCR at 0x5252) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM2_SMCR reset value
    #define sfr_TIM2_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM2 external trigger register (ETR at 0x5253) */
  union {

    /// bytewise access to ETR
    uint8_t  byte;

    /// bitwise access to register ETR
    struct {
      BITS   ETF                 : 4;      // bits 0-3
      BITS   ETPS                : 2;      // bits 4-5
      BITS   ECE                 : 1;      // bit 6
      BITS   ETP                 : 1;      // bit 7
    };  // ETR bitfield

    /// register _TIM2_ETR reset value
    #define sfr_TIM2_ETR_RESET_VALUE   ((uint8_t) 0x00)

  } ETR;


  /** TIM2 Interrupt enable register (IER at 0x5254) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIE                 : 1;      // bit 6
      BITS   BIE                 : 1;      // bit 7
    };  // IER bitfield

    /// register _TIM2_IER reset value
    #define sfr_TIM2_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM2 Status register 1 (SR1 at 0x5255) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIF                 : 1;      // bit 6
      BITS   BIF                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _TIM2_SR1 reset value
    #define sfr_TIM2_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM2 Status register 2 (SR2 at 0x5256) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // SR2 bitfield

    /// register _TIM2_SR2 reset value
    #define sfr_TIM2_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM2 Event Generation register (EGR at 0x5257) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TG                  : 1;      // bit 6
      BITS   BG                  : 1;      // bit 7
    };  // EGR bitfield

    /// register _TIM2_EGR reset value
    #define sfr_TIM2_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM2 Capture/Compare mode register 1 (CCMR1_OUT_CCMR1_IN at 0x5258) */
  union {

    /// bytewise access to CCMR1_OUT_CCMR1_IN
    uint8_t  byte;

    /// bitwise access to register CCMR1_OUT
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS   OC1FE               : 1;      // bit 2
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1_OUT bitfield

    /// register _TIM2_CCMR1_OUT reset value
    #define sfr_TIM2_CCMR1_OUT_RESET_VALUE   ((uint8_t) 0x00)


    /// bitwise access to register CCMR1_IN
    struct {
      BITS                       : 2;      // CC1S defined above 
      BITS   IC1PSC              : 2;      // bits 2-3
      BITS   IC1F                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1_IN bitfield

    /// register _TIM2_CCMR1_IN reset value
    #define sfr_TIM2_CCMR1_IN_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1_OUT_CCMR1_IN;


  /** TIM2 Capture/Compare mode register 2 (CCMR2_OUT_CCMR2_IN at 0x5259) */
  union {

    /// bytewise access to CCMR2_OUT_CCMR2_IN
    uint8_t  byte;

    /// bitwise access to register CCMR2_OUT
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS   OC2FE               : 1;      // bit 2
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2_OUT bitfield

    /// register _TIM2_CCMR2_OUT reset value
    #define sfr_TIM2_CCMR2_OUT_RESET_VALUE   ((uint8_t) 0x00)


    /// bitwise access to register CCMR2_IN
    struct {
      BITS                       : 2;      // CC2S defined above 
      BITS   IC2PSC              : 2;      // bits 2-3
      BITS   IC2F                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2_IN bitfield

    /// register _TIM2_CCMR2_IN reset value
    #define sfr_TIM2_CCMR2_IN_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2_OUT_CCMR2_IN;


  /** TIM2 Capture/Compare enable register 1 (CCER1 at 0x525a) */
  union {

    /// bytewise access to CCER1
    uint8_t  byte;

    /// bitwise access to register CCER1
    struct {
      BITS   CC1E                : 1;      // bit 0
      BITS   CC1P                : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   CC2E                : 1;      // bit 4
      BITS   CC2P                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CCER1 bitfield

    /// register _TIM2_CCER1 reset value
    #define sfr_TIM2_CCER1_RESET_VALUE   ((uint8_t) 0x00)

  } CCER1;


  /** TIM2 Counter high (CNTRH at 0x525b) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRH bitfield

    /// register _TIM2_CNTRH reset value
    #define sfr_TIM2_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM2 Counter low (CNTRL at 0x525c) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRL bitfield

    /// register _TIM2_CNTRL reset value
    #define sfr_TIM2_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM2 Prescaler register (PSCR at 0x525d) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // PSCR bitfield

    /// register _TIM2_PSCR reset value
    #define sfr_TIM2_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM2 Auto-reload register high (ARRH at 0x525e) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRH bitfield

    /// register _TIM2_ARRH reset value
    #define sfr_TIM2_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM2 Auto-reload register low (ARRL at 0x525f) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRL bitfield

    /// register _TIM2_ARRL reset value
    #define sfr_TIM2_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM2 Capture/Compare register 1 high (CCR1H at 0x5260) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR                 : 8;      // bits 0-7
    };  // CCR1H bitfield

    /// register _TIM2_CCR1H reset value
    #define sfr_TIM2_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM2 Capture/Compare register 1 low (CCR1L at 0x5261) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR                 : 8;      // bits 0-7
    };  // CCR1L bitfield

    /// register _TIM2_CCR1L reset value
    #define sfr_TIM2_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM2 Capture/Compare register 2 high (CCR2H at 0x5262) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2H bitfield

    /// register _TIM2_CCR2H reset value
    #define sfr_TIM2_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM2 Capture/Compare register 2 low (CCR2L at 0x5263) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2L bitfield

    /// register _TIM2_CCR2L reset value
    #define sfr_TIM2_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM2 Break register (BKR at 0x5264) */
  union {

    /// bytewise access to BKR
    uint8_t  byte;

    /// bitwise access to register BKR
    struct {
      BITS   LOCK                : 2;      // bits 0-1
      BITS   OSSI                : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   BKE                 : 1;      // bit 4
      BITS   BKP                 : 1;      // bit 5
      BITS   AOE                 : 1;      // bit 6
      BITS   MOE                 : 1;      // bit 7
    };  // BKR bitfield

    /// register _TIM2_BKR reset value
    #define sfr_TIM2_BKR_RESET_VALUE   ((uint8_t) 0x00)

  } BKR;


  /** TIM2 Output idle state register (OISR at 0x5265) */
  union {

    /// bytewise access to OISR
    uint8_t  byte;

    /// bitwise access to register OISR
    struct {
      BITS   OIS1                : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   PIS2                : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // OISR bitfield

    /// register _TIM2_OISR reset value
    #define sfr_TIM2_OISR_RESET_VALUE   ((uint8_t) 0x00)

  } OISR;

} TIM2_t;

/// access to TIM2 SFR registers
#define sfr_TIM2   (*((TIM2_t*) 0x5250))


//------------------------
// Module TIM3
//------------------------

/** struct containing TIM3 module registers */
typedef struct {

  /** TIM3 Control register 1 (CR1 at 0x5280) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS   DIR                 : 1;      // bit 4
      BITS   CMS                 : 2;      // bits 5-6
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM3_CR1 reset value
    #define sfr_TIM3_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM3 Control register 2 (CR2 at 0x5281) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 4;      // 4 bits
      BITS   MMS                 : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _TIM3_CR2 reset value
    #define sfr_TIM3_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM3 Slave Mode Control register (SMCR at 0x5282) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM3_SMCR reset value
    #define sfr_TIM3_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM3 external trigger register (ETR at 0x5283) */
  union {

    /// bytewise access to ETR
    uint8_t  byte;

    /// bitwise access to register ETR
    struct {
      BITS   ETF                 : 4;      // bits 0-3
      BITS   ETPS                : 2;      // bits 4-5
      BITS   ECE                 : 1;      // bit 6
      BITS   ETP                 : 1;      // bit 7
    };  // ETR bitfield

    /// register _TIM3_ETR reset value
    #define sfr_TIM3_ETR_RESET_VALUE   ((uint8_t) 0x00)

  } ETR;


  /** TIM3 Interrupt enable register (IER at 0x5284) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS   CC1IE               : 1;      // bit 1
      BITS   CC2IE               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIE                 : 1;      // bit 6
      BITS   BIE                 : 1;      // bit 7
    };  // IER bitfield

    /// register _TIM3_IER reset value
    #define sfr_TIM3_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM3 Status register 1 (SR1 at 0x5285) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS   CC1IF               : 1;      // bit 1
      BITS   CC2IF               : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TIF                 : 1;      // bit 6
      BITS   BIF                 : 1;      // bit 7
    };  // SR1 bitfield

    /// register _TIM3_SR1 reset value
    #define sfr_TIM3_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM3 Status register 2 (SR2 at 0x5286) */
  union {

    /// bytewise access to SR2
    uint8_t  byte;

    /// bitwise access to register SR2
    struct {
      BITS                       : 1;      // 1 bit
      BITS   CC1OF               : 1;      // bit 1
      BITS   CC2OF               : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // SR2 bitfield

    /// register _TIM3_SR2 reset value
    #define sfr_TIM3_SR2_RESET_VALUE   ((uint8_t) 0x00)

  } SR2;


  /** TIM3 Event Generation register (EGR at 0x5287) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS   CC1G                : 1;      // bit 1
      BITS   CC2G                : 1;      // bit 2
      BITS                       : 3;      // 3 bits
      BITS   TG                  : 1;      // bit 6
      BITS   BG                  : 1;      // bit 7
    };  // EGR bitfield

    /// register _TIM3_EGR reset value
    #define sfr_TIM3_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM3 Capture/Compare mode register 1 (CCMR1_OUT_CCMR1_IN at 0x5288) */
  union {

    /// bytewise access to CCMR1_OUT_CCMR1_IN
    uint8_t  byte;

    /// bitwise access to register CCMR1_OUT
    struct {
      BITS   CC1S                : 2;      // bits 0-1
      BITS   OC1FE               : 1;      // bit 2
      BITS   OC1PE               : 1;      // bit 3
      BITS   OC1M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1_OUT bitfield

    /// register _TIM3_CCMR1_OUT reset value
    #define sfr_TIM3_CCMR1_OUT_RESET_VALUE   ((uint8_t) 0x00)


    /// bitwise access to register CCMR1_IN
    struct {
      BITS                       : 2;      // CC1S defined above 
      BITS   IC1PSC              : 2;      // bits 2-3
      BITS   IC1F                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR1_IN bitfield

    /// register _TIM3_CCMR1_IN reset value
    #define sfr_TIM3_CCMR1_IN_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR1_OUT_CCMR1_IN;


  /** TIM3 Capture/Compare mode register 2 (CCMR2_OUT_CCMR2_IN at 0x5289) */
  union {

    /// bytewise access to CCMR2_OUT_CCMR2_IN
    uint8_t  byte;

    /// bitwise access to register CCMR2_OUT
    struct {
      BITS   CC2S                : 2;      // bits 0-1
      BITS   OC2FE               : 1;      // bit 2
      BITS   OC2PE               : 1;      // bit 3
      BITS   OC2M                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2_OUT bitfield

    /// register _TIM3_CCMR2_OUT reset value
    #define sfr_TIM3_CCMR2_OUT_RESET_VALUE   ((uint8_t) 0x00)


    /// bitwise access to register CCMR2_IN
    struct {
      BITS                       : 2;      // CC2S defined above 
      BITS   IC2PSC              : 2;      // bits 2-3
      BITS   IC2F                : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CCMR2_IN bitfield

    /// register _TIM3_CCMR2_IN reset value
    #define sfr_TIM3_CCMR2_IN_RESET_VALUE   ((uint8_t) 0x00)

  } CCMR2_OUT_CCMR2_IN;


  /** TIM3 Capture/Compare enable register 1 (CCER1 at 0x528a) */
  union {

    /// bytewise access to CCER1
    uint8_t  byte;

    /// bitwise access to register CCER1
    struct {
      BITS   CC1E                : 1;      // bit 0
      BITS   CC1P                : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   CC2E                : 1;      // bit 4
      BITS   CC2P                : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CCER1 bitfield

    /// register _TIM3_CCER1 reset value
    #define sfr_TIM3_CCER1_RESET_VALUE   ((uint8_t) 0x00)

  } CCER1;


  /** TIM3 Counter high (CNTRH at 0x528b) */
  union {

    /// bytewise access to CNTRH
    uint8_t  byte;

    /// bitwise access to register CNTRH
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRH bitfield

    /// register _TIM3_CNTRH reset value
    #define sfr_TIM3_CNTRH_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRH;


  /** TIM3 Counter low (CNTRL at 0x528c) */
  union {

    /// bytewise access to CNTRL
    uint8_t  byte;

    /// bitwise access to register CNTRL
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTRL bitfield

    /// register _TIM3_CNTRL reset value
    #define sfr_TIM3_CNTRL_RESET_VALUE   ((uint8_t) 0x00)

  } CNTRL;


  /** TIM3 Prescaler register (PSCR at 0x528d) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 3;      // bits 0-2
      BITS                       : 5;      // 5 bits
    };  // PSCR bitfield

    /// register _TIM3_PSCR reset value
    #define sfr_TIM3_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM3 Auto-reload register high (ARRH at 0x528e) */
  union {

    /// bytewise access to ARRH
    uint8_t  byte;

    /// bitwise access to register ARRH
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRH bitfield

    /// register _TIM3_ARRH reset value
    #define sfr_TIM3_ARRH_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRH;


  /** TIM3 Auto-reload register low (ARRL at 0x528f) */
  union {

    /// bytewise access to ARRL
    uint8_t  byte;

    /// bitwise access to register ARRL
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARRL bitfield

    /// register _TIM3_ARRL reset value
    #define sfr_TIM3_ARRL_RESET_VALUE   ((uint8_t) 0xFF)

  } ARRL;


  /** TIM3 Capture/Compare register 1 high (CCR1H at 0x5290) */
  union {

    /// bytewise access to CCR1H
    uint8_t  byte;

    /// bitwise access to register CCR1H
    struct {
      BITS   CCR                 : 8;      // bits 0-7
    };  // CCR1H bitfield

    /// register _TIM3_CCR1H reset value
    #define sfr_TIM3_CCR1H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1H;


  /** TIM3 Capture/Compare register 1 low (CCR1L at 0x5291) */
  union {

    /// bytewise access to CCR1L
    uint8_t  byte;

    /// bitwise access to register CCR1L
    struct {
      BITS   CCR                 : 8;      // bits 0-7
    };  // CCR1L bitfield

    /// register _TIM3_CCR1L reset value
    #define sfr_TIM3_CCR1L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR1L;


  /** TIM3 Capture/Compare register 2 high (CCR2H at 0x5292) */
  union {

    /// bytewise access to CCR2H
    uint8_t  byte;

    /// bitwise access to register CCR2H
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2H bitfield

    /// register _TIM3_CCR2H reset value
    #define sfr_TIM3_CCR2H_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2H;


  /** TIM3 Capture/Compare register 2 low (CCR2L at 0x5293) */
  union {

    /// bytewise access to CCR2L
    uint8_t  byte;

    /// bitwise access to register CCR2L
    struct {
      BITS   CCR2                : 8;      // bits 0-7
    };  // CCR2L bitfield

    /// register _TIM3_CCR2L reset value
    #define sfr_TIM3_CCR2L_RESET_VALUE   ((uint8_t) 0x00)

  } CCR2L;


  /** TIM3 Break register (BKR at 0x5294) */
  union {

    /// bytewise access to BKR
    uint8_t  byte;

    /// bitwise access to register BKR
    struct {
      BITS   LOCK                : 2;      // bits 0-1
      BITS   OSSI                : 1;      // bit 2
      BITS                       : 1;      // 1 bit
      BITS   BKE                 : 1;      // bit 4
      BITS   BKP                 : 1;      // bit 5
      BITS   AOE                 : 1;      // bit 6
      BITS   MOE                 : 1;      // bit 7
    };  // BKR bitfield

    /// register _TIM3_BKR reset value
    #define sfr_TIM3_BKR_RESET_VALUE   ((uint8_t) 0x00)

  } BKR;


  /** TIM3 Output idle state register (OISR at 0x5295) */
  union {

    /// bytewise access to OISR
    uint8_t  byte;

    /// bitwise access to register OISR
    struct {
      BITS   OIS1                : 1;      // bit 0
      BITS                       : 1;      // 1 bit
      BITS   PIS2                : 1;      // bit 2
      BITS                       : 5;      // 5 bits
    };  // OISR bitfield

    /// register _TIM3_OISR reset value
    #define sfr_TIM3_OISR_RESET_VALUE   ((uint8_t) 0x00)

  } OISR;

} TIM3_t;

/// access to TIM3 SFR registers
#define sfr_TIM3   (*((TIM3_t*) 0x5280))


//------------------------
// Module TIM4
//------------------------

/** struct containing TIM4 module registers */
typedef struct {

  /** TIM4 Control register 1 (CR1 at 0x52e0) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   CEN                 : 1;      // bit 0
      BITS   UDIS                : 1;      // bit 1
      BITS   URS                 : 1;      // bit 2
      BITS   OPM                 : 1;      // bit 3
      BITS                       : 3;      // 3 bits
      BITS   ARPE                : 1;      // bit 7
    };  // CR1 bitfield

    /// register _TIM4_CR1 reset value
    #define sfr_TIM4_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** TIM4 Control register 2 (CR2 at 0x52e1) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS                       : 4;      // 4 bits
      BITS   MMS                 : 3;      // bits 4-6
      BITS                       : 1;      // 1 bit
    };  // CR2 bitfield

    /// register _TIM4_CR2 reset value
    #define sfr_TIM4_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** TIM4 Slave Mode Control Register (SMCR at 0x52e2) */
  union {

    /// bytewise access to SMCR
    uint8_t  byte;

    /// bitwise access to register SMCR
    struct {
      BITS   SMS                 : 3;      // bits 0-2
      BITS                       : 1;      // 1 bit
      BITS   TS                  : 3;      // bits 4-6
      BITS   MSM                 : 1;      // bit 7
    };  // SMCR bitfield

    /// register _TIM4_SMCR reset value
    #define sfr_TIM4_SMCR_RESET_VALUE   ((uint8_t) 0x00)

  } SMCR;


  /** TIM4 Interrupt enable register (IER at 0x52e3) */
  union {

    /// bytewise access to IER
    uint8_t  byte;

    /// bitwise access to register IER
    struct {
      BITS   UIE                 : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TIE                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // IER bitfield

    /// register _TIM4_IER reset value
    #define sfr_TIM4_IER_RESET_VALUE   ((uint8_t) 0x00)

  } IER;


  /** TIM4 Status register 1 (SR1 at 0x52e4) */
  union {

    /// bytewise access to SR1
    uint8_t  byte;

    /// bitwise access to register SR1
    struct {
      BITS   UIF                 : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TIF                 : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // SR1 bitfield

    /// register _TIM4_SR1 reset value
    #define sfr_TIM4_SR1_RESET_VALUE   ((uint8_t) 0x00)

  } SR1;


  /** TIM4 Event Generation register (EGR at 0x52e5) */
  union {

    /// bytewise access to EGR
    uint8_t  byte;

    /// bitwise access to register EGR
    struct {
      BITS   UG                  : 1;      // bit 0
      BITS                       : 5;      // 5 bits
      BITS   TG                  : 1;      // bit 6
      BITS                       : 1;      // 1 bit
    };  // EGR bitfield

    /// register _TIM4_EGR reset value
    #define sfr_TIM4_EGR_RESET_VALUE   ((uint8_t) 0x00)

  } EGR;


  /** TIM4 Counter (CNTR at 0x52e6) */
  union {

    /// bytewise access to CNTR
    uint8_t  byte;

    /// bitwise access to register CNTR
    struct {
      BITS   CNT                 : 8;      // bits 0-7
    };  // CNTR bitfield

    /// register _TIM4_CNTR reset value
    #define sfr_TIM4_CNTR_RESET_VALUE   ((uint8_t) 0x00)

  } CNTR;


  /** TIM4 Prescaler register (PSCR at 0x52e7) */
  union {

    /// bytewise access to PSCR
    uint8_t  byte;

    /// bitwise access to register PSCR
    struct {
      BITS   PSC                 : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // PSCR bitfield

    /// register _TIM4_PSCR reset value
    #define sfr_TIM4_PSCR_RESET_VALUE   ((uint8_t) 0x00)

  } PSCR;


  /** TIM4 Auto-reload register low (ARR at 0x52e8) */
  union {

    /// bytewise access to ARR
    uint8_t  byte;

    /// bitwise access to register ARR
    struct {
      BITS   ARR                 : 8;      // bits 0-7
    };  // ARR bitfield

    /// register _TIM4_ARR reset value
    #define sfr_TIM4_ARR_RESET_VALUE   ((uint8_t) 0xFF)

  } ARR;

} TIM4_t;

/// access to TIM4 SFR registers
#define sfr_TIM4   (*((TIM4_t*) 0x52e0))


//------------------------
// Module USART
//------------------------

/** struct containing USART module registers */
typedef struct {

  /** USART Status Register (SR at 0x5230) */
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

    /// register _USART_SR reset value
    #define sfr_USART_SR_RESET_VALUE   ((uint8_t) 0xC0)

  } SR;


  /** USART Data Register (DR at 0x5231) */
  union {

    /// bytewise access to DR
    uint8_t  byte;

    /// bitwise access to register DR
    struct {
      BITS   DR                  : 8;      // bits 0-7
    };  // DR bitfield

    /// register _USART_DR reset value
    #define sfr_USART_DR_RESET_VALUE   ((uint8_t) 0x00)

  } DR;


  /** USART Baud Rate Register 1 (BRR1 at 0x5232) */
  union {

    /// bytewise access to BRR1
    uint8_t  byte;

    /// bitwise access to register BRR1
    struct {
      BITS   USART_DIV           : 8;      // bits 0-7
    };  // BRR1 bitfield

    /// register _USART_BRR1 reset value
    #define sfr_USART_BRR1_RESET_VALUE   ((uint8_t) 0x00)

  } BRR1;


  /** USART Baud Rate Register 2 (BRR2 at 0x5233) */
  union {

    /// bytewise access to BRR2
    uint8_t  byte;

    /// bitwise access to register BRR2
    struct {
      BITS   USART_DIV           : 8;      // bits 0-7
    };  // BRR2 bitfield

    /// register _USART_BRR2 reset value
    #define sfr_USART_BRR2_RESET_VALUE   ((uint8_t) 0x00)

  } BRR2;


  /** USART Control Register 1 (CR1 at 0x5234) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   PIEN                : 1;      // bit 0
      BITS   PS                  : 1;      // bit 1
      BITS   PCEN                : 1;      // bit 2
      BITS   WAKE                : 1;      // bit 3
      BITS   MSL                 : 1;      // bit 4
      BITS   USARTD              : 1;      // bit 5
      BITS   T8                  : 1;      // bit 6
      BITS   R8                  : 1;      // bit 7
    };  // CR1 bitfield

    /// register _USART_CR1 reset value
    #define sfr_USART_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** USART Control Register 2 (CR2 at 0x5235) */
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

    /// register _USART_CR2 reset value
    #define sfr_USART_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;


  /** USART Control Register 3 (CR3 at 0x5236) */
  union {

    /// bytewise access to CR3
    uint8_t  byte;

    /// bitwise access to register CR3
    struct {
      BITS   LBCL                : 1;      // bit 0
      BITS   CPHA                : 1;      // bit 1
      BITS   CPOL                : 1;      // bit 2
      BITS   CLKEN               : 1;      // bit 3
      BITS   STOP                : 2;      // bits 4-5
      BITS                       : 2;      // 2 bits
    };  // CR3 bitfield

    /// register _USART_CR3 reset value
    #define sfr_USART_CR3_RESET_VALUE   ((uint8_t) 0x00)

  } CR3;


  /** USART Control Register 4 (CR4 at 0x5237) */
  union {

    /// bytewise access to CR4
    uint8_t  byte;

    /// bitwise access to register CR4
    struct {
      BITS   ADD                 : 4;      // bits 0-3
      BITS                       : 4;      // 4 bits
    };  // CR4 bitfield

    /// register _USART_CR4 reset value
    #define sfr_USART_CR4_RESET_VALUE   ((uint8_t) 0x00)

  } CR4;

} USART_t;

/// access to USART SFR registers
#define sfr_USART   (*((USART_t*) 0x5230))


//------------------------
// Module WFE
//------------------------

/** struct containing WFE module registers */
typedef struct {

  /** WFE control register 1 (CR1 at 0x50a6) */
  union {

    /// bytewise access to CR1
    uint8_t  byte;

    /// bitwise access to register CR1
    struct {
      BITS   TIM2_EV0            : 1;      // bit 0
      BITS   TIM2_EV1            : 1;      // bit 1
      BITS                       : 2;      // 2 bits
      BITS   EXTI_EV0            : 1;      // bit 4
      BITS   EXTI_EV1            : 1;      // bit 5
      BITS   EXTI_EV2            : 1;      // bit 6
      BITS   EXTI_EV3            : 1;      // bit 7
    };  // CR1 bitfield

    /// register _WFE_CR1 reset value
    #define sfr_WFE_CR1_RESET_VALUE   ((uint8_t) 0x00)

  } CR1;


  /** WFE control register 2 (CR2 at 0x50a7) */
  union {

    /// bytewise access to CR2
    uint8_t  byte;

    /// bitwise access to register CR2
    struct {
      BITS   EXTI_EV4            : 1;      // bit 0
      BITS   EXTI_EV5            : 1;      // bit 1
      BITS   EXTI_EV6            : 1;      // bit 2
      BITS   EXTI_EV7            : 1;      // bit 3
      BITS   EXTI_EVB            : 1;      // bit 4
      BITS   EXTI_EVD            : 1;      // bit 5
      BITS                       : 2;      // 2 bits
    };  // CR2 bitfield

    /// register _WFE_CR2 reset value
    #define sfr_WFE_CR2_RESET_VALUE   ((uint8_t) 0x00)

  } CR2;

} WFE_t;

/// access to WFE SFR registers
#define sfr_WFE   (*((WFE_t*) 0x50a6))


//------------------------
// Module WDG
//------------------------

/** struct containing WDG module registers */
typedef struct {

  /** WWDG control register (WWDG_CR at 0x50d3) */
  union {

    /// bytewise access to WWDG_CR
    uint8_t  byte;

    /// bitwise access to register WWDG_CR
    struct {
      BITS   T                   : 7;      // bits 0-6
      BITS   WDGA                : 1;      // bit 7
    };  // WWDG_CR bitfield

    /// register _WDG_WWDG_CR reset value
    #define sfr_WDG_WWDG_CR_RESET_VALUE   ((uint8_t) 0x7F)

  } WWDG_CR;


  /** WWDG window register (WWDG_WR at 0x50d4) */
  union {

    /// bytewise access to WWDG_WR
    uint8_t  byte;

    /// bitwise access to register WWDG_WR
    struct {
      BITS   W                   : 7;      // bits 0-6
      BITS                       : 1;      // 1 bit
    };  // WWDG_WR bitfield

    /// register _WDG_WWDG_WR reset value
    #define sfr_WDG_WWDG_WR_RESET_VALUE   ((uint8_t) 0x7F)

  } WWDG_WR;


  /// Reserved register (11B)
  uint8_t     Reserved_1[11];


  /** IWDG Key register (IWDG_KR at 0x50e0) */
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


  /** IWDG Prescaler register (IWDG_PR at 0x50e1) */
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


  /** IWDG Reload register (IWDG_RLR at 0x50e2) */
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
#define sfr_WDG   (*((WDG_t*) 0x50d3))


// undefine local macros
#undef  BITS

// required for C++
#ifdef __cplusplus
  }   // extern "C"
#endif

/*-------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-------------------------------------------------------------------------*/
#endif // STM8TL53C4_H
