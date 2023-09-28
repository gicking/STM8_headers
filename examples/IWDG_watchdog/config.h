/**
  \file config.h
   
  \brief set project configurations
   
  set project configurations like used device or board etc.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _CONFIG_H_
#define _CONFIG_H_


/*----------------------------------------------------------
    SELECT BOARD
----------------------------------------------------------*/
//#define SDUINO
//#define STM8L_DISCOVERY
//#define STM8S_DISCOVERY
#define NUCLEO_8S207K8
//#define NUCLEO_8S208RB


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(SDUINO)
  #include "../../include/STM8S105K6.h"

#elif defined(STM8L_DISCOVERY)
  #include "../../include/STM8L152C6.h"
  
#elif defined(STM8S_DISCOVERY)
  #include "../../include/STM8S105C6.h"

#elif defined(NUCLEO_8S207K8)
  #include "../../include/STM8S207K8.h"

#elif defined(NUCLEO_8S208RB)
  #include "../../include/STM8S208RB.h"

#else
  #error undefined board
#endif


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
