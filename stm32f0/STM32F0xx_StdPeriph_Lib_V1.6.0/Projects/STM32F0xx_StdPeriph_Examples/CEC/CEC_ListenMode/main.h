/**
  ******************************************************************************
  * @file    CEC/CEC_ListenMode/main.h 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    13-October-2021
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#ifdef USE_STM320518_EVAL
  #include "stm320518_eval.h"
  #include "stm320518_eval_lcd.h"
#else 
  #include "stm32072b_eval.h"
  #include "stm32072b_eval_lcd.h" 
#endif /* USE_STM320518_EVAL */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below if you will use the CEC peripheral as a Device1 */
//#define DEVICE_1
/* Uncomment the line below if you will use the CEC peripheral as a Device2 */ 
//#define DEVICE_2 
/* Uncomment the line below if you will use the CEC peripheral as a Spy Device */
#define DEVICE_3 

#define DEVICE_ADDRESS_1               0x01  /* CEC device 1 address      */
#define DEVICE_ADDRESS_2               0x03  /* CEC device 2 address      */
#define DEVICE_ADDRESS_3               0x04  /* CEC Spy device address */
                     
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

