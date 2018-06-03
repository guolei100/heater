#ifndef __TIMER_H__
#define __TIMER_H__


/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include "SegLed.h"

#define TIM2_PERIOD   12000UL


/********************************************************************************
*fun name:          delay_us 
*fun description:   delay us    us
*input param:       
*output param:
*return value:  
*remark:        creat 2017-03-02  guolei
********************************************************************************/
extern void delay_us(unsigned int us);

/********************************************************************************
*fun name:          real_delay_10us 
*fun description:   delay us10  10us
*input param:       
*output param:
*return value:  
*remark:        creat 2017-03-02  guolei
********************************************************************************/
extern void real_delay_10us(unsigned int us10);
/********************************************************************************
*fun name:          real_delay_ms 
*fun description:   delay ms ms
*input param:       
*output param:
*return value:  
*remark:        creat 2017-03-02  guolei
********************************************************************************/
extern void real_delay_ms(unsigned int ms);
extern void time_init(void);
extern u32 get_tick(void);
extern DISPLAY_DATA *pSegLed;

#endif
