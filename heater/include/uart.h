#ifndef __UART_H__
#define __UART_H__


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
#include "key.h"

/********************************************************************************
*fun name:          print_work_param 
*fun description:   print work parma
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
extern void print_work_param(PWR_HTR *pWorkParam);


/********************************************************************************
*fun name:          uart1_cfg 
*fun description:   set uart1 baud
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
extern void uart1_cfg(u32 baud);


/********************************************************************************
*fun name:          uart1_init 
*fun description:   config uart1
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
extern void uart1_init(void);

#endif
