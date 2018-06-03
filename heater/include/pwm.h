#ifndef __PWM_H__
#define __PWM_H__


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

#define  MAX_PWM_U_DUTY  950UL
#define  MAX_PWM_I_DUTY  930UL
#define  PWM_MAX_VALUE   24000UL

#define  MAX_PWM_U   ((MAX_PWM_U_DUTY*PWM_MAX_VALUE)/(1000UL))
#define  MAX_PWM_I   ((MAX_PWM_I_DUTY*PWM_MAX_VALUE)/(1000UL))

#define  DUTY_CYCLE_5  ((5*PWM_MAX_VALUE)/100)   
#define  DUTY_CYCLE_90  ((90*PWM_MAX_VALUE)/100) 

extern u16   gVoltPwm;
extern u16   gCurPwm;



extern void pwm_u_init(void);
extern void pwm_i_init(void);
extern void pwm_init(void);
extern void enable_pwm(void);

/********************************************************************************
*fun description:   current pwm duty set
*input param:       duty :0~999 
*output param:
*return value:
********************************************************************************/
extern void pwm_i_duty(u16 duty);

/********************************************************************************
*fun description:   voltage pwm duty set
*input param:       duty :0~999 
*output param:
*return value:
********************************************************************************/
extern void pwm_u_duty(u16 duty);
#endif
