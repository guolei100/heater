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
#include "stm32f10x_gpio.h"
#include "pwm.h"
#include <stdio.h>



#if 0
static void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	/*?TIM2 clock enable*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	//配置TIM3为比较输出切换模式,TIM3_CLK=36MHz 默认；预分频：TIM_Prescaler=36; 则TIM3_counter clock = 1MHz
	/*Time base configuration*/
	TIM_TimeBaseStructure.TIM_Period = 999;								//下一个更新事件自动重装值0x0000~0xFFFF
	TIM_TimeBaseStructure.TIM_Prescaler = 36;								//定时器时钟预分频值=24000000/47+1=1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//TIME时钟分割,不分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM计数模式，向上计数
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);					//初始化时基单位
	//配置比较输出的参数  //通道1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;//TIM输出比较触发模式TIM_OCMode_Toggle
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//正向通道有效
	//TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;//返向通道无效，只适用TIM1和TIM8
	TIM_OCInitStructure.TIM_Pulse = 500;//待装入捕获比较寄存器脉冲值;:1KHz
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM输出比较极性为高
	//TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_Low;//互补端极性，只适用TIM1和TIM8
	//TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;//比较输出脚空闲状态
	//TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;//空闲状态端极性，只适用TIM1和TIM8
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE); 
	 TIM_CtrlPWMOutputs(TIM3,ENABLE);
#if  0
	//通道4,PWM输出0~5V控制
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 1001;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	//TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	//TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Disable);//TIMx在CCR1上的预装寄存器禁能
	//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 
	//TIM_ARRPreloadConfig(TIM3,ENABLE);//使能预装载
	/* TIM3 counter enable*/
	TIM_Cmd(TIM3,ENABLE);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	/*TIM IT enable*/
	//TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE); //不需要中断，减小CPU负荷
#endif
	
}










static void TIM3_Mode_Config(void)
{
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 TIM_OCInitTypeDef TIM_OCInitStructure;

 /* PWM ??????? */
 u16 CCR1_Val = 500;
 u16 CCR2_Val = 375;
 u16 CCR3_Val = 250;
 u16 CCR4_Val = 125;

 /* -----------------------------------------------------------------------
 TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
 TIM3CLK = 36 MHz, Prescaler = 0x0, TIM3 counter clock = 36 MHz
 TIM3 ARR Register = 999 => TIM3 Frequency = TIM3 counter clock/(ARR + 1)
 TIM3 Frequency = 36 KHz.
 TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
 TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
 TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
 TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
 ----------------------------------------------------------------------- */

 /* Time base configuration */
 //????? 0 ??? 999??? 1000 ?????????
 TIM_TimeBaseStructure.TIM_Period = 999;

 //????????????? 36MHz
 TIM_TimeBaseStructure.TIM_Prescaler = 0;

 TIM_TimeBaseStructure.TIM_ClockDivision = 0; //????????????
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //??????

 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

 /* PWM1 Mode configuration: Channel1 */
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //??? PWM ?? 1
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

 //????????????????????????
 TIM_OCInitStructure.TIM_Pulse = CCR1_Val;

 //????????? CCR1_Val ?????
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

 TIM_OC1Init(TIM3, &TIM_OCInitStructure); //???? 1
 TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

 /* PWM1 Mode configuration: Channel2 */
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

 //???? 2 ????????????????? PWM
 TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

 TIM_OC2Init(TIM3, &TIM_OCInitStructure); //???? 2
 TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

 /* PWM1 Mode configuration: Channel3 */
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;


 TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

 TIM_OC3Init(TIM3, &TIM_OCInitStructure); //???? 3
 TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

 /* PWM1 Mode configuration: Channel4 */
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

 //???? 4 ????????????????? PWM
 TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
 TIM_OC4Init(TIM3, &TIM_OCInitStructure); //???? 4
 TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

 TIM_ARRPreloadConfig(TIM3, ENABLE);


 TIM_Cmd(TIM3, ENABLE); 



}
#endif

void TIM3_PWM_Init(void)
{        
    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

//        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB2Periph_GPIOA, ENABLE); //使能TIM3时钟，GPIOA时钟
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3,ENABLE);
    RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#if 0
  /*GPIOB Configuration: TIM3 channel 3 and 4 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif
        /* Time base configuration */
	//freq=24000000/((TIM_Prescaler+1)*PWM_MAX_VALUE)
    TIM_TimeBaseStructure.TIM_Period = PWM_MAX_VALUE-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 1-1;//24M进行24分频 计数1000，产生事件  周期为1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;// init duty is 0
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;// init duty is 0
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void pwm_u_init(void)
{        
        GPIO_InitTypeDef         GPIO_InitStructure;
 //   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 //   TIM_OCInitTypeDef        TIM_OCInitStructure;


 TIM_ARRPreloadConfig(TIM3, DISABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, DISABLE);


    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void pwm_i_init(void)
{        
    GPIO_InitTypeDef         GPIO_InitStructure;
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
///    TIM_OCInitTypeDef        TIM_OCInitStructure;


 TIM_ARRPreloadConfig(TIM3, DISABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, DISABLE);


    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


#if 0

 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;// init duty is 0
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
#endif 


  
}

void enable_pwm(void)
{
	TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}


/********************************************************************************
*fun description:   PA6 and PA7 config pwm
*input param:    
*output param:
*return value:
********************************************************************************/
void pwm_init(void)
{
	TIM3_PWM_Init();	
	
}


u16   gVoltPwm = 0;
u16   gCurPwm = 0;

/********************************************************************************
*fun description:   current pwm duty set
*input param:       duty :0~999 
*output param:
*return value:
********************************************************************************/
void pwm_i_duty(u16 duty)
{
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	
	if(duty>MAX_PWM_I) duty = MAX_PWM_I;
	gCurPwm = duty;
	
	  /* PWM1 Mode configuration: Channel1 */
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = duty;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;


  
  TIM_OC2Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
 

}

/********************************************************************************
*fun description:   voltage pwm duty set  //PA6
*input param:       duty :0~999 
*output param:
*return value:
********************************************************************************/
void pwm_u_duty(u16 duty)
{
	TIM_OCInitTypeDef        TIM_OCInitStructure;
	
	if(duty>MAX_PWM_U) duty = MAX_PWM_U;

	gVoltPwm = duty;

	//duty = 8;   //test
	  /* PWM1 Mode configuration: Channel1 */
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = duty;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

 

   TIM_OC1Init(TIM3, &TIM_OCInitStructure); 
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

}
