/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/timer.c 
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
#include "timer.h"
#include <stdio.h>

#include "heater.h"
#include "power.h"
#include "key.h"
#include "universal_key.h"




#define  MS_PER_MIN       (60*1000)

DISPLAY_DATA *pSegLed = NULL;




/********************************************************************************
*fun name:          heat_running  
*fun description:   pid control heater heating 
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
static void heat_running(void)
{
	static u16 HeatTicks = 0;
#if 1		
	if(((HEATER_NORMAL == SysStat.pHtrS->mode) &&\
		(HEATER_ON == SysStat.pHtrS->HtrOn)     &&\
		(HEATING == SysStat.pHtrS->heating)     &&\
		(HEATER_NO_ERR == SysStat.pHtrS->HtrErr)) ||
		(HEATER_ADJUST == SysStat.pHtrS->mode))
#endif			
	{		
		if(HeatTicks < HeatTime)
		{
			POWER_HEATER_ON;
			//HINT_DEBUG("h%d\r\n",HeatTime);
			Heating = HEATING;
		}
		else
		{
			POWER_HEATER_OFF;
			//HINT_DEBUG("t%d\r\n",HeatTime);
			Heating = COOLING;
		}
		if(HeatTicks++ >= FULL_TIME_HEAT)HeatTicks= 0;
	}
	else
	{
		POWER_HEATER_OFF;
		Heating = COOLING;
	}
}
/********************************************************************************
*fun name:          TIM2_IRQHandler  
*fun description:   timer2 irq 
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void TIM2_IRQHandler(void)
{
	static u8 i = 0;
       //����Ƿ�����������¼�
   if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
   {
		//���TIM2���жϴ�����λ
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
#if 0		
		if(i)
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_10);
			GPIO_SetBits(GPIOB, GPIO_Pin_11);
			i = 0;
		}
		else
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_10);
			GPIO_ResetBits(GPIOB, GPIO_Pin_11);
			i = 1;
		}
#endif			 
		SysStat.Tmr2CntMs++;
		process_timer();
		if(SysStat.pPwrStat->PwrSet.EnTimeCtrPwr)
		{
			if( compare_timestamp(SysStat.PwrOffTmrStmp,ph.pwr.PwrOffTimerM*MS_PER_MIN) )
			{
				SysStat.pPwrStat->PwrOn = POWER_OFF; 
				SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 0;
			}
			
		}
		i++;
		if(9 == i)
		{
			Pannelkey_Polling();
			i = 0;
		}
//display(pSegLed);
		if(0 == SysStat.Tmr2CntMs%50)
		{
			TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
		}
		if(HOT_MODE == ph.wm)
		{
			heat_running();
		}
		
   }
}


/********************************************************************************
*fun name:          timer_NVIC_cfg  
*fun description:   config timer2 priority 
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void timer_NVIC_cfg()

{
       NVIC_InitTypeDef NVIC_InitStructure;
        //ѡ���жϷ���1
        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        //ѡ��TIM2���ж�ͨ��
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;      
        //��ռʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
       //��Ӧʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        //ʹ���ж�
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);


		//timer1
		  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;      
        //��ռʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
       //��Ӧʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        //ʹ���ж�
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

}



/********************************************************************************
*fun name:          timer2_cfg 
*fun description:   config timer2 timer 1ms 
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
static void timer2_cfg(void)
{

       TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

       //���½�Timer����Ϊȱʡֵ
       TIM_DeInit(TIM2);

       //�����ڲ�ʱ�Ӹ�TIM2�ṩʱ��Դ
       TIM_InternalClockConfig(TIM2);

	   //Tout= ((TIM_Prescaler+1)*(TIM_Period+1))/Tclk
       //Tclk is into TIMER clk ,is 24M
       TIM_TimeBaseStructure.TIM_Prescaler = 2-1;
       //����ʱ�ӷָ�
       TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
       //���ü�����ģʽΪ���ϼ���ģʽ
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
       //���ü��������С
       TIM_TimeBaseStructure.TIM_Period = TIM2_PERIOD-1;
       //������Ӧ�õ�TIM2��
       TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);


       //�������жϱ�־
       TIM_ClearFlag(TIM2, TIM_FLAG_Update);

       //��ֹARRԤװ�ػ�����
       TIM_ARRPreloadConfig(TIM2, DISABLE);
	   
       //����TIM2���ж�
       TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}



/********************************************************************************
*fun name:          Time1_Cfg 
*fun description:   tim1_ch1 input capture config 
*input param:                     
*output param:
*return value:  
*remark:        creat 2017-04-14  guolei
********************************************************************************/
static void Time1_Cfg(void)
{  
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef  TIM_ICInitStruct;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
    
    TIM_TimeBaseInitStruct.TIM_ClockDivision=0;
    TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period=TIM2_PERIOD-1;//TIM2_PERIOD-1;
    TIM_TimeBaseInitStruct.TIM_Prescaler=2-1;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
    
    TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter=0;
    TIM_ICInitStruct.TIM_ICPolarity=TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICPrescaler=TIM_ICPSC_DIV2;
    TIM_ICInitStruct.TIM_ICSelection=TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM1,&TIM_ICInitStruct);

	
    TIM_ClearFlag(TIM1,TIM_IT_CC1);
    TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
    TIM_PWMIConfig(TIM1,&TIM_ICInitStruct);
    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM1,TIM_MasterSlaveMode_Enable);
    TIM_Cmd(TIM1,ENABLE);   
	TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
}



/********************************************************************************
*fun name:          TIM1_CC_IRQHandler 
*fun description:   tim1_ch1 input capture irq 
*input param:                     
*output param:
*return value:  
*remark:        creat 2017-04-14  guolei
********************************************************************************/
void TIM1_CC_IRQHandler(void)
{   
	//static u16 count1 = 0;
	static u16 count2 = 0;
	//static u16 count3 = 0;
	static u8 stat = 0;
  
   if(TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET)
	{
	 	TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);

		if(0 == stat)
		{
			//count1 = TIM1->CCR1;
			stat = 1;
		}
		else if(1 == stat)
		{
			count2 = TIM1->CCR1;
			//count3 = count2-count1;
			// 2��������
			SysStat.PwmFreq = TIM2_PERIOD/(count2+2);
			TIM_ITConfig(TIM1, TIM_IT_CC1, DISABLE);
		}
	}
 }






/********************************************************************************
*fun name:          delay_us 
*fun description:   delay us    us
*input param:       
*output param:
*return value:  
*remark:        creat 2017-03-02  guolei
********************************************************************************/
void delay_us(unsigned int us)
{	
	while(us--);

}

/********************************************************************************
*fun name:          real_delay_10us 
*fun description:   delay us10  10us
*input param:       
*output param:
*return value:  
*remark:        creat 2017-03-02  guolei
********************************************************************************/
void real_delay_10us(unsigned int us10)
{
	u32 i = 0;
	
	while(us10--)
		for(i=0;i<53;i++);
		
}

/********************************************************************************
*fun name:          real_delay_ms 
*fun description:   delay ms ms
*input param:       
*output param:
*return value:  
*remark:        creat 2017-03-02  guolei
********************************************************************************/
void real_delay_ms(unsigned int ms)
{
	u32 i = 0;
	
	while(ms--)
		for(i=0;i<4700;i++);
		
}




/********************************************************************************
*fun description:   timer for displpay. update seg displayer per 10ms
*input param:    
*output param:
*return value:
********************************************************************************/
void time_init(void)
{
	
	timer_NVIC_cfg();
    timer2_cfg();
	Time1_Cfg();
	
	//������ʱ��2
	TIM_Cmd(TIM2,ENABLE);	
	
}


u32 get_tick(void)
{
	return SysStat.Tmr2CntMs;
}
