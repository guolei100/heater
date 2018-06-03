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
#include "ad.h"
#include "stm32f10x_adc.h"
#include <stdio.h>
#include "power.h"

#define 	ADC1_DMA_CHANNEL          DMA1_Channel1		//ADC1 and ADC2 only use DMA1_Channel1
#define     CONVETING                   1
#define     CONVETED                    0

#define 	ADC1_SAMPLE_CNT	    	 8	
#define     ADC1_MAX_CNT                2048
#define     ADC1_MAX_CNT_AVG            11

/******************************************************************************************
																		ȫ�ֱ���
*******************************************************************************************/

volatile	u16 	AdcValue[ADC1_SAMPLE_CNT][ADC1_CHANNEL_CNT];	//DMA target memory
u16 	SortData[ADC1_SAMPLE_CNT];	//DMA target memory 

u32     AvgValue[ADC1_SAMPLE_CNT];

volatile 	u8  	AdcEnd = CONVETING;	
u16 			    externAdcVREF 	= 3300;

u32     AdcCnt = 0;



u8 calc_ad_avg(void);
/*******************************************************************************************
													NVIC_Configuration
*******************************************************************************************/





void ADC1_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1,ADC_IT_EOC)){	
		AdcEnd = CONVETED;
		ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
	} 	
}

void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
	{
		AdcEnd = CONVETED;
		DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
}



/*******************************************************************************************
													RCC_Configuration
*******************************************************************************************/
static void RCC_Configuration(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* Enable ADC1 and GPIOC clock */ 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
}

/*******************************************************************************************
													GPIO_Configuration
*******************************************************************************************/
static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//PA1=VREF��PA4=HV+,PA5=HV-
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

#if  0
//��ʱ����ʱ50us�ɼ�
static void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	/*?TIM2 clock enable*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	//����TIM2Ϊ�Ƚ�����л�ģʽ,TIM2_CLK=36MHz Ĭ�ϣ�Ԥ��Ƶ��TIM_Prescaler=36; ��TIM3_counter clock = 1MHz
	/*Time base configuration,50us*/
	TIM_TimeBaseStructure.TIM_Period = 10;					//��һ�������¼��Զ���װֵ0x0000~0xFFFF
	TIM_TimeBaseStructure.TIM_Prescaler = (24-1);			//��ʱ��ʱ��Ԥ��Ƶֵ=24000000/23+1=1MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//TIMEʱ�ӷָ�,���ָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM����ģʽ�����ϼ���
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);					//��ʼ��ʱ����λ
	//���ñȽ�����Ĳ���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM����Ƚϴ���ģʽTIM_OCMode_Toggle
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//����ͨ����Ч
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Disable;//����ͨ����Ч��ֻ����TIM1��TIM8
	TIM_OCInitStructure.TIM_Pulse = 5;//��װ�벶��ȽϼĴ�������ֵ;:1KHz
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;//TIM����Ƚϼ���Ϊ��
	//ͨ��2	����ADת��
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	/* TIM2 counter enable */
 	TIM_Cmd(TIM2, DISABLE);
	/* TIM2 main Output Enable */
  TIM_CtrlPWMOutputs(TIM2, ENABLE);  
}


#endif

//DMA�жϲɼ�
static void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(ADC1_DMA_CHANNEL);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_BASE+0x4C;	   			//�������ַ��ADC
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AdcValue;		//�洢����ַ���ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;				//��Ϊ���ݴ������Դ
	DMA_InitStructure.DMA_BufferSize = (ADC1_CHANNEL_CNT*ADC1_SAMPLE_CNT);	  							//3��������,��3��ͨ����ADC
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  	//�����ַ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		  				//�ڴ��ַ����,��ΪҪ��3·
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	 //�������ݵĿ��16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		   			//�ڴ����ݿ��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		   		//������ѭ��ģʽ��ֻ�����赽�ڴ������ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;	   	//�ϸ����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		   			//�豸���ڴ�ģʽ
	DMA_Init(ADC1_DMA_CHANNEL, &DMA_InitStructure);				   	//��ʼ��DMA
	
//	DMA_ITConfig(ADC1_DMA_CHANNEL, DMA_IT_TC | DMA_IT_TE, ENABLE); //ʹ��ָ��ADC��DMA�жϣ�������ɺʹ����ж�
//	DMA_ClearFlag(DMA1_FLAG_TC1);						   									//���DMAͨ��1�ж�
//	DMA_ClearITPendingBit(DMA_IT_TC | DMA_IT_TE);								//����ж�
	/* Enable DMA1 channel1 */
	DMA_Cmd(ADC1_DMA_CHANNEL, ENABLE);	
	DMA_ITConfig(ADC1_DMA_CHANNEL, DMA_IT_TC, ENABLE); //ʹ��ADC��DMA
}
/*********************************************************************************************************************
��������: 	initializationADC  

��������:   ��ʼ��ADCӲ��	

�䡡��:   	��

�䡡��:    	��	  	                 
*********************************************************************************************************************/
void ad_init(void)
{
	ADC_InitTypeDef ADC_InitStructure;


	//ADC clock :12M
	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
	/* Enable ADC1 and GPIOC clock */
	RCC_Configuration();
	GPIO_Configuration();
	/* Enable DMA1  */
	DMA_Configuration();
	/* Enable TIM2  */
//	TIM2_Configuration();
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;						//�����ڷ�ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//�����ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ADC1_CHANNEL_CNT;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/* ADC1 regular channel 10-15 configuration */ //�趨������ת��˳���ʱ��
	//a dot adc timer = 55.5+12.5 = 68/ADC_CLOCK=5.67us
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);  //AD_I
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_55Cycles5);  //AD_U
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);  //heater temp
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 4, ADC_SampleTime_55Cycles5);  //cpu temp
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 5, ADC_SampleTime_55Cycles5);  //���յ�ѹ

	ADC_TempSensorVrefintCmd(ENABLE);
	
	
	/* Enable ADC1 external trigger conversion */ 
//	ADC_ExternalTrigConvCmd(ADC1, DISABLE);	   //��ʱ�����������ADCģ�������ⲿ
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1); //����У׼�Ĵ���
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1)); //��ȡ����У׼�Ĵ���״̬
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);	 //��ʼָ����ADCУ׼״̬
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1)); //��ȡָ����ADCУ׼״̬
	/* register adc1 */
	/* Enable automatic injected conversion start after regular one */

	ADC_SoftwareStartConvCmd(ADC1,  ENABLE);
}








void sort_data(u16 *a,u16 n)
{

	u16 i,j,temp;

	for(i=0;i<n-1;i++)

	for(j=i+1;j<n;j++) /*ע��ѭ����������*/

	if(a[i]>a[j]) {
		temp=a[i];
		a[i]=a[j];
		a[j]=temp;
	}

}



u8 calc_ad_avg(void)
{
	u16 i = 0,j=0;
	static u32 z = 0;

#if 0
	AdcCnt++;
	if(3== AdcCnt)// when update pwm ,delay  and wait adc pin volt is stable
	{
		z = 0;
		for(i=0; i<ADC1_CHANNEL_CNT; i++)
			AvgValue[i] = 0;
		
	}
	if(AdcCnt>=PWM_UPDATE_CNT)AdcCnt = 0;
#endif
	
	for(i=0; i<ADC1_CHANNEL_CNT; i++)
	{
		for(j=0; j<ADC1_SAMPLE_CNT; j++)
		{
			AvgValue[i] += AdcValue[j][i];
		}	
	}
	z += ADC1_SAMPLE_CNT;
	if(z >= ADC1_MAX_CNT)
	{
		for(i=0; i<ADC1_CHANNEL_CNT; i++)
		{
			AvgValue[i] >>= ADC1_MAX_CNT_AVG;// 2^ADC1_MAX_CNT_AVG = ADC1_MAX_CNT

			//AvgValue[i] /= ADC1_MAX_CNT;
		}
		z = 0;
		return 0;
	}
	else
	{
		return 1;
	}
}




u8 get_adc_data(u16* pdat)
{
	u8 i;
	static u32     AV[ADC1_SAMPLE_CNT];

	if(CONVETING == AdcEnd) return 1;
	AdcEnd = CONVETING;
	if(pdat == NULL){
		return 1;
	}

	if(calc_ad_avg())
	{
		for(i=0;i<ADC1_CHANNEL_CNT;i++){
			pdat[i] = AV[i];
		}
		return 1;
	}
	for(i=0;i<ADC1_CHANNEL_CNT;i++){
		pdat[i] = AvgValue[i];
		AV[i] = AvgValue[i];
		AvgValue[i] = 0;
	}

	return 0;
}




/*********************************************************************************************************
                                              end file
*********************************************************************************************************/


