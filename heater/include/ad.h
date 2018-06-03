
#if 0

#ifndef __AD_H__
#define __AD_H__ 
/******************************************************************************************
																		ͷ�ļ�����
*******************************************************************************************/
#include "config.h"
#include "stm32f10x.h"
/******************************************************************************************
																		�궨��
*******************************************************************************************/
#define 	_ENABLE_EXTI_11_IT							((EXTI->PR |= (1<<11)),EXTI->IMR |= (1<<11))
#define 	_DISABLE_EXTI_11_IT							(EXTI->IMR &= ~(1<<11))

extern 	u16 														externAdcVREF;
#define ADC_EXT_VREF										2.5f
#define ADC_INT_VREF										externAdcVREF   //2495
#define	ADC_COUNT												34
//���صĴ������Ͷ���
#define ADC_ERROR_NO										0
#define ADC_ERROR_NOCODE								-1
#define ADC_ERROR_OTHER									-2
#define ADC_ERROR_PEVENT_NULL						-3	//��������
#define	ADC_ERROR_MODE						    	-4	//����Ĺ���ģʽ
#define ADC_ERROR_CMD	  								-5  //����Ĺ�����
//�ⲿ�����������붨��
#define ADC_CH_MAX											9  //ADCͨ������,����ʵʱʱ��Ҫ�󲻸ߵģ���������74HC4051������ͨ������
#define AIN01_CODE											0x00
#define AIN02_CODE											0x01
#define AIN03_CODE											0x02
#define AIN04_CODE											0x03
#define AIN05_CODE											0x04
#define AIN06_CODE											0x05
#define AIN07_CODE											0x06
#define AIN08_CODE											0x07
#define AIN09_CODE											0x08

//#define ADC_DMA_USING
#define ADC_NVIC_USING
#define ADC_USING												//ADC1����ʹ��

extern void ad_init(void);					
#if 0
extern  int8s_t readADCData(int8u_t  code, float_t* pdat);
extern int8s_t removeDecimal(float_t *pdat,int8u_t bit);
#endif

#endif

#endif


#ifndef  __AD_H__
#define  __AD_H__

#define 	ADC1_CHANNEL_CNT			 5			//ADCʹ��5��ͨ�� and cpu temp adc

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


extern void ad_init(void);
extern u8 get_adc_data(u16* pdat);

#endif


/******************************************************************************************
																		end file
*******************************************************************************************/

