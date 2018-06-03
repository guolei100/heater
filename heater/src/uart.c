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
#include "uart.h"
#include <stdio.h>
#include "key.h"
#include "ph.h"





int fputc(int ch, FILE *f)

{
	//u32 tick;

	//tick = SysStat.Tmr2CntMs;
	USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等

	while (!(USART1->SR & USART_FLAG_TXE))
	{
		//if((SysStat.Tmr2CntMs - tick) > 100)
		{
			//break;
		}
	}

	return (ch);

}




void USART1_IRQHandler(void) 
{ 
    u16 i; 
     
    if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET) 
    {               
        i = USART_ReceiveData(USART1); 
        cmd_explain(i);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) 
        { 
        }                
    }
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
    { 
         
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}



/********************************************************************************
*fun name:          print_work_param 
*fun description:   print work parma
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
void print_work_param(PWR_HTR *pWorkParam)
{
	if(NULL==pWorkParam)return;

	if(POWER_MODE==pWorkParam->wm) 
		HINT_DEBUG("power mode\r\n");
	else
		HINT_DEBUG("heater mode\r\n");
	
	HINT_DEBUG("set volt:%dmv\r\n",pWorkParam->pwr.pv.SetVol);
//	HINT_DEBUG("volt total duty:%d\n",pWorkParam->pwr.pv.VoltDutyTotal);
	HINT_DEBUG("volt:%dmv\r\n",SysStat.pPwrStat->pvs.CurVol*10);
	HINT_DEBUG("volt duty:%d\r\n",SysStat.pPwrStat->pvs.pwmu);
	HINT_DEBUG("ADJUST_INDEX duty:%d\r\n",adjustDat.pwrJ.VPwmSet[ADJUST_INDEX]);
	HINT_DEBUG("INDEX_60V duty:%d\r\n",adjustDat.pwrJ.VPwmSet[INDEX_60V]);

	
	HINT_DEBUG("set current:%dmA\r\n",pWorkParam->pwr.pc.SetCur*10);
	
//	HINT_DEBUG("current total duty:%d\n",pWorkParam->pwr.pc.CurDutyTotal);
	
	HINT_DEBUG("current:%dmA\r\n",SysStat.pPwrStat->pcs.CurCur);
	
	HINT_DEBUG("current duty:%d\r\n",SysStat.pPwrStat->pcs.pwmi);
	HINT_DEBUG("INDEX_0_1A duty:%d\r\n",adjustDat.pwrJ.APwmSet[INDEX_0_1A]);
	HINT_DEBUG("INDEX_3A duty:%d\r\n",adjustDat.pwrJ.APwmSet[INDEX_3A]);
	
	if(SET_VOLTAGE_MODE==ph.pwr.CurSetMode)
		HINT_DEBUG("set volt\r\n");
	else
		HINT_DEBUG("set cur\r\n");
	
}

/********************************************************************************
*fun name:          uart1_cfg 
*fun description:   set uart1 baud
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
void uart1_cfg(u32 baud)
{
		USART_InitTypeDef USART_InitStructure;
	
	 	USART_InitStructure.USART_BaudRate =baud;                                                  // 波特率为：115200
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;                          // 8位数据
        USART_InitStructure.USART_StopBits = USART_StopBits_1;                                  // 在帧结尾传输1个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No ;                                  // 奇偶失能
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        // 硬件流控制失能
        
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 发送使能+接收使能
        /* Configure USART1 basic and asynchronous paramters */
        USART_Init(USART1, &USART_InitStructure);
}

/********************************************************************************
*fun name:          uart1_NVIC_config 
*fun description:   config uart1 irq
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
void uart1_NVIC_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure); 
}

/********************************************************************************
*fun name:          uart1_init 
*fun description:   config uart1
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-06  guolei
********************************************************************************/
void uart1_init(void)
{
         USART_InitTypeDef USART_InitStructure;
        USART_ClockInitTypeDef  USART_ClockInitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE  );//RCC_APB2Periph_TIM1


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
        
        USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;                        // 时钟低电平活动
        USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;                                // 时钟低电平
        USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;                                // 时钟第二个边沿进行数据捕获
        USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;                // 最后一位数据的时钟脉冲不从SCLK输出
        /* Configure the USART1 synchronous paramters */
        USART_ClockInit(USART1, &USART_ClockInitStructure);                                        // 时钟参数初始化设置
                                                                                                                                                 
        USART_InitStructure.USART_BaudRate =9600;                                                  // 波特率为：115200
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;                          // 8位数据
        USART_InitStructure.USART_StopBits = USART_StopBits_1;                                  // 在帧结尾传输1个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No ;                                  // 奇偶失能
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;        // 硬件流控制失能
        
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // 发送使能+接收使能
        /* Configure USART1 basic and asynchronous paramters */
        USART_Init(USART1, &USART_InitStructure);
            
          /* Enable USART1 */
        USART_ClearFlag(USART1, USART_IT_RXNE);                         //清中断，以免一启用中断后立即产生中断
        USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);                //使能USART1中断源
		uart1_NVIC_config();
		USART_Cmd(USART1, ENABLE);                                                        //USART1总开关：开启

}

