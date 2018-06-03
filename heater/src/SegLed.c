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
#include "SegLed.h"
#include "timer.h"
#include <stdio.h>
#include "ph.h"





#define SRCLK_L   GPIO_ResetBits(GPIOB, GPIO_Pin_15)//GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_RESET)
#define SRCLK_H   GPIO_SetBits(GPIOB, GPIO_Pin_15)//GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_SET)


#define RCLK_L    GPIO_ResetBits(GPIOB, GPIO_Pin_14)//GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_RESET)
#define RCLK_H    GPIO_SetBits(GPIOB, GPIO_Pin_14)//GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET)

#define SER_L     GPIO_ResetBits(GPIOB, GPIO_Pin_13)//GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_RESET)
#define SER_H     GPIO_SetBits(GPIOB, GPIO_Pin_13)//GPIO_WriteBit(GPIOB,GPIO_Pin_13,Bit_SET)


GPIO_InitTypeDef srclk;
GPIO_InitTypeDef rclk;
GPIO_InitTypeDef ser;




/*******************************************
                         bit0
                   ---------
                 /        /
            bit5/        /bit1
               /________/        
          bit4/  bit6  /
             /        /bit2
            /________/
               bit3      . bit7
    0: led off
    1: led on


********************************************/





                  //              0       1    2    3      4     5     6      7     8    9 
unsigned char const segmcode[]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,
						// A    B     C     d     E     F     V     r      -     H    N
						0x77,0x7C,0x39,0x5E,0x79,0x71,0x3e,0x50,0x40,0x76,0x54,0x02};
unsigned char const bitcode[]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};




u8  dis_char(char a)
{
	u8 data = 0;
	
	if( (a >= '0') && (a <= '9') )
	{
		data =  segmcode[a-'0'];
	}
	else
	{
		switch(a)
		{
			case 'A':
				data = segmcode[10];
				break;
			case 'B':
				data = segmcode[11];
				break;
			case 'C':
				data = segmcode[12];
				break;
			case 'd':
				data = segmcode[13];
				break;
			case 'E':
				data = segmcode[14];
				break;
			case 'F':
				data = segmcode[15];
				break;
			case 'V':
				data = segmcode[16];
				break;
			case 'r':
				data = segmcode[17];
				break;
			case '-':
				data = segmcode[18];
				break;
			case 'H':
				data = segmcode[19];
				break;
			case 'N':
				data = segmcode[20];
				break;
			case ' ':
				data = segmcode[0];
				break;
			case 'c'://C的点
				data = segmcode[21];
				break;
			default:
				data = 0;
				break;
		}
	}
	return data;
}

//??????????????:

void DTDisplayChar(unsigned char segmd,unsigned char bitd )//???????????

{
        unsigned char i;
        unsigned int dat;

        dat=bitd; 
        dat=dat<<8|segmd; 
        for(i=0;i<16;i++)
 
        {
             if(dat&0x8000)
             {
             	SER_H;
             }
			 else
             {
             	SER_L;
             }
             SRCLK_H; 
             SRCLK_L;
             dat<<=1;
        }
        RCLK_H;
        RCLK_L;  
}



void close_display(void)
{
	DTDisplayChar(0x00,0xff);
}

void all_display(void)
{
	//u8 i = 0;
	
	//for(i=0;i<8;i++)
	DTDisplayChar(segmcode[8]|0x80,0x00);
}




void display(DISPLAY_DATA *pSegLed)
{
	u32 ThrTime[8];
	static u32 tick[8] = {0,0,0,0,0,0,0,0};
	static u8  FastFlashStat[8];
	static u8  SlowFlashStat[8];
	u32 FastFlashTick = 0;
	u32 SlowFlashTick = 0;
	//display total time  per cycle
	u32 TTime = LED_FLASH_FREQ;
	static u8 i = 0;
	u8 j =0;
	static u8 cnt = 0;

	//close display
	if(NULL == pSegLed)
	{
		for(j=0; j<8; j++)
		{
			DTDisplayChar(0x00,bitcode[j]);
			FastFlashStat[j] = 0;
			SlowFlashStat[j] = 0;
		}
		i = 0;
		cnt = 0;
		return ;
	}
	
	
	cnt++;
	if(cnt < 6)
	{
		return ;
	}
	cnt = 0;



	FastFlashStat[i] = 0;
	SlowFlashStat[i] = 0;
	
	
	if(pSegLed->SegLedStat[i] == LED_FLASH_FAST)
	{
		TTime = LED_FLASH_FREQ>>1;
		FastFlashStat[i] = 1;
#if 1
		if(pSegLed->FlashCnt[i]&&(DISPLAY_ON != pSegLed->FlashCnt[i]))
		{
			pSegLed->FlashCnt[i]--;
		}
		if(0 == pSegLed->FlashCnt[i])
		{
			pSegLed->SegLedStat[i] = LED_ON;
		}
#endif
	}
	else if(pSegLed->SegLedStat[i] == LED_FLASH_SLOW)
	{
		TTime = LED_FLASH_FREQ<<1;
		SlowFlashStat[i] = 1;
	}
	
	switch(pSegLed->SegLedStat[i])
	{
		case LED_OFF:
			ThrTime[i] = 0;
			break;
		case LED_ON:
			ThrTime[i] = TTime;
			break;
		case LED_FLASH_SLOW:
			ThrTime[i] = TTime>>1;
			break;
		case LED_FLASH_FAST:
			ThrTime[i] = TTime>>1;
			break;
		default:
			ThrTime[i] = 0;
			break;
	}
	if(0 == tick[i])
	{
		tick[i] = TTime;
	}
	if(--tick[i] >= ThrTime[i])
	{
		//pSegLed->data[i] = 0x00;
		DTDisplayChar(0x00,bitcode[i]);
	}
	else
	{
		DTDisplayChar(pSegLed->data[i],bitcode[i]);
	}
	i++;
	//闪显步调要一致
	if(i>=8)
	{
		i = 0;
		for(j=0; j <8;j++)
		{
			if(SlowFlashStat[j])
			{
				if(SlowFlashTick)
					tick[j] = SlowFlashTick;
				else
					SlowFlashTick = tick[j];
			}
			if(FastFlashStat[j])
			{
				if(FastFlashTick)
					tick[j] = FastFlashTick;
				else
					FastFlashTick = tick[j];
			}
		}
	}
	

}

/********************************************************************************
*fun name:          set_err_display_data 
*fun description:   set err display data when err happen 
*input param:       pSegLed: set display data                               
*output param:
*return value:  
*remark:        creat 2016-12-25  guolei
********************************************************************************/
#if 0
void set_err_display_data(u8 ErrCode,DISPLAY_DATA *pSegLed)
{
	if(pSegLed)
	{
		if(ErrCode > 15)
		{
			pSegLed->data[0] = segmcode[0];
		}
		else
		{
			pSegLed->data[0] = segmcode[ErrCode];
		}
		pSegLed->SegLedStat[0] = LED_ON;
		pSegLed->data[1] = segmcode[14];
		pSegLed->data[2] = segmcode[17];
		pSegLed->data[3] = segmcode[17];
		pSegLed->data[4] = segmcode[0];
		pSegLed->SegLedStat[4] = LED_OFF;
		pSegLed->data[5] = segmcode[14];
		pSegLed->data[6] = segmcode[17];
		pSegLed->data[7] = segmcode[17];
	}
}
#endif

void SegLedInit(void)
{

	
	srclk.GPIO_Pin = GPIO_Pin_15; 
	srclk.GPIO_Mode = GPIO_Mode_Out_PP;
	srclk.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &srclk);
	
	
	rclk.GPIO_Pin = GPIO_Pin_14; 
	rclk.GPIO_Mode = GPIO_Mode_Out_PP;
	rclk.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &rclk);
	
	ser.GPIO_Pin = GPIO_Pin_13; 
	ser.GPIO_Mode = GPIO_Mode_Out_PP;
	ser.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &ser);
	
	
}
