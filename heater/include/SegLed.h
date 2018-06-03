#ifndef __SEGLED_H__
#define __SEGLED_H__


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



#define  LED_FLASH_FREQ          200//seg led flash in per 15*34 ms
#define  FLASH_DISP_CNT          500
#define  DISPLAY_ON               0xffffffff

typedef enum{
	LED_OFF = 0,
	LED_ON,
	LED_FLASH_SLOW,
	LED_FLASH_FAST,
}SEG_LED_STATE;

typedef struct DISPLAY_DATA_STR{
	SEG_LED_STATE SegLedStat[8];
	u32 FlashCnt[8];
	u8 data[8];
	u8 UpUnit;// up  row unit
	u8 DownUnit;//down  row unit;
}DISPLAY_DATA;



/********************************************************************************
*fun name:          set_err_display_data 
*fun description:   set err display data when err happen 
*input param:       pSegLed: set display data                               
*output param:
*return value:  
*remark:        creat 2016-12-25  guolei
********************************************************************************/
//extern void set_err_display_data(u8 ErrCode,DISPLAY_DATA *pSegLed);
extern void SegLedInit(void);
//extern int test_seg_led(void);
extern int test_dynamic_seg_led(u8 DisplayData);
//extern void display_data(void);
extern void DTDisplayChar(unsigned char segmd,unsigned char bitd );
extern void all_display(void);
extern void close_display(void);
extern void display(DISPLAY_DATA *pSegLed);
extern u8  dis_char(char a);
extern void Delayms(unsigned int t);

extern unsigned char const segmcode[];
extern unsigned char const bitcode[];

#endif

