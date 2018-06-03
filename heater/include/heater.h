#ifndef __HEATER_H__
#define __HEATER_H__


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
#include "universal_timer.h"



#define  HEATER_OFF      0
#define  HEATER_ON       1


#define    FULL_TIME_HEAT     200





#define  TEST_SLEEP_TIME       1

//#define  DISPLAY_THRESHOLD    1
#define  THRESHOLD_OFFSET      5






#define  HEATER_TEMP_LOW_TIMER     60000


#define  HEATER_HALF_SLEEP_TEMP   200
#define  HEATER_SET_MAX_TEMP           MAX_TEMP
#define  HEATER_OFFSET_MAX_TEMP        50
#define  HEATER_MIN_TEMP                80

#define  ADJUST_TEMP1             200
#define  ACTUAL_TEMP1_DEFAULT    218
#define  ADJUST_TEMP2             350
#define  ACTUAL_TEMP2_DEFAULT    372

#define  HEATER_FLOAT_TEMP         520    
#define  ALARM_TEMP                 500

#define  HEATER_TEMP_ERR_CNT       5

#define  HTR_NORMAL        0
#define  HTR_ERR           1





typedef enum{
	HEATER_SLEEPED   = 0,
	HEATER_HALF_SLEEP,
	HEATER_WAKING,
	HEATER_WAKEN
}HEATER_SLEEP;

typedef enum{
	HEATING   = 0,//heating now
	COOLING        //heater now close because  temperature over
}HEATING_STAT;
typedef struct HEATER_SET_STR{
	u16 SetTemp;
}HEATER;


typedef enum{
	HEATER_NO_ERR,
	HEATER_TEMP_BOARD_ERR= 0x01,
	HEATER_PWM_FREQ_ERR  = 0x02,
	HEATER_TEMP_OVER_ERR = 0x04,
	HEATER_TEMP_LOW_ERR  = 0x08,
	HEATER_CUR_OVER_ERR  = 0x10,
}HEATER_ERR;

typedef enum{
	HEATER_NORMAL,
	HEATER_ADJUST= 0x01,
}HEATER_MODE;




typedef struct HEATER_STAT_STR{
	u32 CurOvertime;
	u32 heatFloatTime;//加热器拔
	u32 TCurOCnt;
	u32 alarmClock;
	u32 key2Tick;
	u16 CurTemp;
	u16 temGap;
	u16 DispTemp;
	u16 AdTemp;//检测烙铁内部温度
	u16 adActTemp;//设置的加热温度为烙铁表面温度，而此时烙铁内部温度为adActTemp
	u16 setTemp;
	u16 cpySetTemp;
	u8 HtrErr;//B1 freq over and temperature
	u8 HtrOn;// 1: heater working  0: off
	u8 intoSleep;
	u8 key2Cnt;
	u8 heaterFloat;//没有接烙铁关
	HEATING_STAT heating;
	HEATER_SLEEP sleep;
	TIMER_TABLE *htrTmr;
	DISPLAY_DATA HtrLed;
	HEATER_MODE  mode;
	
	

}HEATER_STAT;





typedef struct HEATER_STR{
	u16 adjTemp1;
	u16 adjActTemp1;
	u16 adjTemp2;
	u16 adjActTemp2;
}HEATER_ADJ;




extern HEATER_STAT HeaterStat;
extern HEATING_STAT Heating;

/********************************************************************************
*fun description:   heater param of adjust init
*input param:    
*output param:
*return value:
********************************************************************************/
extern void heater_adjust_parm_init(void);

/********************************************************************************
*fun name:          into_heater 
*fun description:   init heater param when into heater
*input param:       
*                   
*output param:
*return value:  
*remark:        creat 2016-12-27  guolei
********************************************************************************/
extern void into_heater(void);

extern void heater_init(HEATER *pheater);
extern void heater_ctrl(HEATER *pheater,u16 *pAdValue,u8 update);
extern void adjust_heater_temp(s8 temp);


/********************************************************************************
*fun name:          calc_actual_temp 
*fun description:   calc actual temp
*input param:       
*output param:
*return value:  
*remark:        creat 2017-7-7  guolei
********************************************************************************/
extern u16 calc_actual_temp(void);

/********************************************************************************
*fun name:          get_actual_temp 
*fun description:   get actual_temp
*input param:       pheater
*output param:
*return value:  
*remark:        creat 2017-7-7  guolei
********************************************************************************/
extern u16 get_actual_temp(void);
/********************************************************************************
*fun name:          test_heater_alarm 
*fun description:   test heater alarm
*input param:       no
*                   
*output param:
*return value:  
*remark:        creat 2017-7-2  guolei
********************************************************************************/
extern s16 get_heater_temp(u16 *pAdValue);

/********************************************************************************
*fun name:          heater_stat_init 
*fun description:   init heater stat
*input param:       pheater: pheater param address
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
********************************************************************************/
extern void heater_stat_init(HEATER *pheater);

extern u16 HeatTime;
#endif
