#ifndef __POWER_H__
#define __POWER_H__


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



#define CUR_STEP_0_0_1_A    10
#define CUR_STEP_0_1_A      100



#define VOL_STEP_0_1_V      100
#define VOL_STEP_1_V        1000



#define  SET_VOLTAGE_MODE   0
#define  SET_CURRENT_MODE   1

#define  POWER_OFF           0
#define  POWER_ON            1

#define PWMU_DEFAULT_DUTY_60  909UL
#define PWMU_MIN_DUTY_60       850UL
#define  PWM_ACTUAL_60       22220UL
#define  PWMU_DEFAULT_60    ((PWMU_DEFAULT_DUTY_60*PWM_MAX_VALUE)/(1000UL))
#define  PWMU_MIN_60        ((PWMU_MIN_DUTY_60*PWM_MAX_VALUE)/(1000UL))
#define  PWMU_DEFAULT_0_1   ((2*PWM_MAX_VALUE)/(1000UL))
#define  PWMU_MAX_0_1       ((10*PWM_MAX_VALUE)/(1000UL))



#define PWMI_DEFAULT_DUTY_3  545UL
#define PWMI_MIN_DUTY_3      480UL

#define  PWMI_DEFAULT_3    ((PWMI_DEFAULT_DUTY_3*PWM_MAX_VALUE)/(1000UL)) 
#define  PWMI_MIN_3         ((PWMI_MIN_DUTY_3*PWM_MAX_VALUE)/(1000UL))
#define  PWMI_DEFAULT_0_1   ((10*PWM_MAX_VALUE)/(1000UL))
#define  PWMI_MAX_0_1       ((50*PWM_MAX_VALUE)/(1000UL))


#define  SMALL_SCOPE        0
#define  BIG_SCOPE          1






#define  PWM_UPDATE_CNT     2000
#define  PWM_STABLE_CNT     400

#define  DISPLAY_UPDATE_CNT  80000


#define DEFAULT_VOL 5000UL //uint:1mv

#define MAX_VOL     60000UL //unit:1mv
#define VOLT_5V     5000UL//
#define VOLT_0_1V   100UL//
#define VOLT_1V     1000UL//
#define MIN_VOL     100UL//unit:1mv

#define ADJUST_VOL  1000UL
//#define VOL_30V      30000UL
#define VOL_0_1V    100UL



#define DEFAULT_CUR 1000UL//uint:1mA
#define MAX_CUR     3000UL //unit:1mA
#define CUR_1A      1000UL
#define CUR_0_1A    100UL
#define CUR_0_0_1A  10UL

#define MIN_CUR     1UL //unit:1mA

#define KNOB_STOP       0
#define KNOB_TURNING    1
#define KNOB_TURN_TIMER      8000

//#define PWM_ADJUST_TIME      100
#define PWM_I_TYPE            0
#define PWM_U_TYPE            1

#define VOLT_CUR_ERR_CNT     5


#define   INDEX_0_1V      0
#define   INDEX_1V        1
#define   INDEX_60V       2
//#define   INDEX_30V       2


#define   INDEX_0V        3


#define   ADJUST_INDEX    INDEX_1V

#define   INDEX_0_0_1A    0
#define   INDEX_0_1A      1
#define   INDEX_3A         2
//#define   INDEX_1A        2

#define   INDEX_0A        3






//#define TEST_POWER       1



typedef struct PWR_VOLT_STR{
	u16 SetVol;// unit:1mv
	u16 VolStep;//  10: 1v step      1: 0.1v step 
}POWER_VOLT;
typedef struct PWR_ADJUST_STR{
	//s32 Vpwmb;
	//s32 Apwmb;
	u16 VPwmSet[3];
	u16 APwmSet[3];
	u16 VAd[4];
	u16 AAd[4];
}POWER_ADJUST;


typedef enum {
	PWR_NO_ERR,
	PWR_TEMP_BOARD_ERR  = 0x01,
	PWR_PWM_FREQ_ERR    = 0x02,
	PWR_CUR_ERR         = 0x04,
	PWR_VOLT_ERR        = 0x08,
	PWR_FLASH_ERR        = 0x10,
}POWER_ERR;
typedef enum {
	POWER_SLEEPED   = 0,
	POWER_WAKING,
	POWER_WAKEN
}POWER_SLEEP;

typedef enum {
	PWR_HTR_EN_INDEX   = 0,
	PWR_CTR_TIM_INEX,
	OVER_TEMP_INDEX
}POWER_SET_INDEX;


typedef struct PWR_VOLT_STAT_STR{
	u16 CurVol;//current voltage 1mv
	u16 pwmu;//
}POWER_VOLT_STAT;


typedef struct PWR_CUR_STR{
	u16 SetCur;// uint:10mA
	u16 CurStep;//  10 : 0.1A step   1: 0.01A step 
}POWER_CUR;

typedef struct PWR_CUR_STAT_STR{
	u16 CurCur;//current current 1mA
	u16 pwmi;// 
}POWER_CUR_STAT;

typedef struct PWR_SET_MODE{
	u8 setModeOn;
	u8 AdjPage;// 1:adjust page  0: adjust item
	u8 EnTimeCtrPwr;// 0:timer cllose power output  1:timer open power output
	POWER_SET_INDEX ItmIdx;// item index start from  0
	TIMER_TABLE *PwrSetTmr;
	
}PWR_SET;
typedef struct PWR_STAT_STR{
	POWER_VOLT_STAT pvs;
	POWER_CUR_STAT pcs;
	DISPLAY_DATA PwrLed;
	u32 PwmUpdateCnt;
	u32 DisplayUpdateCnt;
	POWER_ERR PwrErr;//B1 freq over  temperature high current and volt high
	POWER_SLEEP PwrSleep;
	u8 PwrOn;// 1:on 0: off
	u8 DisSetValue;
	u32 DisSetValueMs;
	u32 DisSetValueStamp;
	u32 voltTurnStamp;
	u32 curTurnStamp;
	PWR_SET PwrSet;
}POWER_STAT;
typedef struct POWER_STR{
	POWER_VOLT pv;	
	POWER_CUR pc;
	u16 PwrOffTimerM;
	u8 CurSetMode;//SET_VOLTAGE_MODE: current setup voltage valuef
	u8 PwrOn;//open power when power on
	u8 PretecTemp;
}POWER;



//y = Va1x+Vb1  x:volt   y:pwm
extern s32 Kv;
extern s32 Vb1;


//y = Aa1x+Ab1  x:current   y:pwm
extern s32 Ka;
extern s32 Ab1;


extern POWER_STAT PwrStat;
/********************************************************************************
*fun description:   power_adjust_parm_init
*input param:    
*output param:
*return value:
********************************************************************************/
extern void power_adjust_parm_init(void);

extern void power_ctrl(POWER *power,u16 *pVolt);
extern void power_init(POWER *power);
extern void set_power_io(void);
extern void power_display(POWER *power);

/********************************************************************************
*fun name:          calc_pwmiu 
*fun description:   calc current pwmu or pwmi by current setting value 
*input param:       power   power param 
*                   type    PWM_U_TYPE    pwm
*output param:
*return value:  
*remark:        creat 2016-12-07  guolei
********************************************************************************/
extern void calc_pwmiu(POWER *power);

/********************************************************************************
*fun name:          power_stat_init 
*fun description:   init power stat
*input param:       power: power param address
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
********************************************************************************/
extern void power_stat_init(POWER *power);

#endif
