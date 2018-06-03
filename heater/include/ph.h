#ifndef __PH_H__
#define  __PH_H__


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
#include "key.h"
#include "pwm.h"

#define   PARAM_VALID   0xfaceecaf
#define   MIN_INTER_TEMP      55//85
#define   MAX_INTER_TEMP      85//85
#define   INER_TEMP_ERR_CNT   5

#define   POWER_INPUT_PWM_FREQKHZ_MIN    80
#define   POWER_INPUT_PWM_FREQKHZ_MAX    210//140
#define   HEATER_INPUT_PWM_FREQKHZ_MIN    210//200
#define   HEATER_INPUT_PWM_FREQKHZ_MAX    360

#define   PWM_ERR_CNT           10



//power err
#define   ERR_OVER_MAX_VOL      1
#define   ERR_OVER_MAX_CUR      2
#define   ERR_OVER_IN_TEMP      2



#define  ADUST_PWMU             0
#define  ADUST_PWMI             1


#define  KEY_INACTIVE        0
#define  KEY_ACTIVE          1



typedef enum {
	PH_OK,
	PH_ERR_NULL,
	PH_FLASH_ERR,
}PH_ERR;

typedef enum {
	PWM_FREQ_NO_ERR,
	PWM_FREQ_OVER,
}PWM_FREQ_ERR;


typedef struct SYS_STAT_STR{
	
	u32 KnobTurnTime;
	u32 ticks;
	volatile	u32 Tmr2CntMs;
	u32 PwrOffTmrStmp;
	u32 PwmFreq;//kHz
	u32 KeyActMs;//unit:ms
	u8 KnobTurn;
	u8 AdjustPwmType;
	s8 InTemp;
	PWM_FREQ_ERR PwmFreqErr;
	u8 KeyAct;
	HEATER_STAT  *pHtrS;
	POWER_STAT *pPwrStat;
}SYS_STAT_PARA;



#pragma pack (4) 
typedef struct PWR_HOT_STR{
	u16 sum;
	u8 wm ;//work mode
	POWER pwr;
	HEATER htr;
	u32 vld;//flash parm is valid ???
}PWR_HTR;



typedef struct ADJUST_PARAM{
	
	HEATER_ADJ htrJ;
	POWER_ADJUST pwrJ;
	u32 vld;//flash parm is valid ???
	u16 sum;
}ADJUST_VALUE;
#pragma pack () 

extern SYS_STAT_PARA SysStat;
extern PWR_HTR ph;
extern ADJUST_VALUE adjustDat;




/********************************************************************************
*fun name:          judge_pwm_err 
*fun description:   judge input pwm is err
*input param:       pPwrHtr 
*output param:
*return value:  
*remark:        creat 2016-12-25  guolei
********************************************************************************/
extern void judge_pwm_err(PWR_HTR *pPwrHtr);

/********************************************************************************
*fun name:          calc_cpu_temp 
*fun description:   calc cpu temp
*input param:       pPwrHtr 
*                   ADC1_channel16 ADC value
*output param:
*return value:  
*remark:        creat 2016-12-24  guolei
********************************************************************************/
extern void calc_cpu_temp(PWR_HTR *pPwrHtr ,u16 *pInAdValue);
/********************************************************************************
*fun name:          data_default 
*fun description:   set system param default 
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
extern void data_default(void);





/********************************************************************************
*fun name:          read_adjust_data 
*fun description:   read adjust param
*input param:       pAdjustDat  
*output param:
*return value:  
*remark:        creat 2017-07-18  guolei
********************************************************************************/
extern void read_adjust_data(ADJUST_VALUE *pAdjustDat);


/********************************************************************************
*fun name:          write_adjust_data 
*fun description:   write adjust param
*input param:       pAdjustDat  
*output param:
*return value:  
*remark:        creat 2017-07-18  guolei
********************************************************************************/
extern PH_ERR write_adjust_data(ADJUST_VALUE *pAdjustDat);


/********************************************************************************
*fun name:          write_sys_data 
*fun description:   write system work param
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
extern PH_ERR write_sys_data(PWR_HTR *pSysDat);



/********************************************************************************
*fun name:          read_sys_data 
*fun description:   read system work param
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
extern PH_ERR read_sys_data(PWR_HTR *pSysDat);


/********************************************************************************
*fun name:          cmd_explain 
*fun description:   deal witch cmd
*input param:       cmd
*output param:
*return value:  
*remark:        creat 2016-12-13  guolei
********************************************************************************/
extern void cmd_explain(u16 cmd);


/********************************************************************************
*fun name:          read_adjust_data_valid 
*fun description:   read adjust value
*input param:       no
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
extern PH_ERR read_adjust_data_valid(void );

/********************************************************************************
*fun name:          read_cur_sys_data_valid 
*fun description:   read current system work param
*input param:       no
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
extern PH_ERR read_cur_sys_data_valid(void );

/********************************************************************************
*fun name:          key_inactive 
*fun description:   deal with from key active
*input param:       TimeMs: timer when key inactive
*output param:
*return value:  
*remark:        creat 2017-01-07  guolei
********************************************************************************/
extern void  key_inactive(u32 TimeMs);


/********************************************************************************
*fun name:          stat_init 
*fun description:   init state of system param
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
********************************************************************************/
extern void stat_init(PWR_HTR *pSysDat);



#endif

