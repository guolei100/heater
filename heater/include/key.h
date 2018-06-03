#ifndef   __KEY_H__
#define    __KEY_H__




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

#include "power.h"
#include "heater.h"
#include "ph.h"


 

#define    CLOSE_DEBUG              1

//#define    ENABLE_WATCH             1

#define    OPEN_POWER_ALARM         1

#define    OPEN_HEATER_ALARM        1
#define    OPEN_HEATER_SLEEEP       1
#define    CLOSE_TEST_HEATER_SLEEP  1
#define    OPEN_ADJUST_TEMP_DISPLAY 1

//#define    OLD_PID                     1
#define    NO_DISYPLAY_AD             1
#define    NO_DISYPLAY_PWMFREQ       1


#ifdef CLOSE_DEBUG

#define KEY_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define PID_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define HINT_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define ADJUST_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define FLASH_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#else
#define KEY_DEBUG(fmt,...)  					//printf(fmt,##__VA_ARGS__)
#define PID_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define HINT_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define ADJUST_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)
#define FLASH_DEBUG(fmt,...)  				//printf(fmt,##__VA_ARGS__)

#endif


#define TEMP_DIS_DEBUG(fmt,...)  //printf(fmt,##__VA_ARGS__)

//还有多少次可以保存系统参数就擦除FLASH
#define  RESERVED_CNT      100




#if      CLOSE_TEST_HEATER_SLEEP
#define  HEATER_HALF_SLEEP_MS     600000
#define  HEATER_SLEEP_MS           600000//1200000
#else
#define  HEATER_HALF_SLEEP_MS     60000
#define  HEATER_SLEEP_MS           180000//60000
#endif

#define KEY1_DOWN         0x01
#define KEY1_LONG         0x04
#define KEY1_UP           0x10

#define KEY2_DOWN         0x02
#define KEY2_LONG         0x08
#define KEY2_UP           0x20
    
#define NOKEY  0xff

#define KEY_WOBBLE_TIME 5000          //去抖动时间(待定)     
#define KEY_OVER_TIME 15000      //等待进入连击时间(待定)，该常数要比正常 //按键时间要长，防止非目的性进入连击模式        
#define KEY_QUICK_TIME 1000   //等待按键抬起的连击时间(待定)                     

#define LED_FLASH_TIME    6000
#define STEP_FLASH_TIME   1600



#define POWER_MODE   0
#define HOT_MODE     1
#define ADJUST_MODE  2


#define MAX_TEMP     450
#define MIN_TEMP     200

#define  HEATER_MODE_CTR    GPIO_ResetBits(GPIOB, GPIO_Pin_1); GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define  POWER_MODE_CTR    GPIO_SetBits(GPIOB, GPIO_Pin_1); GPIO_SetBits(GPIOB, GPIO_Pin_0)

#define POWER_HEATER_OFF    GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define POWER_HEATER_ON     GPIO_SetBits(GPIOB, GPIO_Pin_12)


#define GET_HEATER_SLEEP_IO   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)
#define CUR_OVER_IO()            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)

#define PINA   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
#define PINB   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)


//#define TEST_KNOB          1


#if 0
#define container_of(ptr, type, member) ({          /  
    const typeof( ((type *)0)->member ) *__mptr = (ptr); /  
    (type *)( (char *)__mptr - offsetof(type,member) );})  
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)

#endif


extern u32 key2UpTimeStamp;


extern void EXTI_PA0_Config(void);
extern void EXTI_PA1_Config(void);




extern int exit_pwr_set(void *p);

/********************************************************************************
*fun name:          display_step 
*fun description:   display current step
*input param:       turn     
*                        0:  display current step
*                        1:  switch step and display  step
*output param:
*return value:  
*remark:        creat 2017-09-06  guolei
********************************************************************************/
extern void display_step(u8 turn);

/********************************************************************************
*fun name:          open_heater_handle_irq 
*fun description:   open heater handle irq
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-07-28  guolei
********************************************************************************/
extern void open_heater_handle_irq(void);
/********************************************************************************
*fun name:          pwm_change_io 
*fun description:   change PWM  IO to  GPIO_Mode_IN_FLOATING  
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-07-02  guolei
********************************************************************************/
extern void pwm_change_io(void);
extern void key_init(void);
extern u8 read_key(void);
extern u8 read_key_delay(void);
extern u8 key_scan(void);



/********************************************************************************
*fun name:          compare_timestamp 
*fun description:   now > (timeStamp +ticks) 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2018-04-16  guolei
********************************************************************************/
extern u8 compare_timestamp(u32 timeStamp, u32 ticks);
extern void open_power(void); 
/********************************************************************************
*fun name:          get_ticks 
*fun description:   calc  time(ms)from LastTick to curTick
*input param:   
*            curTick : current time
*            LastTick: last time
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
extern u32 get_ticks(u32 curTick,u32 LastTick);

extern void scan_encoder(void);

extern void short_press_key2(void);
extern  void long_press_key2(void);

//extern PWR_HTR ph;





extern void deal_key(u8 KeyValue);
#endif

