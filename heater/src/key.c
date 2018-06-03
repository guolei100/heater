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
#include "key.h"
#include <stdio.h>
#include "system_stm32f10x.h"
#include "heater.h"
#include "power.h"
#include "pwm.h"
#include "ph.h"
#include "timer.h"
#include "ad.h"
#include "pid.h"
#include "universal_key.h"



#define  MAX_CLOSE_TIMER_MIN      9999


extern unsigned char const segmcode[];
extern unsigned char const bitcode[];


u32 key2UpTimeStamp = 0;






//return value: bit0:key1  bit1:key2
u8 read_key(void)  
{          
	static u8 LastKey = NOKEY;         //保存上一次的键值      
	static u16 KeyCount = 0;         //按键延时计数器     
	static u16 KeyOverTime = KEY_OVER_TIME;  //按键抬起时间      
	u8 KeyTemp = NOKEY;             //临时保存读到的键值        

	//KeyTemp = P1 & 0x0f                 //读键值         
	KeyTemp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) |     \
			  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)<<1;

	if( KeyTemp == 0x03 )         
	{              
		KeyCount = 0;              
		KeyOverTime = KEY_OVER_TIME;  

        return NOKEY;             //无键按下返回NOKEY        
	}         
	else         
	{                      

		if( KeyTemp == LastKey )    //是否第一次按下            
		{                                              
			if(  ++KeyCount  ==  KEY_WOBBLE_TIME )    //不是第一次按下，则判断//抖动是否结束  
			{                 
				return (~KeyTemp )&0x03;;                 //去抖动结束，返回键值            
			}             
			else            
			{                           
				if( KeyCount > KeyOverTime )   
				{                
					KeyCount = 0;              
					KeyOverTime = KEY_QUICK_TIME;          
				}                            
				return NOKEY;             
			}
		}                     
		else    //是第一次按下则保存键值，以便下次执行此函数时与读到的键值作比较     
		{             
			LastKey = KeyTemp;             //保存第一次读到的键值         
			KeyCount = 0;                 //延时计数器清零           
			KeyOverTime = KEY_OVER_TIME;            
			return NOKEY;                     
		} 
    }    
} 


#if 0

//return value: bit0:key1  bit1:key2
uint8_t read_key(void)  
{          
	static uint8_t LastKey = NOKEY;         //保存上一次的键值      
	static uint16_t KeyCount = 0;         //按键延时计数器     
	static uint16_t KeyOverTime = KEY_OVER_TIME;  //按键抬起时间      
	uint8_t KeyTemp = NOKEY;             //临时保存读到的键值        

	//KeyTemp = P1 & 0x0f                 //读键值         
	KeyTemp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) |     \
			  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)<<1;

	if( KeyTemp == 0x03 )         
	{              
		KeyCount = 0;              
		KeyOverTime = KEY_OVER_TIME;  

        return NOKEY;             //无键按下返回NOKEY        
	}         
	else         
	{                      

		if( KeyTemp == LastKey )    //是否第一次按下            
		{                                              
			if(  ++KeyCount  ==  KEY_WOBBLE_TIME )    //不是第一次按下，则判断//抖动是否结束  
			{                 
				return (~KeyTemp )&0x03;;                 //去抖动结束，返回键值            
			}             
			else            
			{                           
				if( KeyCount > KeyOverTime )   
				{                
					KeyCount = 0;              
					KeyOverTime = KEY_QUICK_TIME;          
				}                            
				return NOKEY;             
			}
		}                     
		else    //是第一次按下则保存键值，以便下次执行此函数时与读到的键值作比较     
		{             
			LastKey = KeyTemp;             //保存第一次读到的键值         
			KeyCount = 0;                 //延时计数器清零           
			KeyOverTime = KEY_OVER_TIME;            
			return NOKEY;                     
		} 
    }    
} 

#endif


#if 0


static u8 Trg = 0;
static u8 holded = 0;
void NewKeyRead( void )

{
	//PA3 KEY1
    u8 TKeyValue = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) |     \
			                  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)<<1;     
	TKeyValue |= 0xfc; 
	TKeyValue ^= 0xff;
    Trg = TKeyValue & (TKeyValue ^ holded);      // 2
    holded = TKeyValue;                         // 3
}

#endif


#define  CNT_20MS          20
#define  LONG_PRESS_CNT   20000
#define  SHORT_PRESS_CNT  80



/********************************************************************************
*fun name:          key_scan 
*fun description:   scan key 
*input param:                                     
*output param:
*return value:  
*                          NOKEY:    no key press
*                          bit0~bit1: short press  
*                          bit2~bit3: long press 
*                          bit4~bit5:key up
*                          bit0  bit2  bit4:key1
*                          bit1  bit3  bit5:key2
*remark:        creat 2016-12-07  guolei
********************************************************************************/

u8 key_scan(void)
{
#if 0	
	static u32 cnt = 0;
	static u8 KeyValue = NOKEY;
	u8 key;
#endif


	return read_universal_key();
	
	
#if 0	
	if(0 == Trg)
	{
		NewKeyRead();
		if(Trg)
			cnt = CNT_20MS;//key firs press
			
		if(holded)
		{
			if(cnt++ == LONG_PRESS_CNT)
			{
			
				KEY_DEBUG("key long press\r\n");	
				key = (KeyValue<<2);
				return key;// long press
			}
			else if(cnt == SHORT_PRESS_CNT)
			{
				KEY_DEBUG("key press\r\n");			
				return KeyValue;// short press
			}
		}
	}
	else
	{
		if( 0== --cnt)
		{
			NewKeyRead();
			if(holded)
			{
				KeyValue = holded;
			}
		}
	}
// key up
	if(NOKEY != KeyValue)
	{
		if(0==holded)
		{
			//if(cnt > LONG_PRESS_CNT)
			{
				//
				//KeyValue = (KeyValue<<4) | KeyValue;//key up
				key = (KeyValue<<4);//key up
			}			
			KEY_DEBUG("key up\r\n");
			cnt = 0;
			KeyValue = NOKEY;
			return key;
			//return NOKEY;
		}
	}
	return NOKEY;
#endif	
}



#if 0
//return value: bit0:key1  bit1:key2
u8 read_key_delay(void)  
{          
	//static uint8_t LastKey = NOKEY;         //保存上一次的键值      
	//static uint16_t KeyCount = 0;         //按键延时计数器     
	//static uint16_t KeyOverTime = KEY_OVER_TIME;  //按键抬起时间      
	uint8_t KeyTemp = NOKEY;             //临时保存读到的键值        

	//KeyTemp = P1 & 0x0f                 //读键值         
	KeyTemp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) |     \
			  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)<<1;

	if(0x03 == KeyTemp)
	{
		return NOKEY;
	}
	else
	{
		Delayms(1000);
		KeyTemp = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) |     \
			  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)<<1;
		return (~KeyTemp )&0x03;
	}
	    
} 

#endif

void key_active(void)
{
	if(HOT_MODE ==ph.wm)
    {
		if( (HEATER_HALF_SLEEP ==SysStat.pHtrS->sleep) || \
			(HEATER_SLEEPED ==SysStat.pHtrS->sleep) )
		{
			if(HEATER_ON == SysStat.pHtrS->HtrOn)
			{
				SysStat.pHtrS->heating = HEATING;
			}
			SysStat.pHtrS->sleep = HEATER_WAKING;
			ph.htr.SetTemp = SysStat.pHtrS->setTemp;
		}
 		
    }
	SysStat.KeyAct = KEY_ACTIVE;
	SysStat.KeyActMs = SysStat.Tmr2CntMs;
	reset_timer(SysStat.pPwrStat->PwrSet.PwrSetTmr);
	set_timer_time(SysStat.pHtrS->htrTmr,HEATER_HALF_SLEEP_MS);
	reset_timer(SysStat.pHtrS->htrTmr);
	start_timer(SysStat.pHtrS->htrTmr);
	
	
}


//static u8 decStep = 1;
//static u8 addStep = 1;

static u8 calc_step_speed(void)
{
	static u32 lastTick = 0;
	u8 step = 1;

	if(compare_timestamp(lastTick,150) )
	{
		step = 1;
		
	}
	else if(compare_timestamp(lastTick,80) )
	{
		step = 2;
	}
	else if(compare_timestamp(lastTick, 60) )
	{
		step = 4;
	}
	else if(compare_timestamp(lastTick,30) )
	{
		step = 5;
	}
	else 
	{
		step = 6;
	}
	lastTick = SysStat.Tmr2CntMs;
	return step;
}





/********************************************************************************
*fun name:          knob_dec 
*fun description:   knob dec 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-07  guolei
********************************************************************************/
void knob_dec(PWR_HTR *pPwrHtr)
{
	u16 step = 1;
	
	if(HOT_MODE == pPwrHtr->wm)
	{
		//addStep = 1;
		if( pPwrHtr->htr.SetTemp>MIN_TEMP )
		{
			step = calc_step_speed();
			
			pPwrHtr->htr.SetTemp -= step;
			if(pPwrHtr->htr.SetTemp <= MIN_TEMP)
			{
				pPwrHtr->htr.SetTemp = MIN_TEMP;
			}
			SysStat.pHtrS->adActTemp = get_actual_temp();
		}
		//heater_display(&PowerHeater.heater);
	}
	else if(POWER_MODE == pPwrHtr->wm)
	{
		step = calc_step_speed();
		if(0==SysStat.pPwrStat->PwrSet.setModeOn)
		{
			if(SET_VOLTAGE_MODE == ph.pwr.CurSetMode)
			{
				if(1==step)
				{
					step = 100;
				}
				else
				{
					step = step*200;
				}
				//if( pPwrHtr->pwr.pv.SetVol >= pPwrHtr->pwr.pv.VolStep )
				if( pPwrHtr->pwr.pv.SetVol >= step )
				{
					//pPwrHtr->pwr.pv.SetVol -= pPwrHtr->pwr.pv.VolStep;
					pPwrHtr->pwr.pv.SetVol -= step;
				}
				else
				{
					pPwrHtr->pwr.pv.SetVol = 0;
				}
				//calc_pwmiu(&pPwrHtr->pwr,PWM_U_TYPE);
				
			}
			else
			{
				if(1==step)
				{
					step = 10;
				}
				else
				{
					step = step*20;
				}
				//if( pPwrHtr->pwr.pc.SetCur >= pPwrHtr->pwr.pc.CurStep )
				if( pPwrHtr->pwr.pc.SetCur >= step )
				{
					//pPwrHtr->pwr.pc.SetCur -= pPwrHtr->pwr.pc.CurStep;
					pPwrHtr->pwr.pc.SetCur -= step;
				}
				else
				{
					pPwrHtr->pwr.pc.SetCur = 0;
				}
				//calc_pwmiu(&pPwrHtr->pwr,PWM_I_TYPE);
			}
		}
		else
		{
			if(0 == SysStat.pPwrStat->PwrSet.AdjPage)
			{
				switch(SysStat.pPwrStat->PwrSet.ItmIdx)
				{
					case PWR_HTR_EN_INDEX:
						ph.pwr.PwrOn = 0;
						break;
					case PWR_CTR_TIM_INEX:
						step *= step;
			#if 0			
						if(1==step)
						{
							step = 1;
						}
						else if(2==step)
						{
							step = step*2;
						}
						else if(3==step)
						{
							step = step*5;
						}
						else if(4==step)
						{
							step = step*7;
						}
						else if(5==step)
						{
							step = step*8;
						}
						else if(6==step)
						{
							step = step*10;
						}
		#endif
						
						if(ph.pwr.PwrOffTimerM <= step)
						{
							//ph.pwr.PwrOffTimerM为0取消定时功能
							SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 0;
							ph.pwr.PwrOffTimerM = 0;
						}
						else
						{
							ph.pwr.PwrOffTimerM -= step;
						}
						SysStat.PwrOffTmrStmp = SysStat.Tmr2CntMs;
						break;
					case OVER_TEMP_INDEX:
						if(ph.pwr.PretecTemp > MIN_INTER_TEMP)
						{
							ph.pwr.PretecTemp--;
						}
						break;
				}
			}
			else
			{				
				if(SysStat.pPwrStat->PwrSet.ItmIdx > 0)
				{
					SysStat.pPwrStat->PwrSet.ItmIdx--;
				}
			}
		}
	
		//power_display(&PowerHeater.power);
	}
	else if(ADJUST_MODE == pPwrHtr->wm)
	{
		if(ADUST_PWMU == SysStat.AdjustPwmType)
		{
			
			if(SysStat.pPwrStat->pvs.pwmu)
				SysStat.pPwrStat->pvs.pwmu--;
			pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
			ADJUST_DEBUG("pwmu:%d\r\n",SysStat.pPwrStat->pvs.pwmu);
		}
		else
		{
			if(SysStat.pPwrStat->pcs.pwmi)
				SysStat.pPwrStat->pcs.pwmi--;
			pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
			ADJUST_DEBUG("pwmi:%d\r\n",SysStat.pPwrStat->pcs.pwmi);
		}
	}
}



/********************************************************************************
*fun name:          knob_add 
*fun description:   knob add 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-07  guolei
********************************************************************************/
void knob_add(PWR_HTR *pPwrHtr)
{
	u16 step;
	
	if(NULL == pPwrHtr) return ;

	if(HOT_MODE == pPwrHtr->wm)
	{
		//decStep = 1;
		if( pPwrHtr->htr.SetTemp < MAX_TEMP )
		{
			step = calc_step_speed();
			
			pPwrHtr->htr.SetTemp += step;
			if(pPwrHtr->htr.SetTemp >= MAX_TEMP)
			{
				pPwrHtr->htr.SetTemp = MAX_TEMP;
			}
			
			SysStat.pHtrS->adActTemp = get_actual_temp();
		}
		//heater_display(&PowerHeater.heater);
	}
	else if(POWER_MODE == pPwrHtr->wm)
	{
		step = calc_step_speed();

		if(0==SysStat.pPwrStat->PwrSet.setModeOn)
		{
			if(SET_VOLTAGE_MODE == ph.pwr.CurSetMode)
			{
				//if( (pPwrHtr->pwr.pv.SetVol+pPwrHtr->pwr.pv.VolStep) <= MAX_VOL)
				if(1==step)
				{
					step = 100;
				}
				else
				{
					step = step*200;
				}
				if((pPwrHtr->pwr.pv.SetVol+step) <= MAX_VOL)
				{
					//pPwrHtr->pwr.pv.SetVol+=pPwrHtr->pwr.pv.VolStep;
					pPwrHtr->pwr.pv.SetVol += step;
				}
				else
				{
					pPwrHtr->pwr.pv.SetVol = MAX_VOL;
				}
			
				SysStat.pPwrStat->voltTurnStamp = SysStat.Tmr2CntMs;
			}
			else
			{
				
				if(1==step)
				{
					step = 10;
				}
				else
				{
					step = step*20;
				}
				//if( (pPwrHtr->pwr.pc.SetCur+pPwrHtr->pwr.pc.CurStep) <= MAX_CUR)
				if((pPwrHtr->pwr.pc.SetCur+step) <= MAX_CUR)
				{
					//pPwrHtr->pwr.pc.SetCur+=pPwrHtr->pwr.pc.CurStep;
					pPwrHtr->pwr.pc.SetCur += step;
				}
				else
				{
					pPwrHtr->pwr.pc.SetCur = MAX_CUR;
				}
				SysStat.pPwrStat->curTurnStamp= SysStat.Tmr2CntMs;
			}
		}
		else
		{
			if(0 == SysStat.pPwrStat->PwrSet.AdjPage)
			{
				switch(SysStat.pPwrStat->PwrSet.ItmIdx)
				{
					case PWR_HTR_EN_INDEX:
						ph.pwr.PwrOn = 1;
						break;
					case PWR_CTR_TIM_INEX:
					#if 0	
						if(1==step)
						{
							step = 1;
						}
						else
						{
							step = step*10;
						}
					#endif
						step *= step;
						ph.pwr.PwrOffTimerM += step;
						if(ph.pwr.PwrOffTimerM > MAX_CLOSE_TIMER_MIN)
						{
							ph.pwr.PwrOffTimerM = MAX_CLOSE_TIMER_MIN;
						}
						SysStat.PwrOffTmrStmp = SysStat.Tmr2CntMs;
						break;
					case OVER_TEMP_INDEX:
						if(ph.pwr.PretecTemp < MAX_INTER_TEMP)
						{
							ph.pwr.PretecTemp++;
						}
						break;
					default:
						break;
				}
			}
			else
			{
				SysStat.pPwrStat->PwrSet.ItmIdx++;
				if(SysStat.pPwrStat->PwrSet.ItmIdx >= OVER_TEMP_INDEX)
				{
					SysStat.pPwrStat->PwrSet.ItmIdx = OVER_TEMP_INDEX;
				}
			}
		}
		
	}
	else if(ADJUST_MODE == pPwrHtr->wm)
	{
		if(ADUST_PWMU == SysStat.AdjustPwmType)
		{
			
			if(SysStat.pPwrStat->pvs.pwmu <= MAX_PWM_U)
				SysStat.pPwrStat->pvs.pwmu++;
			pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
			ADJUST_DEBUG("pwmu:%d\r\n",SysStat.pPwrStat->pvs.pwmu);
		}
		else
		{
			if(SysStat.pPwrStat->pcs.pwmi <= MAX_PWM_I)
				SysStat.pPwrStat->pcs.pwmi++;
			pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
			ADJUST_DEBUG("pwmi:%d\r\n",SysStat.pPwrStat->pcs.pwmi);
		}
	}
	
}

/********************************************************************************
*fun name:          knob_turn 
*fun description:   wake heat when knob turning 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-04-27  guolei
********************************************************************************/
void knob_turn(void)
{
	
	key_active();
	SysStat.KnobTurn = KNOB_TURNING;
	SysStat.KnobTurnTime = 0;
}

u8  pinAStat = 8;
u8  pinBStat = 8;
/********************************************************************************
*fun name:          scan_encoder 
*fun description:   detect coded key 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-04-27  guolei
********************************************************************************/
void scan_encoder(void)
{
	static u8 LastPinB;
	static u8 CurPinB;
	static u8 update = 0;
	static u8 stat =0;

	if(0 == stat)
	{
		pinAStat = PINA;
		pinBStat = PINB;
		if(PINA && PINB)
		{
			update = 0;
			stat = 0;
			return ;
		}
		stat = 1;
		LastPinB = PINB;
	}
	else if(1 == stat)
	{
		if(!PINA)
		{
			CurPinB = PINB;
			update = 1;
		}
		else
		{
			stat = 2;
		}
	}
	else if(2 == stat)
	{
		if(update)
		{
			update = 0;
			knob_turn();
			if((0==LastPinB)&&(1==CurPinB))
			{
				knob_add(&ph);//add
			}
			else if((1==LastPinB)&&(0==CurPinB))
			{
				knob_dec(&ph);//dec
			}
		}
		stat = 0;
	}
}





/********************************************************************************
*fun name:          EXTI0_IRQHandler 
*fun description:   detecting  knob of running  
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-07  guolei
********************************************************************************/
#define  INTERVAL    50
void EXTI0_IRQHandler(void)
{
	//static u32 t1 = 0;
	//static u8 LastStat = 0;
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
#if 0		
		if(SysStat.Tmr2CntMs > t1)
		{
			if(SysStat.Tmr2CntMs - t1 <= INTERVAL)
			{
				t1 = SysStat.Tmr2CntMs;
				EXTI_ClearITPendingBit(EXTI_Line0);
				return ;
			}
		}
#endif		
		if(HOT_MODE ==ph.wm)
	    {
			if( (HEATER_HALF_SLEEP ==SysStat.pHtrS->sleep) || \
				(HEATER_SLEEPED ==SysStat.pHtrS->sleep) )
			{
				SysStat.pHtrS->sleep = HEATER_WAKING;
				EXTI_ClearITPendingBit(EXTI_Line0);
				return ;
			}
	 		SysStat.pHtrS->sleep = HEATER_WAKING;
	    }
		key_active();
		SysStat.KnobTurn = KNOB_TURNING;
		SysStat.KnobTurnTime = 0;
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
		{
			knob_add(&ph);
		}
		else
		{
			//KEY_DEBUG(":%d?",SysStat.Tmr2CntMs-t1);
			knob_dec(&ph);
		}
		
		//t1 = SysStat.Tmr2CntMs;
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	
}


void EXTI1_IRQHandler(void)
{
	static u8 Pb1Cnt = 0;
	static u16 TimCount = 0;
	static u16 LastTimCount = 0;
	
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		//EXTI_ClearITPendingBit(EXTI_Line1);
	
		TimCount = TIM_GetCounter(TIM2);
		if(0 == Pb1Cnt)
		{
			Pb1Cnt = 1;
			LastTimCount = TimCount;
		}
		else
		{
			if(TimCount<LastTimCount)
			{
				TimCount = TIM2_PERIOD - LastTimCount + TimCount;
			}
			else
			{
				TimCount -= LastTimCount;
			}

			SysStat.PwmFreq =  (TIM2_PERIOD)/TimCount;
			Pb1Cnt = 0;	
			EXTI->IMR&=~(1<<1);// mask pb1 irq	 
		}
		
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
	
}

void EXTI3_IRQHandler(void)
{
	if(HOT_MODE == ph.wm)
	{
		if(0 == SysStat.pHtrS->TCurOCnt)
		{
			SysStat.pHtrS->alarmClock = SysStat.Tmr2CntMs;
		}
		SysStat.pHtrS->TCurOCnt++;
		SysStat.pHtrS->CurOvertime = SysStat.Tmr2CntMs;
		POWER_HEATER_OFF;
	}
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
	
}
void EXTI9_5_IRQHandler(void)
{ 
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)	  //判别
	{
		KEY_DEBUG(" EXTI_Line5\r\n");
		EXTI_ClearITPendingBit(EXTI_Line5);	 //清除中断
	}
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)	  //判别
	{
		KEY_DEBUG(" EXTI_Line6\r\n");
		EXTI_ClearITPendingBit(EXTI_Line6);	 //清除中断
	                         
	} 
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)	  //判别
	{
		KEY_DEBUG(" EXTI_Line7\r\n");
		EXTI_ClearITPendingBit(EXTI_Line7);	 //清除中断
	}
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)	  //判别
	{
		KEY_DEBUG(" EXTI_Line8\r\n");		
		into_heater();
		EXTI_ClearITPendingBit(EXTI_Line8);	 //清除中断
	                         
	} 
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)	  //判别
	{
		KEY_DEBUG(" EXTI_Line9\r\n");
		EXTI_ClearITPendingBit(EXTI_Line9);	 //清除中断
	}
  
}
void EXTI15_10_IRQHandler(void)
{
	
	if(EXTI_GetITStatus(EXTI_Line10) != RESET) 
	{ 
		KEY_DEBUG(" EXTI_Line10\r\n");
		EXTI_ClearITPendingBit(EXTI_Line10); 
	} 
	
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) 
	{ 
		KEY_DEBUG(" EXTI_Line11\r\n");
		EXTI_ClearITPendingBit(EXTI_Line11);
	} 
	 
	if(EXTI_GetITStatus(EXTI_Line12) != RESET) 
	{ 
		KEY_DEBUG(" EXTI_Line12\r\n");
		EXTI_ClearITPendingBit(EXTI_Line12); 
	} 
	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) 
	{ 
		KEY_DEBUG(" EXTI_Line13\r\n");
		EXTI_ClearITPendingBit(EXTI_Line13);
	} 
	 
	if(EXTI_GetITStatus(EXTI_Line14) != RESET) 
	{ 
		KEY_DEBUG(" EXTI_Line14\r\n");
		EXTI_ClearITPendingBit(EXTI_Line14); 
	} 
	
	if(EXTI_GetITStatus(EXTI_Line15) != RESET) 
	{ 
		KEY_DEBUG(" EXTI_Line15\r\n");
		EXTI_ClearITPendingBit(EXTI_Line15); 
	} 
	
}






void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);;//设置分组  0 1 2 3 4

	//PB3优先级最高  over current detect
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级 1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级 3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
#if 1
 	//heater tremble
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级 1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级 3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
#endif
	//DMA1 irq for read ADC value
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//lowest priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#if 0
	//calc input pwm freq
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//抢占优先级 1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级 3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
#endif



}

void EXTI_PA0_Config(void)
{
	 EXTI_InitTypeDef EXTI_InitStructure;

 /* config the extiline(PA1) clock and AFIO clock */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

 /* config the NVIC(PA1) */
 NVIC_Configuration();


 /* EXTI line(PA1) mode config */
 GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
 EXTI_InitStructure.EXTI_Line = EXTI_Line0;
 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //falling irq

 EXTI_ClearITPendingBit(EXTI_Line0);
 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 EXTI_Init(&EXTI_InitStructure);

}

//烙铁手柄抖动检测
void EXTI_PB8_Config(void)
{
	 EXTI_InitTypeDef EXTI_InitStructure;

		/* config the extiline(PA1) clock and AFIO clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

		NVIC_Configuration();

		/* EXTI line(PB3) mode config */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
		EXTI_InitStructure.EXTI_Line = EXTI_Line8;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising; //falling irq

		EXTI_ClearITPendingBit(EXTI_Line5);
		EXTI_ClearITPendingBit(EXTI_Line6);
		EXTI_ClearITPendingBit(EXTI_Line7);
		EXTI_ClearITPendingBit(EXTI_Line8);
		EXTI_ClearITPendingBit(EXTI_Line9);

		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

}


void EXTI_PB3_Config(void)
{
	 EXTI_InitTypeDef EXTI_InitStructure;

 /* config the extiline(PA1) clock and AFIO clock */
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

 

 /* EXTI line(PB3) mode config */
 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
 EXTI_InitStructure.EXTI_Line = EXTI_Line3;
 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Rising; //falling irq

 EXTI_ClearITPendingBit(EXTI_Line3);
 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 EXTI_Init(&EXTI_InitStructure);

}

/********************************************************************************
*fun name:          pwm_change_io 
*fun description:   change PWM  IO to  GPIO_Mode_IN_FLOATING  
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-07-02  guolei
********************************************************************************/
void pwm_change_io(void)
{
	GPIO_InitTypeDef pwmIo;
		
    pwmIo.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    pwmIo.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_PP;
    pwmIo.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pwmIo);

	 //GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	 //GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}

void pwmi_change_io(void)
{
	GPIO_InitTypeDef pwmIo;
		
    pwmIo.GPIO_Pin =  GPIO_Pin_7 ;
    pwmIo.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_PP;
    pwmIo.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pwmIo);

	 //GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	 //GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}
void pwmu_change_io(void)
{
	GPIO_InitTypeDef pwmIo;
		
    pwmIo.GPIO_Pin =   GPIO_Pin_6;
    pwmIo.GPIO_Mode = GPIO_Mode_IN_FLOATING;//GPIO_Mode_AF_PP;
    pwmIo.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pwmIo);

	 //GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	 //GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}



/********************************************************************************
*fun name:          compare_timestamp 
*fun description:   now > (timeStamp +ticks) 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2018-04-16  guolei
********************************************************************************/
u8 compare_timestamp(u32 timeStamp, u32 ticks)
{
	u8 timeOut = 0;
	u32 now = SysStat.Tmr2CntMs;

	if(timeStamp <= now)
	{
		if( (now - timeStamp) > ticks )
		{
			timeOut = 1;
		}
	}
	else
	{
		if( (0xffffffff - timeStamp + now) > ticks)	
		{
			timeOut = 1;
		}
	}

	return timeOut;
	
}





void pwm_change_IPD(void)
{
	GPIO_InitTypeDef pwmIo;
		
    pwmIo.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    pwmIo.GPIO_Mode = GPIO_Mode_IPD;//GPIO_Mode_AF_PP;
    pwmIo.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pwmIo);

	
}

static u8 PressKey1OFF = 0;
static u8 adChToNo = 0;
u8 StopHeating = 0;

/********************************************************************************
*fun name:          short_press_key1 
*fun description:   deal with short press key1 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
static void short_press_key1(void)
{
	u16 pwmu,pwmi;
	u16 uStep,iStep;
	static u8 htAdjKey2 = 0;
		DISPLAY_DATA dpDat;

	if(POWER_MODE ==ph.wm)
 	{
 		if(POWER_ON == SysStat.pPwrStat->PwrOn)
 		{
			POWER_HEATER_OFF;
			SysStat.pPwrStat->PwrOn = POWER_OFF;
			
			PressKey1OFF = 1;

			pwmu = SysStat.pPwrStat->pvs.pwmu;
			pwmi = SysStat.pPwrStat->pcs.pwmi;
			uStep = (SysStat.pPwrStat->pvs.pwmu >> 8);
			iStep = (SysStat.pPwrStat->pcs.pwmi >> 8);

			if(0 == uStep)uStep = 1;
			while( pwmu > uStep)
			{
				pwmu -= uStep;
				pwm_u_duty(pwmu);
				if(pwmi > iStep)
					pwmi -= iStep;
				pwm_i_duty(pwmi);
				real_delay_10us(20);
				display(pSegLed);
			}
			
			pwm_i_duty(0);
			pwm_u_duty(0);
			pwm_change_io();
 		}
 	}
	else
	{
		if(HEATER_NORMAL == SysStat.pHtrS->mode)
		{
			if((HEATER_ON == SysStat.pHtrS->HtrOn) && (SysStat.pHtrS->sleep > HEATER_HALF_SLEEP))
			{
				POWER_HEATER_OFF;
				SysStat.pHtrS->HtrOn = HEATER_OFF;
				PressKey1OFF = 1;
				StopHeating = 1;
#ifndef    OLD_PID		
			SetMode(0);
#endif		
				pwm_init();
				pwm_i_duty(0);
				pwm_u_duty(0);
				pwm_change_io();
			}
		}
		else
		{
			htAdjKey2++;
			if(1 == htAdjKey2 )
			{
				adjustDat.htrJ.adjTemp1 =ADJUST_TEMP1;
				adjustDat.htrJ.adjActTemp1 = ph.htr.SetTemp;
				ph.htr.SetTemp = ADJUST_TEMP2;
				SysStat.pHtrS->adActTemp = ADJUST_TEMP2;
			}
			else
			{
				adjustDat.htrJ.adjTemp2 =ADJUST_TEMP2;
				adjustDat.htrJ.adjActTemp2 = ph.htr.SetTemp;
				
				ph.htr.SetTemp = SysStat.pHtrS->cpySetTemp;
				SysStat.pHtrS->adActTemp = calc_actual_temp();
				SysStat.pHtrS->mode = HEATER_NORMAL;
				htAdjKey2 = 0;
				adChToNo = 1;
				write_adjust_data(&adjustDat);
				if(PH_OK != read_adjust_data_valid())
				{
					dpDat.data[0] = 0;
					dpDat.data[1] = 0;
					dpDat.data[2] = 0;
					dpDat.data[3] = 0;
					dpDat.data[4] = 0;
					dpDat.data[5] = segmcode[14];
					dpDat.data[6] = segmcode[17];
					dpDat.data[7] = segmcode[17];
					while(1)
						display(&dpDat);
				}
			}
			
		}
	}
}
extern void IWDG_Feed(void);

void flow_data(u8 *data,u8 newDat,u8 cnt)
{
	u8 i;
	if(cnt>7)cnt=7;
	for( i=0; i<cnt;i++)
	{
		data[7-cnt+i] = data[8-cnt+i];
	}
	data[7] = segmcode[newDat];
}

void display_phone_num(u8 io)
{
	int i = 0,j;	
	DISPLAY_DATA flow;
	//u8 num[11] = {1,8,0,3,3,0,6,3,1,8,1};
	u8 num;
	
	
	for( i=0; i<10; i++)
	{
		if(i<8)
		{
			flow.data[i] = 0x00;
			flow.SegLedStat[i] = LED_OFF;
		}
		display(NULL);//close display
	}
	

	
	if(io )	
		pwm_change_IPD();
	
	for( i=0; i<11; i++)
	{
		if(i<8)
		{
			flow.SegLedStat[7-i] = LED_ON;
		}

		num = i;
		if(i<6)
		{
			num=0;
		}
		else if(i>7)
		{
			num = 1;
		}
		else
		{
			num = i;
		}
		
		if(0 == i)num=i+1;
		else if(1 == i)num = i<<3;
		else if(3 == i)num = i;
		else if((4 == i) || (9 == i))num = i-1;
		else if(7 == i)num = i-4;
		
		flow_data(&flow.data[0],num,i);
		for(j=0;j<3500;j++)
		{
			display(&flow);
			real_delay_10us(20);
		}
		
		if(io && (4==i))	
			pwm_change_io();
		
		IWDG_Feed();
	}
	
	
}


/********************************************************************************
*fun name:          long_press_key1 
*fun description:   deal with long press key1 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
static void long_press_key1(void)
{
#if 0	
	int pwmu = 0,pwmi = 0;
	int uStep = 0,iStep = 0;
	int i = 0;
#endif
	


	u16 AV[ADC1_CHANNEL_CNT]; 
	u32 heaterTemp;
	
	SysStat.pPwrStat->PwrOn = POWER_OFF;
	SysStat.pHtrS->HtrOn = HEATER_OFF;
	/*在判断长按前，有可能手抖动按了两次调用了两次short_press_key1，
	导致PressKey1OFF置1此时从电源模式转到加热模式再转到加热模式后需
	要按两次key1才能打开电源，因此对其清0
     20170628
	*/
	PressKey1OFF = 0;
	POWER_HEATER_OFF;
	if(POWER_MODE ==ph.wm)
	{
		get_adc_data(AV);
		heaterTemp = get_heater_temp(&AV[2]);
		//pwm_init();
		//pwm_i_duty(0);
		//pwm_u_duty(0);
		pwm_change_io();
		while(get_adc_data(AV))
		{
			real_delay_10us(20);
			display(pSegLed);
		}
		heaterTemp = get_heater_temp(&AV[2]);
		while(get_adc_data(AV))
		{
			real_delay_10us(20);
			display(pSegLed);
		}
		heaterTemp = (heaterTemp + get_heater_temp(&AV[2])) >> 1;
		SysStat.pHtrS->HtrOn = HEATER_OFF;
		if( heaterTemp > HEATER_FLOAT_TEMP )
		{
			SysStat.pHtrS->HtrErr |= HEATER_TEMP_OVER_ERR;
			SysStat.pHtrS->heaterFloat = 1;
		}
		
		ph.wm = HOT_MODE;
		HEATER_MODE_CTR;
		SysStat.pHtrS->adActTemp = get_actual_temp();
		into_heater();
 		//SysStat.pHtrS->sleep = HEATER_WAKING;
		//update_sleep_stamp();
		
	}
	else
	{
		ph.wm = POWER_MODE;
		POWER_MODE_CTR;
		display_phone_num(1);
		SysStat.pPwrStat->DisSetValue = 1;
		SysStat.pPwrStat->DisSetValueMs = 1000;
		SysStat.pPwrStat->DisSetValueStamp = SysStat.Tmr2CntMs;
		SysStat.pPwrStat->PwrOn = POWER_OFF;
		clear_key_buf();

		display_step(0);
	
		
	}
	
	write_sys_data(&ph);
	//  PwmFreqErr should no err when mode switch 
	SysStat.PwmFreqErr = PWM_FREQ_NO_ERR;
	
}




void open_power(void)
{
	u16 i = 0;
	u16 pwmu,pwmi;
	u16 uStep,iStep;
	
	
	//SysStat.KnobTurnTime = 0;
	SysStat.pPwrStat->PwrOn = POWER_ON;
	
	//current unit don't flash when power open
	SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_ON;
	SysStat.pPwrStat->PwrLed.FlashCnt[7] = 0;

	//pwm_i_duty(0);
	//pwm_u_duty(0);
	POWER_HEATER_ON;
	calc_pwmiu(&ph.pwr);
	pwm_i_init();
	pwmu_change_io();
	enable_pwm();
	iStep = (SysStat.pPwrStat->pcs.pwmi >> 8);
	pwmi = iStep?iStep:1;
	if(0 == iStep)
	{
		iStep = 1;
	}
	while( (pwmi+iStep) <= SysStat.pPwrStat->pcs.pwmi)
	{
		pwmi += iStep;
		
		if(pwmi < SysStat.pPwrStat->pcs.pwmi)
			pwm_i_duty(pwmi);
		real_delay_10us(20);
		display(pSegLed);
	}
	pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);


	for(i=0; i<1000;i++)
	{
		real_delay_10us(20);
		
		display(pSegLed);
	}

	//calc_pwmiu(&ph.pwr);
	pwm_u_init();
	enable_pwm();
	pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
	pwm_u_duty(0);
	//#endif		


	uStep = (SysStat.pPwrStat->pvs.pwmu >> 8);

	pwmu = uStep?uStep:1;
	if(0 == uStep)
	{
		uStep = 1;
	}

	while( (pwmu+uStep) <= SysStat.pPwrStat->pvs.pwmu)
	{
		pwmu += uStep;
		pwm_u_duty(pwmu);
		
		real_delay_10us(20);
		display(pSegLed);
	}
	pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
	pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
}

/********************************************************************************
*fun name:          up_key1 
*fun description:   deal with when  key1  up
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-01-03  guolei
********************************************************************************/
static void up_key1(u8 LastKey)
{
	

	if(1 != LastKey)return ;
	if(1 == PressKey1OFF)
	{// do nothing because press key1 off
		PressKey1OFF = 0;
		return ;
	}

	if(POWER_MODE ==ph.wm)
	{
		if(POWER_OFF == SysStat.pPwrStat->PwrOn)
		{
			SysStat.KnobTurn = KNOB_TURNING;
			open_power();
		}
	}
	else
	{
		
		if( (HEATER_NORMAL == SysStat.pHtrS->mode)&&\
			(HEATER_OFF == SysStat.pHtrS->HtrOn)&&\
			(0 == adChToNo) &&\
			(HEATER_WAKEN == SysStat.pHtrS->sleep))
		{
			POWER_HEATER_ON;
			pwm_init();
			pwm_i_duty(DUTY_CYCLE_90);
			pwm_u_duty(DUTY_CYCLE_5);
			SysStat.pHtrS->HtrOn = HEATER_ON;
			SysStat.pHtrS->heating = HEATING;
#ifndef    OLD_PID		
			SetMode(1);
#endif
		}
		SysStat.pHtrS->sleep = HEATER_WAKEN;
	}
	
	adChToNo = 0;
}



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
void display_step(u8 turn)
{
	u8 i = 0,j;
	
	if(SET_VOLTAGE_MODE == ph.pwr.CurSetMode)
 	{
 		i = 1;
		//open index 1 and index 2 display on first flash current index
		SysStat.pPwrStat->PwrLed.SegLedStat[1] = LED_ON;
		SysStat.pPwrStat->PwrLed.FlashCnt[1] = 0;
		SysStat.pPwrStat->PwrLed.SegLedStat[2] = LED_ON;
		SysStat.pPwrStat->PwrLed.FlashCnt[2] = 0;
		if(turn)
		{
	 		ph.pwr.pv.VolStep = \
				 (VOL_STEP_0_1_V == ph.pwr.pv.VolStep) ? VOL_STEP_1_V : VOL_STEP_0_1_V;
		}
		if(VOL_STEP_0_1_V == ph.pwr.pv.VolStep)i++;
 	}
	else
	{
		i = 5;
		SysStat.pPwrStat->PwrLed.SegLedStat[5] = LED_ON;
		SysStat.pPwrStat->PwrLed.FlashCnt[5] = 0;
		SysStat.pPwrStat->PwrLed.SegLedStat[6] = LED_ON;
		SysStat.pPwrStat->PwrLed.FlashCnt[6] = 0;
		if(turn)
		{
			ph.pwr.pc.CurStep = \
				(CUR_STEP_0_0_1_A == ph.pwr.pc.CurStep) ? CUR_STEP_0_1_A : CUR_STEP_0_0_1_A;
		}
		if(CUR_STEP_0_0_1_A == ph.pwr.pc.CurStep)
		{
			i++;
		}
		
	}
	SysStat.KnobTurnTime = 0;
	if(i < 4)
	{
		i = 0;
	}
	else
	{
		i = 4;
	}
	for(j=i; j<(i+3);j++)
	{
		SysStat.pPwrStat->PwrLed.FlashCnt[j] = 250;
		SysStat.pPwrStat->PwrLed.SegLedStat[j] = LED_FLASH_FAST;
	}
	
	
}


int exit_pwr_set(void *p)
{
	SysStat.pPwrStat->PwrSet.setModeOn = 0;
	SysStat.pPwrStat->PwrSet.ItmIdx = PWR_HTR_EN_INDEX;
	return 0;
}

/********************************************************************************
*fun name:          long_press_key2 
*fun description:   deal with long press key2 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2018-03-07  guolei
********************************************************************************/
 void long_press_key2(void)
{
	//( == ph.pwr.PwrOn)
	SysStat.pPwrStat->PwrSet.setModeOn = (SysStat.pPwrStat->PwrSet.setModeOn==0)?1:0;
	if(SysStat.pPwrStat->PwrSet.setModeOn)
	{
		reset_timer(SysStat.pPwrStat->PwrSet.PwrSetTmr);
		start_timer(SysStat.pPwrStat->PwrSet.PwrSetTmr);
	}
	else
	{
		
		stop_timer(SysStat.pPwrStat->PwrSet.PwrSetTmr);
		if( (0 == SysStat.pPwrStat->PwrSet.AdjPage) && \
			(PWR_CTR_TIM_INEX == SysStat.pPwrStat->PwrSet.ItmIdx) )
		{
			if(ph.pwr.PwrOffTimerM)
			{
				SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 1;
			}
			else
			{
				SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 0;
			}
			SysStat.PwrOffTmrStmp = SysStat.Tmr2CntMs;
		}
	}
	SysStat.pPwrStat->PwrSet.AdjPage = 1;
}


/********************************************************************************
*fun name:          short_press_key2 
*fun description:   deal with long press key2 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
 void short_press_key2(void)
{
	u8 i = 0,j = 0;
	

	if(0==SysStat.pPwrStat->PwrSet.setModeOn)
	{
		if(SET_VOLTAGE_MODE == ph.pwr.CurSetMode)
	 	{
			ph.pwr.CurSetMode = SET_CURRENT_MODE;
			i = 4;
	 	}
		else
		{		
			ph.pwr.CurSetMode = SET_VOLTAGE_MODE;
			i = 0;		
		}
		write_sys_data(&ph);
		SysStat.KnobTurnTime = 0;
		for(j=i;j<(i+3);j++)
		{
			SysStat.pPwrStat->PwrLed.FlashCnt[j] = 300;
			SysStat.pPwrStat->PwrLed.SegLedStat[j] = LED_FLASH_FAST;
		}
	}
	else
	{
		SysStat.pPwrStat->PwrSet.AdjPage = SysStat.pPwrStat->PwrSet.AdjPage?0:1;
		if( (1==SysStat.pPwrStat->PwrSet.AdjPage) &&\
			(PWR_CTR_TIM_INEX == SysStat.pPwrStat->PwrSet.ItmIdx))
		{
			if(ph.pwr.PwrOffTimerM)
			{
				SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 1;
			}
			else
			{
				SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 0;
			}
			
		}
	}
}




/********************************************************************************
*fun name:          up_key2 
*fun description:   deal with when  key2  up
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-01-03  guolei
********************************************************************************/
static void up_key2(void)
{
	//u32 tMs;
	
	if(0 == key2UpTimeStamp)
	{
		key2UpTimeStamp = SysStat.Tmr2CntMs;
	}
	else
	{
		long_press_key2();
		key2UpTimeStamp = 0;
	}
	
	if(HOT_MODE ==ph.wm)
	{
		if(HEATER_NORMAL == SysStat.pHtrS->mode)
		{
			SysStat.pHtrS->key2Cnt++;
		 	
				//SysStat.pHtrS->intoSleep = 1;

		 	
			if(1 == SysStat.pHtrS->key2Cnt)
			{
				SysStat.pHtrS->key2Tick = SysStat.Tmr2CntMs;
			}
			else if(5 == SysStat.pHtrS->key2Cnt)
			{
				if(0 == compare_timestamp(SysStat.pHtrS->key2Tick, 5000) )
				{
					SysStat.pHtrS->mode = HEATER_ADJUST;
					SysStat.pHtrS->cpySetTemp = ph.htr.SetTemp;
					ph.htr.SetTemp = ADJUST_TEMP1;
					SysStat.pHtrS->adActTemp = ADJUST_TEMP1;
				}
				SysStat.pHtrS->key2Cnt = 0;
				
			}
			else
			{
				if( compare_timestamp(SysStat.pHtrS->key2Tick, 5000) )
				{
					SysStat.pHtrS->key2Cnt = 0;
				}
			}
		}
	}
	
}



extern DISPLAY_DATA *pSegLed;
/********************************************************************************
*fun name:          deal_key 
*fun description:   deal with key 
*input param:       KeyValue:
*                          NOKEY:    no key press
*                          bit0~bit1: short press  
*                          bit2~bit3: long press 
*                          bit4~bit5:key up
*                          bit0  bit2  bit4:key1
*                          bit1  bit3  bit5:key2
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void deal_key(u8 KeyValue)
{
	u8 key1 = 0;// 1: short press   2: long press   4:key up
	u8 key2 = 0;// 1: short press   2: long press  4:key up  
	static u8 LastKey1 = 0;
	static u8 LastKey2 = 0;

	if(NOKEY == KeyValue)return;
	
	 
	 key1 =  KeyValue & (KEY1_DOWN|KEY1_UP|KEY1_LONG);//0x15;
	 key2 =  KeyValue & (KEY2_DOWN|KEY2_UP|KEY2_LONG);//0x2A;
	 
	//deal with key1
	 if(key1 >= KEY1_UP )
	 {//up
		up_key1(LastKey1);
		LastKey1 = KEY1_UP;
	 }
	 else if(key1 >= KEY1_LONG )
	 {//long press
	 	long_press_key1();
		LastKey1 = KEY1_LONG;
	 }
	 else if(key1 >= KEY1_DOWN )
	 {//short press
	 	short_press_key1();
		LastKey1 = KEY1_DOWN;
	 }
	 else
 	{
 		LastKey1 = 0;
 	}
	 if(0 != LastKey1)
	 {
	 	//模式切换参数已在long_press_key1中保存
	 	if(KEY1_LONG != LastKey1)
	 	{
		 	key_active();
	 	}
	 }
	 
	 //deal with key2
	 if((key2 >= KEY2_UP ))//&&(POWER_MODE == ph.wm))
	 {//up	
	 	if(KEY2_LONG != LastKey2)
	 	{
	 		up_key2();
	 	}
		else
		{
			SysStat.pPwrStat->PwrLed.SegLedStat[3] = LED_ON;
			SysStat.pPwrStat->PwrLed.FlashCnt[3] = 0;
			SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_ON;
			SysStat.pPwrStat->PwrLed.FlashCnt[7] = 0;
		}
		LastKey2 = KEY2_UP;
	 }
	 else if((key2 >= KEY2_LONG )&&(POWER_MODE ==ph.wm))
	 {//long press
	 	long_press_key2();
		LastKey2 = KEY2_LONG;
	 }
	 else if(key2 >= KEY2_DOWN )
	 {//short press	 	
		LastKey2 = KEY2_DOWN;
	 }
	  else
 	{
 		LastKey2 = 0;
 	}
	if(0 != LastKey2)
	 {
	 	key_active();
		if((KEY2_LONG == LastKey2) && (SET_CURRENT_MODE == ph.pwr.CurSetMode))
		{
			SysStat.KeyAct = KEY_INACTIVE;
		}
	}
	if(HOT_MODE ==ph.wm)
	{
			key_active();
	}

}



/********************************************************************************
*fun name:          open_heater_handle_irq 
*fun description:   open heater handle irq
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-07-28  guolei
********************************************************************************/
void open_heater_handle_irq(void)

{
	EXTI_PB8_Config();
}



/********************************************************************************
*fun name:          key_init 
*fun description:   init key io and other io 
                    key1:PA3  key2:PB9   
                    power mode :PA8 and  PB0 is High 
                    heater mode:PA8 and PB0 is Low
                    PB12:  High open hardware on power mode and heater mode
                           Low close hardware on power mode and heater mode
                    PB1:  pwm input
input param:
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void key_init(void)
{
	GPIO_InitTypeDef key1;
	GPIO_InitTypeDef key2;
	GPIO_InitTypeDef ModeIO_;
	GPIO_InitTypeDef ModeIO;
	GPIO_InitTypeDef PowerCtrIO;
	GPIO_InitTypeDef CoderPa0;
	GPIO_InitTypeDef CoderPa1;
	GPIO_InitTypeDef OverCurDectIo;
	GPIO_InitTypeDef PwmInputIo;
	GPIO_InitTypeDef TrembleIo;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE); 
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);



	
	key1.GPIO_Pin = GPIO_Pin_3; 
	key1.GPIO_Mode = GPIO_Mode_IPU;
	key1.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &key1);

	key2.GPIO_Pin = GPIO_Pin_9; 
	key2.GPIO_Mode = GPIO_Mode_IPU;
	key2.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &key2);




	PwmInputIo.GPIO_Pin = GPIO_Pin_8; 
	PwmInputIo.GPIO_Mode =  GPIO_Mode_IN_FLOATING;
	PwmInputIo.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &PwmInputIo);

	ModeIO.GPIO_Pin = GPIO_Pin_1; 
	ModeIO.GPIO_Mode = GPIO_Mode_Out_OD;
	ModeIO.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &ModeIO);


	TrembleIo.GPIO_Pin = GPIO_Pin_8; 
	TrembleIo.GPIO_Mode = GPIO_Mode_IPU;
	TrembleIo.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &TrembleIo);
	//EXTI_PB8_Config();




	OverCurDectIo.GPIO_Pin = GPIO_Pin_3; 
	OverCurDectIo.GPIO_Mode = GPIO_Mode_IPU;
	OverCurDectIo.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &OverCurDectIo);
	EXTI_PB3_Config();

	

	/* EXTI line gpio config(PA0) */
	CoderPa0.GPIO_Pin = GPIO_Pin_0; 
	CoderPa0.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;
	CoderPa0.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &CoderPa0);

	 
	 CoderPa1.GPIO_Pin = GPIO_Pin_1;
	 CoderPa1.GPIO_Mode =  GPIO_Mode_IPU;// ;//GPIO_Mode_IPD; // ????
	 GPIO_Init(GPIOA, &CoderPa1);


	
	
	ModeIO_.GPIO_Pin = GPIO_Pin_0; 
	ModeIO_.GPIO_Mode = GPIO_Mode_Out_PP;
	ModeIO_.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &ModeIO_);
	PowerCtrIO.GPIO_Pin = GPIO_Pin_12; 
	PowerCtrIO.GPIO_Mode = GPIO_Mode_Out_OD;
	PowerCtrIO.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &PowerCtrIO);
	POWER_HEATER_OFF;



	
	EXTI_PA0_Config();
	
}



