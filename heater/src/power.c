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
#include "power.h"
#include "Segled.h"
#include <timer.h>
#include "key.h"
#include "pwm.h"
#include "universal_timer.h"

#define  PWMI_STEP        5*(PWM_MAX_VALUE/1000)
#define  PWMU_STEP        5*(PWM_MAX_VALUE/1000)


POWER_STAT PwrStat;



static u16 LastUduty = 0;
static u16 LastIduty = 0;


#ifndef   NO_DISYPLAY_AD
volatile static u16 pwmiAd = 0;
#endif



/********************************************************************************
*fun name:          calc_pwmiu 
*fun description:   calc current pwmu or pwmi by current setting value 
*input param:       power   power param 
*                  
*output param:
*return value:  
*remark:        creat 2016-12-07  guolei
********************************************************************************/
void calc_pwmiu(POWER *power)
{
	u32 pwm;

	if(NULL == power)return ;

	//pwm = (adjustDat.pwrJ.pv.VPwmSet[INDEX_60V]*power->pv.SetVol)/MAX_VOL;
	if( power->pv.SetVol >= ADJUST_VOL )
	{
		pwm = (power->pv.SetVol-ADJUST_VOL)*(adjustDat.pwrJ.VPwmSet[INDEX_60V]-\
			     adjustDat.pwrJ.VPwmSet[INDEX_1V])/(MAX_VOL-ADJUST_VOL);
		SysStat.pPwrStat->pvs.pwmu = adjustDat.pwrJ.VPwmSet[ADJUST_INDEX] + pwm;
		//SysStat.pPwrStat->pvs.pwmu -= adjustDat.pwrJ.Vpwmb;
	}
	else
	{
		if(power->pv.SetVol)
		{
			pwm = (power->pv.SetVol-VOLT_0_1V)*(adjustDat.pwrJ.VPwmSet[INDEX_1V]-\
				     adjustDat.pwrJ.VPwmSet[INDEX_0_1V])/(VOLT_1V-VOLT_0_1V);
			
			//pwm = power->pv.SetVol*adjustDat.pwrJ.VPwmSet[INDEX_0_1V]/VOLT_0_1V;
			SysStat.pPwrStat->pvs.pwmu = pwm + adjustDat.pwrJ.VPwmSet[INDEX_0_1V];
		}
		else
		{
			SysStat.pPwrStat->pvs.pwmu = 0;
		}
	}

	//pwm = (power->pc.PwmSet[INDEX_3A]*power->pc.SetCur)/MAX_CUR;

	
	if(power->pc.SetCur > CUR_0_1A)
	{
		pwm = (power->pc.SetCur-CUR_0_1A)*(adjustDat.pwrJ.APwmSet[INDEX_3A]-\
			adjustDat.pwrJ.APwmSet[INDEX_0_1A])/(MAX_CUR-CUR_0_1A);
		SysStat.pPwrStat->pcs.pwmi =adjustDat.pwrJ.APwmSet[INDEX_0_1A] + pwm;
	}
	else if(power->pc.SetCur > CUR_0_0_1A)
	{
		pwm = (power->pc.SetCur - CUR_0_0_1A)*(adjustDat.pwrJ.APwmSet[INDEX_0_1A]-\
			adjustDat.pwrJ.APwmSet[INDEX_0_0_1A])/(CUR_0_1A-CUR_0_0_1A);
		SysStat.pPwrStat->pcs.pwmi =adjustDat.pwrJ.APwmSet[INDEX_0_0_1A] + pwm;
	}
	
	if(power->pc.SetCur == MAX_CUR)
	{
		SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_3A];
	}
	else if(power->pc.SetCur == CUR_0_1A)
	{
		SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_0_1A];
	}
	else if(power->pc.SetCur == CUR_0_0_1A)
	{
		SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_0_0_1A];
	}
	else if(0 == power->pc.SetCur)
	{
		SysStat.pPwrStat->pcs.pwmi = 0;
	}
	
}

void dis_data(char *pdata,DISPLAY_DATA *pDisDat)
{
	u8 i = 0;
	
	if(pDisDat)
	{
		for(i=0; i<8; i++)
		{
			pDisDat->data[i] = dis_char(*pdata & 0x7f);
			pDisDat->data[i] |= (*pdata & 0x80);
			pdata++;
		}
		if(0 == SysStat.pPwrStat->PwrSet.AdjPage)
		{
			pDisDat->data[3] |= 0x80;
		}
		else
		{
			pDisDat->data[3] &= ~0x80;
		}
	}
}
void dis_pwr_data(POWER *power)
{
	u8 i = 0;

	u16 DisVol = 0;
	u16 DisCur = 0;
	
	for( i=0;i<8;i++)
	{
		if(0==SysStat.pPwrStat->PwrLed.FlashCnt[i])
			SysStat.pPwrStat->PwrLed.SegLedStat[i] = LED_ON;
	}
	if(SysStat.pPwrStat->DisSetValue)
	{
		DisVol = power->pv.SetVol;
		DisCur = power->pc.SetCur;
		
		if(compare_timestamp(SysStat.pPwrStat->DisSetValueStamp,SysStat.pPwrStat->DisSetValueMs))
		{
			SysStat.pPwrStat->DisSetValue = 0;
			SysStat.pPwrStat->DisSetValueMs = 0;
		}
	}
	else
	{		
		DisVol = SysStat.pPwrStat->pvs.CurVol;
		DisCur = SysStat.pPwrStat->pcs.CurCur;
	}

	//ËÄÉáÎåÈë
	if(DisVol%100 >= 50)
	{
		DisVol += 100;
	}
	if(DisCur%10 >= 5)
	{
		DisCur += 10;
	}
	
	
	if(SysStat.KnobTurnTime++ >= KNOB_TURN_TIMER)
	{
		SysStat.KnobTurnTime = KNOB_TURN_TIMER;
		SysStat.KnobTurn = KNOB_STOP;
	}
	else
	{
		if(SET_VOLTAGE_MODE == ph.pwr.CurSetMode)
		{
			DisVol = power->pv.SetVol;
		}
		else
		{
			DisCur = power->pc.SetCur;
		}
	}
	// display unit
	SysStat.pPwrStat->PwrLed.data[3] = segmcode[16];
	// display unit
	SysStat.pPwrStat->PwrLed.data[7] = segmcode[10];


	
	if(DisVol)
	{
		SysStat.pPwrStat->PwrLed.data[2] = segmcode[DisVol/100%10];
		SysStat.pPwrStat->PwrLed.data[1] = segmcode[DisVol/1000%10]|0x80;//0x80 display dot
		

		if(DisVol/10000)
		{
			SysStat.pPwrStat->PwrLed.data[0] = segmcode[DisVol/10000%10];
		}
		else
		{
			SysStat.pPwrStat->PwrLed.SegLedStat[0] = LED_OFF;
			SysStat.pPwrStat->PwrLed.data[0] = 0;
			SysStat.pPwrStat->PwrLed.FlashCnt[0] = 0;
		}
	}
	else
	{
		
		SysStat.pPwrStat->PwrLed.SegLedStat[0] = LED_OFF;
		SysStat.pPwrStat->PwrLed.data[2] = segmcode[0];
		SysStat.pPwrStat->PwrLed.data[1] = segmcode[0]|0x80;
		SysStat.pPwrStat->PwrLed.data[0] = 0;
	}

	DisCur /= 10;
	SysStat.pPwrStat->PwrLed.data[6] = segmcode[DisCur%10];
	SysStat.pPwrStat->PwrLed.data[5] = segmcode[DisCur/10%10];
	SysStat.pPwrStat->PwrLed.data[4] = segmcode[DisCur/100%10]|0x80;
	key_inactive(1500);
}


void dis_pwr_item(void)
{
	char data[8]= {0}; 
	
	switch(SysStat.pPwrStat->PwrSet.ItmIdx)
	{
		case PWR_HTR_EN_INDEX:
			if(ph.pwr.PwrOn)
			{
				data[0] = '0';
				data[1] = 'N';
				//itoa(ph.pwr.PwrOffTimerM,a,10);
			}
			else
			{
				data[0] = '0';
				data[1] = 'F';
				data[2] = 'F';
			}
			//ph.pwr.PwrOffTimerM = 99;
			sprintf(data+4,"%4d",ph.pwr.PwrOffTimerM);
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
#if   0			
		case 1:
			sprintf(data,"%4d",ph.pwr.PwrOffTimerM);
			sprintf(data+4,"%3d", ph.pwr.PretecTemp);
			data[4] = 0;
			data[6] |= 0x80;
			data[7] = 'C';
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
		case 2:
			sprintf(data,"%3d", ph.pwr.PretecTemp);
			data[0] = 0;
			data[2] |= 0x80;
			data[3] = 'C';
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
#else			
		case PWR_CTR_TIM_INEX:
			sprintf(data,"%4d",ph.pwr.PwrOffTimerM);
			sprintf(data+4,"%d", ph.pwr.PretecTemp);
			data[6] = 'c';
			data[7] = 'C';
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
		case OVER_TEMP_INDEX:
			sprintf(data,"%d", ph.pwr.PretecTemp);
			data[2] = 'c';
			data[3] = 'C';
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
#if 0			
		case 2:
			sprintf(data,"%4d", ph.pwr.PretecTemp);
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
#endif	
#endif
		//add display other item
		default:
			dis_data(data, &SysStat.pPwrStat->PwrLed);
			break;
	}
}


//first short press key2 into setup item data,second short press key2 outo setup item data
void dis_pwr_set(POWER *power)
{
	u8 i = 0;

	for( i=0;i<8;i++)
	{
		if(0==SysStat.pPwrStat->PwrLed.FlashCnt[i])
			SysStat.pPwrStat->PwrLed.SegLedStat[i] = LED_ON;
	}
	dis_pwr_item();
}
void dis_pwr_alarm(void)
{
	if(POWER_OFF == SysStat.pPwrStat->PwrOn)
	{
		SysStat.pPwrStat->PwrLed.data[7] = segmcode[15];
		SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_FLASH_SLOW;
		SysStat.pPwrStat->PwrLed.FlashCnt[7] = DISPLAY_ON;
	}

	if(PWR_FLASH_ERR & SysStat.pPwrStat->PwrErr)
	{
		SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pPwrStat->PwrLed.data[7] = segmcode[14];
		SysStat.pPwrStat->PwrLed.FlashCnt[7] = DISPLAY_ON;
	}
	
	if(PWR_CUR_ERR & SysStat.pPwrStat->PwrErr)
	{
		SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pPwrStat->PwrLed.data[7] = segmcode[10];
		SysStat.pPwrStat->PwrLed.FlashCnt[7] = DISPLAY_ON;
	}
	
	if(PWR_TEMP_BOARD_ERR & SysStat.pPwrStat->PwrErr)
	{
		SysStat.pPwrStat->PwrLed.data[7] = segmcode[12];
		SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pPwrStat->PwrLed.FlashCnt[7] = DISPLAY_ON;
	}

	if(PWR_VOLT_ERR & SysStat.pPwrStat->PwrErr)
	{
		SysStat.pPwrStat->PwrLed.data[3] = segmcode[16];
		SysStat.pPwrStat->PwrLed.SegLedStat[3] = LED_FLASH_FAST;
		SysStat.pPwrStat->PwrLed.FlashCnt[3] = DISPLAY_ON;
	}
		
		
	if(PWR_PWM_FREQ_ERR & SysStat.pPwrStat->PwrErr)
	{
		SysStat.pPwrStat->PwrLed.data[7] = segmcode[15];
		SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pPwrStat->PwrLed.FlashCnt[7] = DISPLAY_ON;
	}
}
/********************************************************************************
*fun name:          power_display 
*fun description:   power mode display 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-07  guolei
********************************************************************************/
void power_display(POWER *power)
{
	
	static u32 timerStamp = 0;
#ifndef   NO_DISYPLAY_AD
	static u32 tick1 = 0;
	u8 n = 0;
	u8 i = 0;
#endif

#ifndef   NO_DISYPLAY_PWMFREQ
static u32 tick1 = 0;
	u32 freqKhz = 0;
#endif
	
	
		
	if(NULL == power)return ;

	if(0==SysStat.pPwrStat->PwrSet.setModeOn)
	{
		dis_pwr_data(power);
		dis_pwr_alarm();
		if( (PWR_NO_ERR==SysStat.pPwrStat->PwrErr) && \
			SysStat.pPwrStat->PwrSet.EnTimeCtrPwr ) 
		{
			if(compare_timestamp(timerStamp, 2000))
			{
				timerStamp = SysStat.Tmr2CntMs;
			}
			else if(compare_timestamp(timerStamp, 1000))
			{
				SysStat.pPwrStat->PwrLed.data[7] |= 0x80;
				
			}
			else
			{
				SysStat.pPwrStat->PwrLed.data[7] &= ~0x80;
			}
		}
		else
		{
			SysStat.pPwrStat->PwrLed.data[7] &= ~0x80;
		}
	}
	else
	{
		dis_pwr_set(power);
	}

	
	
#ifndef   NO_DISYPLAY_PWMFREQ
	if(compare_timestamp(tick1, 10000) )
	{
		tick1 = SysStat.Tmr2CntMs;
	}
	else if(compare_timestamp(tick1, 5000) )
	{
		SysStat.pPwrStat->PwrLed.SegLedStat[0] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[1] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[2] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[3] = LED_OFF;
		freqKhz = SysStat.PwmFreq;
		
		SysStat.pPwrStat->PwrLed.SegLedStat[3] = LED_ON;
		SysStat.pPwrStat->PwrLed.data[3] = segmcode[freqKhz%10];
		freqKhz /= 10;
		
		if(freqKhz)
		{
			SysStat.pPwrStat->PwrLed.SegLedStat[2] = LED_ON;
			SysStat.pPwrStat->PwrLed.data[2] = segmcode[freqKhz%10];
			freqKhz /= 10;
		}
		if(freqKhz)
		{
			SysStat.pPwrStat->PwrLed.SegLedStat[1] = LED_ON;
			SysStat.pPwrStat->PwrLed.data[1] = segmcode[freqKhz%10];
			freqKhz /= 10;
		}
		if(freqKhz)
		{
			SysStat.pPwrStat->PwrLed.SegLedStat[0] = LED_ON;
			SysStat.pPwrStat->PwrLed.data[0] = segmcode[freqKhz%10];
			freqKhz /= 10;
		}
	}
#endif			
	

#ifndef   NO_DISYPLAY_AD
	if(compare_timestamp(tick1, 10000) )
	{
		tick1 = SysStat.Tmr2CntMs;
	}
	else if(compare_timestamp(tick1, 5000) )
	{
		SysStat.pPwrStat->PwrLed.SegLedStat[0] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[1] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[2] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[3] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[4] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[5] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[6] = LED_OFF;
		SysStat.pPwrStat->PwrLed.SegLedStat[7] = LED_OFF;
		for(i=0; i<8; i++)
		{
			if(pwmiAd)
			{
				n = pwmiAd % 10;
				SysStat.pPwrStat->PwrLed.SegLedStat[7-i] = LED_ON;
				SysStat.pPwrStat->PwrLed.data[7-i] = segmcode[n];

				pwmiAd /= 10;
			}
			else
			{
				break;
			}
		}
	}

	
	
#endif

	//pSegLed = &SysStat.pPwrStat->PwrLed;
	pSegLed = &PwrStat.PwrLed;
	
}

/**********************************************************************
*fun name:          Cal_volt_Cur 
*fun description:  calc value which volt and current
*input param:      pStat: sys stat param
*                          pVolt:
*                                   pVolt[0]  current  AD
*                                   pVolt[1]  volt  AD
*output param:
*                          0: measure  value between set value  is in CURRENT_TAP_LINE and
*                              VOLT_TAP_LINE
*                          1: over CURRENT_TAP_LINE and VOLT_TAP_LINE
*return value:  
*remark:        creat 2017-01-06  guolei
**********************************************************************/
void Cal_volt_Cur(SYS_STAT_PARA *pStat,u16 *pVolt)
{
	static u8 ErrCnt = 0;
	u16 cur;
	u16 volt;
	int vref = 1200;//unit:1mv
	//u32 k;
	static u32 Timer = 0,voltS = 0,curS = 0,cnt = 0;
		
	if(pStat&&pVolt)
	{

		cur = vref*pVolt[0]/pVolt[4] ;
		cur = 5*cur/3;
		volt = 20*vref*pVolt[1]/pVolt[4] ;

#ifndef   NO_DISYPLAY_AD
		pwmiAd = pVolt[0];
#endif		
#if  1
		if( 0xffff != adjustDat.pwrJ.VAd[INDEX_0_1V] )
		{
			if(pVolt[1] >= adjustDat.pwrJ.VAd[INDEX_1V])
			{
				volt = VOLT_1V + (pVolt[1] - adjustDat.pwrJ.VAd[INDEX_1V])*(MAX_VOL-VOLT_1V)/((adjustDat.pwrJ.VAd[INDEX_60V] - adjustDat.pwrJ.VAd[INDEX_1V]));
			}
			else if(pVolt[1] > adjustDat.pwrJ.VAd[INDEX_0_1V])
			{
				
				volt = VOLT_1V - (adjustDat.pwrJ.VAd[INDEX_1V] - pVolt[1])*(VOLT_1V-VOLT_0_1V)/(adjustDat.pwrJ.VAd[INDEX_1V] - adjustDat.pwrJ.VAd[INDEX_0_1V]);
			}
			else 
			{
				volt = 0;
				if( (POWER_ON == pStat->pPwrStat->PwrOn) && (VOLT_0_1V == ph.pwr.pv.SetVol) )	
				{
					volt = VOLT_0_1V;
				}
				
			}
		}
		if( 0xffff != adjustDat.pwrJ.AAd[INDEX_0_1A] )
		{
			if(pVolt[0] >= adjustDat.pwrJ.AAd[INDEX_0_1A])
			{
				cur = CUR_0_1A + (pVolt[0] - adjustDat.pwrJ.AAd[INDEX_0_1A])*(MAX_CUR-CUR_0_1A)/((adjustDat.pwrJ.AAd[INDEX_3A] - adjustDat.pwrJ.AAd[INDEX_0_1A]));
			}
			else if(pVolt[0] >= adjustDat.pwrJ.AAd[INDEX_0_0_1A])
			{
				cur = CUR_0_1A - (adjustDat.pwrJ.AAd[INDEX_0_1A]-pVolt[0])*(CUR_0_1A-CUR_0_0_1A)/((adjustDat.pwrJ.AAd[INDEX_0_1A] - adjustDat.pwrJ.AAd[INDEX_0_0_1A]));
			}
			else 
			{
				cur = 0;
				if( (POWER_ON == pStat->pPwrStat->PwrOn) && (CUR_0_0_1A == ph.pwr.pc.SetCur) )	
				{
					cur = CUR_0_0_1A;
				}
			}
		}
		
#endif		

		if((volt <= MAX_VOL)&&(cur <= MAX_CUR))
		{
			ErrCnt = 0;
		}
		//volt and current err?
		if((volt > MAX_VOL)&&(ErrCnt < VOLT_CUR_ERR_CNT))
		{
			ErrCnt++;
			volt = MAX_VOL;
			if(ErrCnt>=VOLT_CUR_ERR_CNT)
			{
				volt = MAX_VOL+VOL_STEP_0_1_V;

			}
		}
		
		if((cur > MAX_CUR) && (ErrCnt < VOLT_CUR_ERR_CNT))
		{
			ErrCnt++;
			cur = MAX_CUR;
			if(ErrCnt>=VOLT_CUR_ERR_CNT)
			{
				cur = MAX_CUR+CUR_STEP_0_0_1_A;
			}
		}

		
			voltS += volt;
			curS += cur;
			cnt++;
			if(compare_timestamp(Timer, 500) )
			{
				voltS /= cnt;
				curS /= cnt;
				pStat->pPwrStat->pcs.CurCur = curS;
				pStat->pPwrStat->pvs.CurVol = voltS;
				voltS = 0;
				curS = 0;
				cnt = 0;
				Timer = SysStat.Tmr2CntMs;
			}
		
		
	}
}



/**********************************************************************
*fun name:          smooth_pwm 
*fun description:   smooth pwm 
*input param:      power: power param address
*                  //type:    PWM_U_TYPE    pwm
*                                    
*output param:
*return value:  
*remark:        creat 2017-06-13  guolei
**********************************************************************/
static void smooth_pwm(POWER *power)
{
	u16 pwmu,pwmi;

	
	pwmu = SysStat.pPwrStat->pvs.pwmu;

	if(pwmu != LastUduty)
	{
		if(pwmu > PWMU_STEP)
		{
			if(((pwmu-PWMU_STEP) <= LastUduty) && (LastUduty <= (pwmu+PWMU_STEP)))
			{
				LastUduty = pwmu;
			}
			else if(LastUduty < (pwmu-PWMU_STEP))
			{
				LastUduty += PWMU_STEP;
			}
			else if((pwmu+PWMU_STEP) < LastUduty)
			{
				LastUduty -= PWMU_STEP;
			}
		}
		else
		{
			LastUduty = pwmu;
		}
		pwm_u_duty(LastUduty);
	}

	
	
	pwmi = SysStat.pPwrStat->pcs.pwmi;
	if(pwmi != LastIduty)
	{
		if(pwmi > PWMI_STEP)
		{
			if(((pwmi-PWMI_STEP) <= LastIduty) && (LastIduty <= (pwmi+PWMI_STEP)))
			{
				LastIduty = pwmi;
			}
			else if(LastIduty < (pwmi-PWMI_STEP))
			{
				LastIduty += PWMI_STEP;
			}
			else if((pwmi+PWMI_STEP) < LastIduty)
			{
				LastIduty -= PWMI_STEP;
			}
		}
		else
		{
			LastIduty = pwmi;
		}
		pwm_i_duty(LastIduty);
	}
}

/**********************************************************************
*fun name:          judge_alarm 
*fun description:   judge power mode alarm 
*input param:      power: power param address
*                  //type:    PWM_U_TYPE    pwm
*                                    
*output param:
*return value:  
*remark:        creat 2017-06-13  guolei
**********************************************************************/
void judge_alarm(void)
{
	static u32 VoltT1 = 0;
	static u32 CurT1 = 0;
	static u32 TempT1 = 0;
	
	
	judge_pwm_err(&ph);
	
	

	if(SysStat.InTemp > ph.pwr.PretecTemp)
	{
		if(0 == TempT1)
		{
			TempT1 = SysStat.Tmr2CntMs;
		}
		else
		{
			if(compare_timestamp(TempT1, 500) )
			{
				SysStat.pPwrStat->PwrErr |= PWR_TEMP_BOARD_ERR;
			}
		}
	}
	if(PWM_FREQ_OVER == SysStat.PwmFreqErr )
	{
		SysStat.pPwrStat->PwrErr |= PWR_PWM_FREQ_ERR;
	}

	

	//Don't update pwm when key turn,volt is not equal set volt so close judge volt alarm
	if( SysStat.pPwrStat->pvs.CurVol > (MAX_VOL+1000) )
	{
	
		
		if(0 == VoltT1)
		{
			VoltT1 = SysStat.Tmr2CntMs;
		}
		else
		{
			if(compare_timestamp(VoltT1, 80) )
			{
				SysStat.pPwrStat->PwrErr |= PWR_VOLT_ERR;
			}
		}
		
	}
	else
	{
		VoltT1 = 0;
	}

	if( SysStat.pPwrStat->pcs.CurCur > (MAX_CUR*11/10) )
	{
		if(0 == CurT1)
		{
			CurT1 = SysStat.Tmr2CntMs;
		}
		else
		{
			if( compare_timestamp(CurT1, 80) )	
			{
				SysStat.pPwrStat->PwrErr |= PWR_CUR_ERR;
			}
		}
	}
	else
	{
		CurT1 = 0;
	}
	


#if 0	
	if(KEY_INACTIVE == SysStat.KeyAct)
	{
		//Don't update pwm when key turn,volt is not equal set volt so close judge volt alarm
		if( ph.pwr.pv.SetVol && (SysStat.pPwrStat->pvs.CurVol > ((ph.pwr.pv.SetVol*11/10)+500)) &&\
			(compare_timestamp(SysStat.pPwrStat->voltTurnStamp, 30000) ) )
		{
			
			
			if(0 == VoltT1)
			{
				VoltT1 = SysStat.Tmr2CntMs;
			}
			else
			{
				if(compare_timestamp(VoltT1, 500) )
				{
					SysStat.pPwrStat->PwrErr |= PWR_VOLT_ERR;
				}
			}
			
		}
		else
		{
			VoltT1 = 0;
		}
	
		if( ph.pwr.pc.SetCur && ( SysStat.pPwrStat->pcs.CurCur > (ph.pwr.pc.SetCur*11/10) +100) &&\
			compare_timestamp(SysStat.pPwrStat->curTurnStamp, 30000) )
		{
			if(0 == CurT1)
			{
				CurT1 = SysStat.Tmr2CntMs;
			}
			else
			{
				if( compare_timestamp(CurT1, 500) )	
				{
					SysStat.pPwrStat->PwrErr |= PWR_CUR_ERR;
				}
			}
		}
		else
		{
			CurT1 = 0;
		}
	}
#endif	

}

/**********************************************************************
*fun name:          test_pwr_alarm 
*fun description:   test power mode alarm 
*input param:     
*                 
*                                    
*output param:
*return value:  
*remark:        creat 2017-07-02  guolei
**********************************************************************/
void test_pwr_alarm(void)
{
	static u32  TestPTime = 0;
	
	if( compare_timestamp(TestPTime, 60000) )
	{
		TestPTime = SysStat.Tmr2CntMs;
		SysStat.pPwrStat->PwrSleep = POWER_WAKEN;
	}
	else if( compare_timestamp(TestPTime, 50000) )
	{
		SysStat.pPwrStat->PwrSleep = POWER_SLEEPED;
	}
	else if( compare_timestamp(TestPTime, 40000) )
	{
		SysStat.pPwrStat->PwrErr = PWR_VOLT_ERR;
	}
	else if( compare_timestamp(TestPTime, 30000) )
	{
		SysStat.pPwrStat->PwrErr = PWR_CUR_ERR;
	}
	else if( compare_timestamp(TestPTime, 20000) )
	{
		SysStat.pPwrStat->PwrErr = PWR_PWM_FREQ_ERR;
	}
	else if( compare_timestamp(TestPTime, 10000) )
	{
		SysStat.pPwrStat->PwrErr = PWR_TEMP_BOARD_ERR;
	}

}

/**********************************************************************
*fun name:          power_ctrl 
*fun description:  power mode 
*input param:      power: power param address
*                          pVolt:  pVolt[0]: current  AD
*                                     pVolt[1]: volt  AD 
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
**********************************************************************/
void power_ctrl(POWER *power,u16 *pVolt)
{


		
	if(NULL == power)return ;

	POWER_MODE_CTR;
	Cal_volt_Cur(&SysStat,pVolt);
	power_display(power);
	
	//power off and set pwmi and pwmu to zero
	if((POWER_OFF == SysStat.pPwrStat->PwrOn) ||\
		(PWR_NO_ERR != SysStat.pPwrStat->PwrErr))
	{
		POWER_HEATER_OFF;
		SysStat.pPwrStat->pvs.pwmu = 0;
		SysStat.pPwrStat->pcs.pwmi = 0;
		//LastUduty = 0;
		//LastIduty = 0;
		pwm_u_duty(0);
		pwm_i_duty(0);
		pwm_change_io();
	}
	else
	{
		POWER_HEATER_ON;
		calc_pwmiu(power);
		smooth_pwm(power);
	}
		
	if(PWR_NO_ERR != SysStat.pPwrStat->PwrErr)
	{
		POWER_HEATER_OFF;
		SysStat.pPwrStat->pvs.pwmu = 0;
		SysStat.pPwrStat->pcs.pwmi = 0;
		pwm_u_duty(0);
		pwm_i_duty(0);
	}
#if OPEN_POWER_ALARM	
	//67.25us a timer
	judge_alarm();
#endif
}


/********************************************************************************
*fun description:   power_adjust_parm_init
*input param:    
*output param:
*return value:
********************************************************************************/
void power_adjust_parm_init(void)
{
	adjustDat.pwrJ.VPwmSet[INDEX_60V] = PWM_ACTUAL_60;
	adjustDat.pwrJ.VPwmSet[ADJUST_INDEX] = (PWMU_DEFAULT_60*ADJUST_VOL)/MAX_VOL;
	adjustDat.pwrJ.VPwmSet[INDEX_0_1V] = PWMU_DEFAULT_60/600;
	//adjustDat.pwrJ.Vpwmb = 0;

	
	adjustDat.pwrJ.VPwmSet[ADJUST_INDEX] = PWMU_DEFAULT_60*ADJUST_VOL/MAX_VOL;

	adjustDat.pwrJ.VAd[ADJUST_INDEX] = 0xffff;
	adjustDat.pwrJ.VAd[INDEX_60V] = 0xffff;
	adjustDat.pwrJ.VAd[INDEX_0_1V] = 0xffff;
	adjustDat.pwrJ.VAd[INDEX_0V] = 0;

	adjustDat.pwrJ.AAd[INDEX_0_1A] = 0xffff;
	adjustDat.pwrJ.AAd[INDEX_3A] = 0xffff;
	adjustDat.pwrJ.AAd[INDEX_0_0_1A] = 0xffff;
	adjustDat.pwrJ.AAd[INDEX_0A] = 0;

	
	adjustDat.pwrJ.APwmSet[INDEX_0_1A] = PWMI_DEFAULT_3/30;
	adjustDat.pwrJ.APwmSet[INDEX_3A] = PWMI_DEFAULT_3;
	adjustDat.pwrJ.APwmSet[INDEX_0_0_1A] = adjustDat.pwrJ.APwmSet[INDEX_0_1A]/10;
	//adjustDat.pwrJ.Apwmb = 0;
}



/********************************************************************************
*fun description:   PA6 and PA7 config pwm
*input param:    
*output param:
*return value:
********************************************************************************/
void power_init(POWER *power)
{
	u8 i = 0;

	
	if(NULL == power)return ;
	power->pv.SetVol = DEFAULT_VOL;
	power->pc.SetCur = DEFAULT_CUR;
	power->pv.VolStep= VOL_STEP_1_V;
	power->pc.CurStep= CUR_STEP_0_0_1_A;
	SysStat.pPwrStat->pcs.pwmi = 13080/3;//54.5%
	SysStat.pPwrStat->pvs.pwmu = 21816/12;//90.9%
	
	
	

	SysStat.KnobTurn = KNOB_STOP;
	SysStat.KnobTurnTime = 0;
	SysStat.pPwrStat->PwrErr = PWR_NO_ERR;
	ph.pwr.CurSetMode = SET_VOLTAGE_MODE;
	ph.pwr.PwrOffTimerM = 120;
	ph.pwr.PwrOn = 0;
	ph.pwr.PretecTemp = 70;
	SysStat.pPwrStat->PwrSet.ItmIdx= PWR_HTR_EN_INDEX;
	SysStat.pPwrStat->PwrOn = POWER_OFF;
	SysStat.pPwrStat->PwrSet.setModeOn = 0;
	POWER_HEATER_OFF;
	pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
	pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
	
	


	for(i=0; i<8; i++)
		SysStat.pPwrStat->PwrLed.FlashCnt[i] = 0;
}

/********************************************************************************
*fun name:          power_stat_init 
*fun description:   init power stat
*input param:       power: power param address
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
********************************************************************************/
void power_stat_init(POWER *power)
{
	u8 i = 0;
	
	if(NULL == power)return ;
	SysStat.pPwrStat->PwrErr = PWR_NO_ERR;
	SysStat.pPwrStat->PwrOn = POWER_OFF;
	SysStat.pPwrStat->PwmUpdateCnt = 0;
	SysStat.pPwrStat->PwmUpdateCnt = DISPLAY_UPDATE_CNT;
	//PWR_VOLT_STR init
	if(power->pv.SetVol > MAX_VOL)power->pv.SetVol = MAX_VOL;
	if((power->pv.VolStep != VOL_STEP_1_V) && (power->pv.VolStep != VOL_STEP_0_1_V) )
		power->pv.VolStep = VOL_STEP_0_1_V;

	if(adjustDat.pwrJ.VPwmSet[INDEX_60V] < PWMU_MIN_60)
		adjustDat.pwrJ.VPwmSet[INDEX_60V] = PWMU_DEFAULT_60;
	//if(adjustDat.pwrJ.VPwmSet[INDEX_0_1V] > PWMU_MAX_0_1)
	//	adjustDat.pwrJ.VPwmSet[INDEX_0_1V] = PWMU_DEFAULT_0_1;

	//POWER_VOLT_STAT
	SysStat.pPwrStat->pvs.CurVol = 0;
	//calc pwmu
	//calc_pwmiu(power,PWM_U_TYPE);

	//POWER_CUR
	if(power->pc.SetCur > MAX_CUR)power->pc.SetCur = MAX_CUR;
	if((power->pc.CurStep != CUR_STEP_0_0_1_A) && (power->pc.CurStep != CUR_STEP_0_1_A) )
		power->pc.CurStep = CUR_STEP_0_0_1_A;
	if(adjustDat.pwrJ.APwmSet[INDEX_3A] < PWMI_MIN_3)
		adjustDat.pwrJ.APwmSet[INDEX_3A] = PWMI_DEFAULT_3;
	if(adjustDat.pwrJ.APwmSet[INDEX_0_1A] > PWMI_MAX_0_1)
		adjustDat.pwrJ.APwmSet[INDEX_0_1A] = PWMI_DEFAULT_0_1;

	//POWER_CUR_STAT
	SysStat.pPwrStat->pcs.CurCur = 0;
	//calc pwmi
	//calc_pwmiu(power,PWM_I_TYPE);
	for(i=0;i<8;i++)
	{
		SysStat.pPwrStat->PwrLed.data[i] = 0x00;
		SysStat.pPwrStat->PwrLed.SegLedStat[i] = LED_OFF;
	}
	SysStat.pPwrStat->PwrSleep = POWER_WAKEN;
	SysStat.pPwrStat->DisSetValue = 0;
	SysStat.pPwrStat->DisSetValueMs = 0;
	SysStat.pPwrStat->DisSetValueStamp = 0;
	SysStat.pPwrStat->voltTurnStamp = 0;
	SysStat.pPwrStat->curTurnStamp = 0;
	
	SysStat.pPwrStat->PwrSet.AdjPage = 1;
	SysStat.pPwrStat->PwrSet.EnTimeCtrPwr = 0;
	SysStat.pPwrStat->PwrSet.PwrSetTmr = creat_timer(60000, 0, exit_pwr_set,NULL);
}
	

