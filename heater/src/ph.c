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
#include <stdio.h>
#include "ph.h"
#include "flash.h"



SYS_STAT_PARA SysStat;
ADJUST_VALUE adjustDat;
PWR_HTR ph;

static u32 SysParaAddr = FLASH_USER_START_ADDR;
static u32 CurSysParaAddr = FLASH_USER_START_ADDR;






/********************************************************************************
*fun name:          stat_init 
*fun description:   init state of system param
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
********************************************************************************/
void stat_init(PWR_HTR *pSysDat)
{
	if(NULL == pSysDat)return ;
	SysStat.pHtrS = &HeaterStat;
	SysStat.pPwrStat = &PwrStat;
	
	SysStat.pHtrS = &HeaterStat;
	SysStat.pPwrStat= &PwrStat;
	pSysDat->vld = 0;
	SysStat.ticks= 0;
	SysStat.Tmr2CntMs = 0;
	SysStat.KnobTurn = KNOB_STOP;
	SysStat.KnobTurnTime = 0;
	SysStat.InTemp = 0;
	SysStat.PwmFreq = 0;
	SysStat.PwmFreqErr = PWM_FREQ_NO_ERR;
	SysStat.KeyActMs = 0;
	SysStat.KeyAct = KEY_INACTIVE;

	heater_stat_init(&pSysDat->htr);
	power_stat_init(&pSysDat->pwr);
	
}




/********************************************************************************
*fun name:          calc_param_sum 
*fun description:   calc PWR_HTR struct sum
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
static void calc_param_sum(PWR_HTR *pSysDat)
{
	u32 sum = 0;
	sum += pSysDat->wm;
	sum += pSysDat->pwr.pv.SetVol;
	sum += pSysDat->pwr.pv.VolStep;
	
	sum += pSysDat->pwr.pc.SetCur;
	sum += pSysDat->pwr.pc.CurStep;

	sum += pSysDat->htr.SetTemp;
	
	pSysDat->sum = sum;
	if(0xffff == pSysDat->sum)pSysDat->sum = 0;
}

/********************************************************************************
*fun name:          calc_adjust_value_sum 
*fun description:   calc ADJUST_VALUE struct sum
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
static void calc_adjust_value_sum(ADJUST_VALUE *pAdjust)
{
	u32 sum = 0;
	
	sum += pAdjust->htrJ.adjTemp1;
	sum += pAdjust->htrJ.adjActTemp1;
	sum += pAdjust->htrJ.adjTemp2;
	sum += pAdjust->htrJ.adjActTemp2;
	
	//sum += pAdjust->pwrJ.Vpwmb;
	
	//sum += pAdjust->pwrJ.Apwmb;
	
	sum += pAdjust->pwrJ.VPwmSet[0];
	sum += pAdjust->pwrJ.VPwmSet[1];
	sum += pAdjust->pwrJ.VPwmSet[2];
	
	sum += pAdjust->pwrJ.APwmSet[0];
	sum += pAdjust->pwrJ.APwmSet[1];
	sum += pAdjust->pwrJ.APwmSet[2];
	
	sum += pAdjust->pwrJ.VAd[0];
	sum += pAdjust->pwrJ.VAd[1];
	sum += pAdjust->pwrJ.VAd[2];
	sum += pAdjust->pwrJ.VAd[3];
	
	sum += pAdjust->pwrJ.AAd[0];
	sum += pAdjust->pwrJ.AAd[1];
	sum += pAdjust->pwrJ.AAd[2];
	sum += pAdjust->pwrJ.AAd[3];
	
	
	if(0xffff == pAdjust->sum)pAdjust->sum = 0;
}


/********************************************************************************
*fun name:          data_default 
*fun description:   set system param default 
*input param:       
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void data_default(void)
{
	ph.wm = POWER_MODE;//POWER_MODE;
	HINT_DEBUG("data defalut\r\n");
	power_init(&ph.pwr);
	heater_init(&ph.htr);
	ph.vld = PARAM_VALID;
	calc_param_sum(&ph);
	
}
/********************************************************************************
*fun name:          read_adjust_data 
*fun description:   read adjust param
*input param:       pAdjustDat  
*output param:
*return value:  
*remark:        creat 2017-07-18  guolei
********************************************************************************/
void read_adjust_data(ADJUST_VALUE *pAdjustDat)
{
	u16 sum;

	
	if(pAdjustDat)
	{
		usr_data_read((u32 *)(pAdjustDat),ADJ_PARAM_START_ADDR,sizeof(ADJUST_VALUE));
		sum = pAdjustDat->sum;
		calc_adjust_value_sum(pAdjustDat);
		if((PARAM_VALID == pAdjustDat->vld) && (sum == pAdjustDat->sum) )
		{
		}
		else
		{
			heater_adjust_parm_init();
			power_adjust_parm_init();
		}
	}
}


/********************************************************************************
*fun name:          write_adjust_data 
*fun description:   write adjust param
*input param:       pAdjustDat  
*output param:
*return value:  
*remark:        creat 2017-07-18  guolei
********************************************************************************/
PH_ERR write_adjust_data(ADJUST_VALUE *pAdjustDat)
{
	PH_ERR err = PH_OK;
		
	if(FLASH_COMPLETE != FLASH_ErasePage(ADJ_PARAM_START_ADDR))
	{
		FLASH_DEBUG("erase flash fail\r\n");
		err = PH_FLASH_ERR;
	}
	pAdjustDat->vld = PARAM_VALID;
	calc_adjust_value_sum(pAdjustDat);
	usr_data_write((u32 *)(pAdjustDat),ADJ_PARAM_START_ADDR,sizeof(ADJUST_VALUE));

	return err;
}


/********************************************************************************
*fun name:          write_sys_data 
*fun description:   write system work param
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
PH_ERR write_sys_data(PWR_HTR *pSysDat)
{
	
	u32 offset = 0;
	int SysParamRelocate = 0;
	
	if(NULL == pSysDat)return PH_ERR_NULL;
	
	pSysDat->vld = PARAM_VALID;

	
	if( (SysParaAddr+sizeof(PWR_HTR)) > FLASH_USER_END_ADDR)
	{
		SysParamRelocate = 1;
	}
	offset = SysParaAddr%PAGE_SIZE;
	//it's no enough space for PWR_HTR struct in a sector
	if(offset &&((PAGE_SIZE-offset) < sizeof(PWR_HTR)))
	{
		//no space in this page must jump next page
		SysParaAddr += (PAGE_SIZE-offset);
	}
	
	calc_param_sum(pSysDat);
	
	
	if( SysParamRelocate )
	{
		SysParaAddr = FLASH_USER_START_ADDR;
		erase_all_data();
	}
	
	usr_data_write((u32 *)(pSysDat),SysParaAddr,sizeof(PWR_HTR));
	
	HINT_DEBUG("addr:0x%0x\r\n",SysParaAddr);
	CurSysParaAddr = SysParaAddr;
	SysParaAddr += sizeof(PWR_HTR);
	
	return PH_OK;
}


#define CALC_INPUT_PWM_CNT   5000
/********************************************************************************
*fun name:          judge_pwm_err 
*fun description:   judge input pwm is err
*input param:       pPwrHtr 
*output param:
*return value:  
*remark:        creat 2016-12-25  guolei
********************************************************************************/
void judge_pwm_err(PWR_HTR *pPwrHtr)
{
	static u8 count = 0;
	
	if(NULL == pPwrHtr)return ;
	if(0 ==SysStat.ticks %CALC_INPUT_PWM_CNT)
	{
		if(POWER_MODE == pPwrHtr->wm)
		{
			if( (POWER_INPUT_PWM_FREQKHZ_MIN < SysStat.PwmFreq )&& \
				(SysStat.PwmFreq < POWER_INPUT_PWM_FREQKHZ_MAX) )
			{
				count = 0;
				SysStat.PwmFreqErr = PWM_FREQ_NO_ERR;
			}
			else
			{
				count++;
				if(count > PWM_ERR_CNT)
				{
					SysStat.PwmFreqErr = PWM_FREQ_OVER;
				}
			}
		}
		else if(HOT_MODE == pPwrHtr->wm)
		{
			if( (HEATER_INPUT_PWM_FREQKHZ_MIN<SysStat.PwmFreq )&& \
				(SysStat.PwmFreq < HEATER_INPUT_PWM_FREQKHZ_MAX) )
			{
				count = 0;
				SysStat.PwmFreqErr = PWM_FREQ_NO_ERR;
			}
			else
			{
				count++;
				if(count > PWM_ERR_CNT)
				{
					SysStat.PwmFreqErr = PWM_FREQ_OVER;
				}
			}
		}
		
		EXTI->IMR|=1<<1;//enable pb1
	}
}
/********************************************************************************
*fun name:          calc_cpu_temp 
*fun description:   calc cpu temp
*input param:       pPwrHtr 
*                   ADC1_channel16 ADC value
*output param:
*return value:  
*remark:        creat 2016-12-24  guolei
********************************************************************************/
void calc_cpu_temp(PWR_HTR *pPwrHtr ,u16 *pInAdValue)
{
	int v25 = 14300;
	int Vtemp = 0;
	int vref = 12000;//unit:0.1mv
	static u8 ITempErrCnt = 0;
	
	if(NULL == pPwrHtr)return ;
	Vtemp = (vref*pInAdValue[0]/pInAdValue[1] );
	SysStat.InTemp = (v25 - Vtemp)/43 + 25;
	if(SysStat.InTemp < ph.pwr.PretecTemp)
	{
		ITempErrCnt = 0;
	}
	else
	{
		ITempErrCnt++;
		SysStat.InTemp  = ph.pwr.PretecTemp;
		if(ITempErrCnt >= INER_TEMP_ERR_CNT)
		{
			SysStat.InTemp  = ph.pwr.PretecTemp+1;
		}
	}
}




/********************************************************************************
*fun name:          read_sys_data 
*fun description:   read system work param
*input param:       pSysDat: system param address
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
PH_ERR read_sys_data(PWR_HTR *pSysDat)
{
	
	u32 ParmAddr = FLASH_USER_START_ADDR-(sizeof(PWR_HTR)+5);
	u32 ParmLastAddr = U32_UNWR_VALUE;
	u16 sum,ToatalWrCnt;
	u32 offset;
	u8 erase = 0;
	u16 i;
		
	if(NULL == pSysDat)return PH_ERR_NULL;
	
	ToatalWrCnt = (PAGE_SIZE/sizeof(PWR_HTR))*SYS_PARAM_SECTIONS;

	for(i=0; i<ToatalWrCnt; i++)
	{
		ParmAddr += sizeof(PWR_HTR);
		offset = (ParmAddr)%PAGE_SIZE;
		//it's no enough space for PWR_HTR struct in a sector
		if(offset &&((PAGE_SIZE-offset) < sizeof(PWR_HTR)))
		{
			//no space in this page must jump next page
			ParmAddr += (PAGE_SIZE-offset);
		}
		
		
		usr_data_read((u32 *)(pSysDat),ParmAddr,sizeof(PWR_HTR));
		sum = pSysDat->sum;
		calc_param_sum(pSysDat);
			
		if((PARAM_VALID == pSysDat->vld) && (sum == pSysDat->sum))
		{
			ParmLastAddr = ParmAddr;
		}
		else
		{
			if(i)
				usr_data_read((u32 *)(pSysDat),ParmLastAddr,sizeof(PWR_HTR));
			break;
		}
		
	}
	if( (ToatalWrCnt-i) < RESERVED_CNT )
	{
		erase = 1;
	}

	if(0==i)
	{
		stat_init(&ph);
		data_default();
		ParmLastAddr = FLASH_USER_START_ADDR;
	}
	if(erase)
	{
		
		erase_all_data();
		write_sys_data(pSysDat);
		if(PH_OK != read_cur_sys_data_valid())
		{
			return PH_FLASH_ERR;
		}
		ParmLastAddr = FLASH_USER_START_ADDR;
	}
	SysParaAddr = ParmAddr;
	CurSysParaAddr = ParmLastAddr;
	
	return PH_OK;	
}




/********************************************************************************
*fun name:          read_adjust_data_valid 
*fun description:   read adjust value
*input param:       no
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
PH_ERR read_adjust_data_valid(void )
{
	u16 sum;
	ADJUST_VALUE adjustDat;

	PH_ERR err = PH_FLASH_ERR;
		
	//return PH_OK;
		usr_data_read((u32 *)(&adjustDat),ADJ_PARAM_START_ADDR,sizeof(ADJUST_VALUE));
		sum = adjustDat.sum;
		calc_adjust_value_sum(&adjustDat);

		if((PARAM_VALID == adjustDat.vld) && (sum == adjustDat.sum))
		{
			err = PH_OK;
		}

		return err;

}




/********************************************************************************
*fun name:          read_cur_sys_data_valid 
*fun description:   read current system work param
*input param:       no
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
PH_ERR read_cur_sys_data_valid(void )
{
	u16 sum;
	PWR_HTR RdDat;

	PH_ERR err = PH_FLASH_ERR;
		
	//return PH_OK;
		usr_data_read((u32 *)(&RdDat),CurSysParaAddr,sizeof(PWR_HTR));
		sum = RdDat.sum;
		calc_param_sum(&RdDat);

		if((PARAM_VALID == RdDat.vld) && (sum == RdDat.sum))
		{
			err = PH_OK;
		}

		return err;

}

/********************************************************************************
*fun name:          key_inactive 
*fun description:   deal with from key active
*input param:       TimeMs: timer when key inactive
*output param:
*return value:  
*remark:        creat 2017-01-07  guolei
********************************************************************************/
void  key_inactive(u32 TimeMs)
{

	if(KEY_ACTIVE == SysStat.KeyAct)
	{
		SysStat.pPwrStat->PwrErr &= ~PWR_VOLT_ERR; 
		
		if(compare_timestamp(SysStat.KeyActMs, TimeMs) )
		{
			SysStat.KeyAct = KEY_INACTIVE;
			write_sys_data(&ph);
			if(PH_OK != read_cur_sys_data_valid())
			{
				SysStat.pPwrStat->PwrErr |= PWR_FLASH_ERR;
			}
			else
			{
				SysStat.pPwrStat->PwrErr &= ~PWR_FLASH_ERR;
			}
		}
		
		
	}
}

extern void print_work_param(PWR_HTR *pWorkParam);
extern u8 print_temp;
/********************************************************************************
*fun name:          cmd_explain 
*fun description:   deal witch cmd
*input param:       cmd
*output param:
*return value:  
*remark:        creat 2016-12-13  guolei
********************************************************************************/
void cmd_explain(u16 cmd)
{
	
	switch(cmd)
	{
		case 'a':
			print_work_param(&ph);
			HINT_DEBUG("\r\n");
			break;
		case 's':
			write_sys_data(&ph);
			break;
		case 'i':
			print_temp =print_temp?0:1;
			break;
		case 'c':
			HINT_DEBUG("set temp:%d\r\n",ph.htr.SetTemp);
			break;
		case 't':
			HINT_DEBUG("temp:%d\r\n",SysStat.InTemp);
			HINT_DEBUG("heater temp:%d\r\n",SysStat.pHtrS->CurTemp);
			HINT_DEBUG("pwm freq:%dkhz\r\n",SysStat.PwmFreq);
			HINT_DEBUG("pwmi:%d  %d\r\n",SysStat.pPwrStat->pcs.pwmi,gCurPwm);
			HINT_DEBUG("pwmu:%d  %d\r\n",SysStat.pPwrStat->pvs.pwmu,gVoltPwm);
			break;
		case 'p':
			HINT_DEBUG("power err:0x%02x\r\n",SysStat.pPwrStat->PwrErr);
			break;
		case 'd':
			SysStat.pPwrStat->pvs.pwmu += 360;
			HINT_DEBUG("pwmu:%d \r\n",SysStat.pPwrStat->pvs.pwmu);
			break;
		case 'h':
			HINT_DEBUG("t:%d r:%d\r\n",SysStat.pHtrS->CurTemp,HeatTime);
			break;
	}
}





