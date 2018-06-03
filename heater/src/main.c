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
#include "stm32f10x_rcc.h"
#include "stm32_eval.h"
#include <stdio.h>

#include "SegLed.h"
#include "key.h"
#include "timer.h"
#include "power.h"
#include "heater.h"
#include "pwm.h"
#include "ad.h"
#include "ph.h"
#include "uart.h"
#include "Flash.h"
#include "pwm.h"
#include "universal_key.h"
#include "encrypt.h"






#if 0
typedef enum
{ GPIO_Mode_AIN = 0x0,         //模拟输入
  GPIO_Mode_IN_FLOATING = 0x04,//浮空输入
  GPIO_Mode_IPD = 0x28,        //下拉输入
  GPIO_Mode_IPU = 0x48,        //上拉输入
  GPIO_Mode_Out_OD = 0x14,     //开漏输出
  GPIO_Mode_Out_PP = 0x10,     //推挽输出
  GPIO_Mode_AF_OD = 0x1C,      //复用开漏输出
  GPIO_Mode_AF_PP = 0x18       //复用推挽输出
}GPIOMode_TypeDef;
#endif





/********************************************************************************
*fun name:          bubble_sort 
*fun description:  bubble sort
*input param:      
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
void bubble_sort(u16 *a,int n) 
{

	int i,j,temp;

	for(i=0;i<n-1;i++)
		for(j=i+1;j<n;j++) 
			if( a[i] > a[j] )
			{
				temp=a[i];
				a[i  ]=a[j];
				a[j]=temp;
			}

}






void RCC_cfg(void)

{
       //定义错误状态变量
       //ErrorStatus HSEStartUpStatus;

       //将RCC寄存器重新设置为默认值
       RCC_DeInit();

       //打开外部高速时钟晶振
       //RCC_HSEConfig(RCC_HSE_ON);
		//enable Internal High Speed oscillator (HSI) 
	   RCC_HSICmd(ENABLE);
	   while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

       //等待外部高速时钟晶振工作
       //HSEStartUpStatus = RCC_WaitForHSEStartUp();
       //if(HSEStartUpStatus == SUCCESS)
       if(1)
       {
              //set AHB prescaler
              RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB =SYSCLK

              //set APB2 prescaler
              RCC_PCLK2Config(RCC_HCLK_Div1);

              //set APB1 prescaler
              RCC_PCLK1Config(RCC_HCLK_Div2);

              //设置FLASH代码延时
              FLASH_SetLatency(FLASH_Latency_2);
         

              //使能预取指缓存
              FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

			  //pll clk = HSI*12/2   HSI:8M    PLL CLK = 24M  stm32f1008 
			  //Maximum Clock Frequency 24 MHz 
				RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_6);
			  
              //使能PLL
              RCC_PLLCmd(ENABLE);
              //等待PLL准备就绪
              while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
 
              //set SYSCLK = PLL
              RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

              //判断PLL是否是系统时钟
              while(RCC_GetSYSCLKSource() != 0x08);

       }
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA| \
										RCC_APB2Periph_AFIO,ENABLE);
  		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
}


#define  KEY2_TIME_OUT   3200000
#define  AVG_CNT         5
#define  DIS_PWM         1




/********************************************************************************
*fun name:          IWDG_Init 
*fun description:   Iwdg init  Tout=((4*2^prer)*rlr)/40 (ms)
*input param:       prer 
*                   rlr
*output param:
*return value:  
*remark:        creat 2017-07-02  guolei
********************************************************************************/


/**
 * 初始化独立看门狗
 * prer:分频数:0~7(只有低 3 位有效!)
 * 分频因子=4*2^prer.但最大值只能是 256!
 * rlr:重装载寄存器值:低 11 位有效.
 * 时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).
 */
void IWDG_Init(u8 prer,u16 rlr)
{
#if ENABLE_WATCH	
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); /* 使能对寄存器IWDG_PR和IWDG_RLR的写操作*/
    IWDG_SetPrescaler(prer);    /*设置IWDG预分频值:设置IWDG预分频值*/
    IWDG_SetReload(rlr);     /*设置IWDG重装载值*/
    IWDG_ReloadCounter();    /*按照IWDG重装载寄存器的值重装载IWDG计数器*/
    IWDG_Enable();        /*使能IWDG*/
#endif	
}
void close_IWDG(void)
{
#if ENABLE_WATCH	
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable); 
#endif
}

/********************************************************************************
*fun name:          IWDG_Feed 
*fun description:   feed iwdg
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-07-02  guolei
********************************************************************************/
/**
 * 喂独立看门狗
 */
void IWDG_Feed(void)
{
#if ENABLE_WATCH
    IWDG_ReloadCounter();    /*reload*/
#endif
}




/********************************************************************************
*fun name:          adjust_vol_cur 
*fun description:   adjust volt and current 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2017-07-02  guolei
********************************************************************************/
void adjust_vol_cur(void)
{
	//u32  i = 40000;
	u32 i = 0;
	u8 j = 0;
	u8 z = 0,flag = 0;
#ifdef   DIS_PWM
	u8 Dpwm = 1;
#endif	
	u8 key;
	u32 temp = 0;
	s32 pwm = 0;
	DISPLAY_DATA Adjust;
	u16 AdcChValue[ADC1_CHANNEL_CNT];
	u8 dat[5] = {0,0,0,0,0};;
	
	


	
	
	
	
#if  1

	key = key_scan();
	
	if((KEY1_DOWN != key))return ;
	
	temp = SysStat.Tmr2CntMs;
	while(SysStat.Tmr2CntMs - temp < 4000)
	{
		key = key_scan();
		if(KEY1_LONG == key)break;
	}	
	if((KEY1_LONG != key))return ;
	
	i = 0;
	while(1)
	{
		Adjust.data[i] = segmcode[15];
		Adjust.SegLedStat[i]= LED_ON;
		display(&Adjust);
		key = key_scan();
		if( NOKEY != key )
		{
			if(KEY2_UP & key)break;
			
		}
		if(i++ >= 8)i = 0;
	}
	for(i=0; i<8; i++)
	{
		Adjust.data[i] = 0;
		Adjust.SegLedStat[i]= LED_OFF;
		display(&Adjust);
	}
	i = KEY2_TIME_OUT;
	j = 0;
	ADJUST_DEBUG("in key2 press\r\n");
	while(i)
	{
		close_display();
		key = key_scan();
		if( NOKEY != key )
		{
			if(KEY2_DOWN & key)
			{
				j++;
				i = KEY2_TIME_OUT;
			}
		}
		if(3 == j)
		{
			i = KEY2_TIME_OUT;
			break;
		}
		i--;
		//Delayms(2);
	}
#endif	
	ADJUST_DEBUG("out key2 press\r\n");
	if(0 != i)
	{		
		i = 0;
		ph.wm = ADJUST_MODE;
		POWER_MODE_CTR;

		//adjust 0V
		POWER_HEATER_OFF;
		pwm_u_duty(0);
		pwm_i_duty(0);
		while(get_adc_data(AdcChValue));
		while(get_adc_data(AdcChValue));
		adjustDat.pwrJ.VAd[INDEX_0V] = AdcChValue[1];
		adjustDat.pwrJ.AAd[INDEX_0A] = AdcChValue[0];


#ifndef   DIS_PWM
		Adjust.SegLedStat[0]= LED_OFF;
#endif

		
		POWER_HEATER_ON;
		
		while(1)
		{
			switch(i)
			{
				case 0://adjust 0.1v
				    SysStat.AdjustPwmType = ADUST_PWMU;
					
					pwm = adjustDat.pwrJ.VPwmSet[INDEX_0_1V];
					SysStat.pPwrStat->pvs.pwmu = pwm;
					pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
					
					pwm_i_duty(PWMI_DEFAULT_3);

					
					Adjust.SegLedStat[1]= LED_ON;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
					
#ifdef   DIS_PWM
					Adjust.data[1] = segmcode[0]|0x80;
					Adjust.data[2] = segmcode[1];
					Adjust.data[3] = segmcode[16];
#else					
					Adjust.data[0] = segmcode[0]|0x80;
					Adjust.data[1] = segmcode[0];
					Adjust.data[2] = segmcode[1];
					Adjust.data[3] = segmcode[16];
					Adjust.SegLedStat[0]= LED_ON;
					
#endif					

					break;
				case 1://adjust 1v
					adjustDat.pwrJ.VPwmSet[INDEX_0_1V] = SysStat.pPwrStat->pvs.pwmu;
					
					SysStat.pPwrStat->pvs.pwmu = adjustDat.pwrJ.VPwmSet[INDEX_1V];
					pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);

#ifdef   DIS_PWM
					Adjust.data[2] = segmcode[1];
					Adjust.data[3] = segmcode[16];
					Adjust.SegLedStat[1]= LED_OFF;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
#else
					Adjust.data[2] = segmcode[1];
					Adjust.data[3] = segmcode[16];
					Adjust.SegLedStat[1]= LED_OFF;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
#endif
					break;
				case 2://adjust 60v
					adjustDat.pwrJ.VPwmSet[INDEX_1V] = SysStat.pPwrStat->pvs.pwmu;//save 1v pwm

					SysStat.pPwrStat->pvs.pwmu = adjustDat.pwrJ.VPwmSet[INDEX_60V];
					pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
					
					
#ifdef   DIS_PWM
					Adjust.data[1] = 0;
					Adjust.data[2] = segmcode[6];
					Adjust.data[3] = segmcode[0];
					Adjust.SegLedStat[1]= LED_OFF;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
#else					
					Adjust.data[1] = segmcode[6];
					Adjust.data[2] = segmcode[0];
					Adjust.data[3] = segmcode[16];
#endif
		
					break;
				case 3://adjust 0.01A
					adjustDat.pwrJ.VPwmSet[INDEX_60V] = SysStat.pPwrStat->pvs.pwmu;
					SysStat.AdjustPwmType = ADUST_PWMI;

					//set volt 5v when adjust current
					temp = (VOLT_5V-ADJUST_VOL)*(adjustDat.pwrJ.VPwmSet[INDEX_60V]-adjustDat.pwrJ.VPwmSet[ADJUST_INDEX])/(MAX_VOL-ADJUST_VOL);
					SysStat.pPwrStat->pvs.pwmu = adjustDat.pwrJ.VPwmSet[ADJUST_INDEX] + temp;				
					pwm_u_duty(SysStat.pPwrStat->pvs.pwmu);
#if 0
					pwm = (CUR_0_1A-CUR_0_0_1A)*(adjustDat.pwrJ.APwmSet[INDEX_3A]-\
					adjustDat.pwrJ.APwmSet[INDEX_0_1A])/(MAX_CUR-CUR_0_1A);
					SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_0_1A] - pwm;
					pwm = SysStat.pPwrStat->pcs.pwmi;
#endif
					SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_0_0_1A];
					pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
					
#ifdef   DIS_PWM
					Adjust.data[0] = segmcode[0]|0x80;
					Adjust.data[1] = segmcode[0];
					Adjust.data[2] = segmcode[1];
					Adjust.data[3] = segmcode[10];
					Adjust.SegLedStat[0]= LED_ON;
					Adjust.SegLedStat[1]= LED_ON;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
#else						
					Adjust.data[4] = segmcode[0]|0x80;
					Adjust.data[5] = segmcode[0];
					Adjust.data[6] = segmcode[1];
					Adjust.data[7] = segmcode[10];
					Adjust.SegLedStat[4]= LED_ON;
					Adjust.SegLedStat[5]= LED_ON;
					Adjust.SegLedStat[6]= LED_ON;
					Adjust.SegLedStat[7]= LED_ON;
#endif	

					break;
				case 4://adjust 0.1A
					adjustDat.pwrJ.APwmSet[INDEX_0_0_1A] = SysStat.pPwrStat->pcs.pwmi;
					SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_0_1A];
					pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);
#ifdef   DIS_PWM
					Adjust.data[1] = segmcode[0]|0x80;
					Adjust.data[2] = segmcode[1];
					Adjust.data[3] = segmcode[10];
					Adjust.SegLedStat[1]= LED_ON;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
#else	
					Adjust.data[5] = segmcode[0]|0x80;
					Adjust.data[6] = segmcode[1];
					Adjust.data[7] = segmcode[10];
					Adjust.SegLedStat[5]= LED_OFF;
					Adjust.SegLedStat[6]= LED_ON;
					Adjust.SegLedStat[7]= LED_ON;
#endif	
					break;

				case 5://adjust 3A
					//save 0.1A pwm
					adjustDat.pwrJ.APwmSet[INDEX_0_1A] = SysStat.pPwrStat->pcs.pwmi;

					SysStat.pPwrStat->pcs.pwmi = adjustDat.pwrJ.APwmSet[INDEX_3A];
					pwm_i_duty(SysStat.pPwrStat->pcs.pwmi);

#ifdef   DIS_PWM
					Adjust.data[1] = 0;
					Adjust.data[2] = segmcode[3];
					Adjust.data[3] = segmcode[10];
					Adjust.SegLedStat[1]= LED_OFF;
					Adjust.SegLedStat[2]= LED_ON;
					Adjust.SegLedStat[3]= LED_ON;
#else						
					Adjust.data[6] = segmcode[3];
					Adjust.data[7] = segmcode[10];
					Adjust.SegLedStat[6]= LED_ON;
					Adjust.SegLedStat[7]= LED_ON;
#endif	
					break;
				case 6://adjust end
					adjustDat.pwrJ.APwmSet[INDEX_3A] = SysStat.pPwrStat->pcs.pwmi;
					//adjustDat.pwrJ.Apwmb = 0;
					Adjust.data[6] = 0;
					Adjust.data[7] = 0;
					i = 8;
					while(i--)
					display(&Adjust);
					write_adjust_data(&adjustDat);
					if(PH_OK != read_adjust_data_valid())
					{
						Adjust.data[0] = 0;
						Adjust.data[1] = 0;
						Adjust.data[2] = 0;
						Adjust.data[3] = 0;
						Adjust.data[4] = 0;
						Adjust.data[5] = segmcode[14];
						Adjust.data[6] = segmcode[17];
						Adjust.data[7] = segmcode[17];
						while(1)
							display(&Adjust);
					}
					ADJUST_DEBUG("\r\nwrite param to flash \r\n");
				default:
					pwm_i_duty(0);
					pwm_u_duty(0);
					ADJUST_DEBUG("\r\n60vduty:%d 0.1vduty:%d 3Aduty:%d 0.1Aduty:%d\r\n",\
						adjustDat.pwrJ.VPwmSet[INDEX_60V],\
						adjustDat.pwrJ.VPwmSet[ADJUST_INDEX],
						adjustDat.pwrJ.APwmSet[INDEX_3A],\
						adjustDat.pwrJ.APwmSet[INDEX_0_1A]);
					
					return ;
			}
			//display(&Adjust);
			real_delay_ms(100);
			while(1)
			{
				get_adc_data(AdcChValue);
				//Cal_volt_Cur(SysStat,pVolt);
				key = key_scan();
				if(KEY1_DOWN ==key)
				{	
					break;
				}
			#if 0	
				if(0 ==  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3))
				{
					real_delay_ms(20);
					if(0 ==  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3))
					{
						break;
					}
				}
			#endif	
				
#ifdef DIS_PWM
				if(KEY2_DOWN ==key)
				{
					Dpwm = Dpwm?0:1;
				}
	#if 0			
				if(0 ==  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9))
				{
					real_delay_ms(20);
					if(0 ==  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9))
					{
						Dpwm = Dpwm?0:1;
					}
				}
	#endif			
				if(i<6)
				{
					Adjust.SegLedStat[0]= LED_ON;
					Adjust.SegLedStat[4]= LED_ON;
					Adjust.SegLedStat[5]= LED_ON;
					Adjust.SegLedStat[6]= LED_ON;
					Adjust.SegLedStat[7]= LED_ON;
					if(Dpwm)
					{
						if(i<3)
							temp = SysStat.pPwrStat->pvs.pwmu;
						else
							temp = SysStat.pPwrStat->pcs.pwmi;
					}
					else
					{
						if(i<3)
							temp = AdcChValue[1];
						else
							temp = AdcChValue[0];;
					}


					for( z=5; z; z--)
					{
						dat[z-1] = temp%10;
						temp /= 10;
					}
					flag = 0;
					if(dat[0])
					{
						flag = 1;
						Adjust.data[0] = segmcode[ dat[0] ];
					}
					else
					{
						Adjust.data[0] = 0;
					}
					
					for(z=1; z<5; z++)
					{
						if(dat[z])
						{
							flag = 1;
							Adjust.data[3+z] = segmcode[ dat[z] ];
						}
						else
						{
							if(flag)
							{
								Adjust.data[3+z]  = segmcode[ dat[z] ];
							}
							else
							{
								Adjust.data[3+z]  = 0;
							}
						}
						
					}
					if(0==flag)
					{
						Adjust.data[7]  = segmcode[ 0 ];
					}
					if(3 == i)
					{
						Adjust.data[0] = segmcode[0]|0x80;
						Adjust.SegLedStat[0]= LED_ON;
					}
				}		
#endif
				display(&Adjust);		
				scan_encoder();
			}
			//adjust volt & current display value
		
			while(get_adc_data(AdcChValue));
			if(i < 3)
			{
				adjustDat.pwrJ.VAd[i] = AdcChValue[1];
			}
			else if(i < 6)
			{
				adjustDat.pwrJ.AAd[i-3] = AdcChValue[0];
			}
			
			i++;
		}
	}
}






//#define	 TEST_IO   1
/********************************************************************************
*fun name:          long_press_key1 
*fun description:   deal with long press key1 
*input param:                                    
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void task(void)
{
	u8 key = 0;
	u8 update = 0;
/*********************
*   AdcChValue[0]: current
*   AdcChValue[1]: volt
*   AdcChValue[2]: heater temp
*   AdcChValue[3]: board temp
*   AdcChValue[4]: vref
*********************/
	
  	static u16 AdcChValue[ADC1_CHANNEL_CNT]; 

    //update per 57~58ms
	update = get_adc_data(AdcChValue);	
	key = key_scan();
	
	if(NOKEY != key)
		deal_key(key);
	if(key2UpTimeStamp)
	{
		if( compare_timestamp(key2UpTimeStamp,300) )
		{
			short_press_key2();
			key2UpTimeStamp = 0;
		}
	}


	
	display(pSegLed);
	if(POWER_MODE == ph.wm)
	{
		SysStat.pHtrS->DispTemp = 0xffff;
		power_ctrl(&ph.pwr,&AdcChValue[0]);
	}
	else
	{
		heater_ctrl(&ph.htr,&AdcChValue[2],!update);
	}
	
	calc_cpu_temp(&ph,&AdcChValue[3]);
	judge_pwm_err(&ph);
	display(pSegLed);
	SysStat.ticks++;
}

#if  0
static DISPLAY_DATA version = {


	.SegLedStat = {LED_ON,LED_ON,LED_ON,LED_ON,LED_OFF,LED_OFF,LED_OFF,LED_OFF},
	.FlashCnt = {0,0,0,0,0,0,0,0},
	//   V                  0.            0          1
	.data = {segmcode[16],segmcode[0]|0x80,segmcode[0],segmcode[16],0,0,0,0},
	.UpUnit = 0,
	.DownUnit = 0
};
#endif
static DISPLAY_DATA version = {


	{LED_ON,LED_ON,LED_ON,LED_ON,LED_OFF,LED_OFF,LED_OFF,LED_OFF},
	{0,0,0,0,0,0,0,0},
	   
	//   V                  0.             0          8
	//{segmcode[16],segmcode[0]|0x80,segmcode[0],segmcode[16],0,0,0,0},
	//{0x3e,          0x3f|0x80,         0x3f,       0x06,    0,0,0,0},
	{0x77,          0x3f|0x80,         0x3f,       0x7f,    0,0,0,0},//test version
	0,
	0
};
#if 0
u8 display_bad(void)
{
	u8 i;
	static u32 badStamp = 0;

	badStamp = badStamp?badStamp:SysStat.Tmr2CntMs;
	for( i=0;i<8;i++)
	{
		SysStat.pHtrS->HtrLed.SegLedStat[i] = LED_ON;
		SysStat.pHtrS->HtrLed.data[i] = segmcode[i];
	}
	pSegLed = &SysStat.pHtrS->HtrLed;
	display(pSegLed);
	if( compare_timestamp(badStamp,500))
	{
		return 1;
	}
	return 0;
}
#endif
//指定变量到指定地址
//u32 var_locate __attribute__((at(0x22000000)));
//void  __attribute__ ((section ("ERASE_PROGRAM_ROM"))) portect_right(void) 
void portect_right(void)
{
	while(1)
	{
		//if(display_bad())
		{
			close_IWDG();
			erase_program();
			
			//FLASH_EraseAllPages();
		}
	}
}
int main(void)
{
  //u8 key = 0;
  u16 AdcV[ADC1_CHANNEL_CNT]; 
  
  u32 tick;
  u32 heaterTemp  = 0;
  PH_ERR ReadParaErr;
  u8 openHandleIrq = 0;
  	//SystemInit();	
  	RCC_cfg();
	key_init();//init io
	real_delay_ms(300);
	
	SegLedInit();
	all_display();//display 8888 8888
	PanakeyVar_Init();//key driver init
	time_init();//timer init
	
#ifndef CLOSE_DEBUG	
  	uart1_init();
	uart1_cfg(115200);
#endif	
	HINT_DEBUG("system start\r\n");
  	ReadParaErr = read_sys_data(&ph);//read system para
	read_adjust_data(&adjustDat);
	SysStat.pHtrS->adActTemp = ph.htr.SetTemp;

	tick = sizeof(PWR_HTR);
	pwm_init();
	
	
	
	IWDG_Init(6,1024);//256*1000/40=6400ms=6.4s

	ad_init();

	

	
	//__enable_irq();
	//wait get correct temp
	tick = SysStat.Tmr2CntMs;
	while(SysStat.Tmr2CntMs - tick < 1000);
  	while(SysStat.Tmr2CntMs - tick < 2000)
  	{
		display(&version);
		get_adc_data(AdcV);
		heaterTemp = get_heater_temp(&AdcV[2]);
		calc_cpu_temp(&ph,&AdcV[3]);
		if(SysStat.InTemp < ph.pwr.PretecTemp)
		{
			//HINT_DEBUG("in temp:%d\r\n",SysStat.InTemp);
			adjust_heater_temp(SysStat.InTemp);
			//break;
		}
		
		
  	}


	timer_init(get_tick,0xffffffff);
	stat_init(&ph);
	
	if(PH_FLASH_ERR == ReadParaErr)
	{
		SysStat.pPwrStat->PwrErr |= PWR_FLASH_ERR;
	}
	adjust_vol_cur();
	//print_work_param(&ph);
	

	//close output in heat or power mode	
	POWER_HEATER_OFF;
	if(ADJUST_MODE == ph.wm )ph.wm = POWER_MODE;



#if 0
	pwm_i_duty(12000);
	pwm_u_duty(6000);
	while(1);
#endif
		

#if 0
	ph.wm = POWER_MODE;
	SysStat.pPwrStat->PwrOn = POWER_ON;
	ph.pwr.pv.SetVol = 3000;
	adjustDat.pwrJ.VPwmSet[INDEX_60V] = PWM_MAX_VALUE;
#endif

#if 0
	ph.wm = HOT_MODE;
	ph.htr.ht.SetTemp = 204;
	SysStat.pHtrS->HtrOn = HEATER_OFF;
	SysStat.pHtrS->heating = HEATING;
#endif	
	HINT_DEBUG("PWR_HTR size:%d\r\n",sizeof(PWR_HTR));
	HINT_DEBUG("adj-------->act\r\n");
	HINT_DEBUG("%d--------->%d\r\n",adjustDat.htrJ.adjTemp1,adjustDat.htrJ.adjActTemp1);
	HINT_DEBUG("%d--------->%d\r\n",adjustDat.htrJ.adjTemp2,adjustDat.htrJ.adjActTemp2);
	
	SysStat.pPwrStat->PwrErr = PWR_NO_ERR;

	if( (POWER_MODE == ph.wm) )
	{
		SysStat.pPwrStat->PwrOn = POWER_OFF;
		pwm_change_io();
		SysStat.pPwrStat->DisSetValue = 1;
		SysStat.pPwrStat->DisSetValueMs = 1000;
		SysStat.pPwrStat->DisSetValueStamp = SysStat.Tmr2CntMs;
		display_step(0);
	}
	else
	{
		heaterTemp = (heaterTemp + get_heater_temp(&AdcV[2])) >> 1;
		SysStat.pHtrS->HtrOn = HEATER_OFF;
		pwm_i_duty(DUTY_CYCLE_90);
		pwm_u_duty(DUTY_CYCLE_5);
		if( heaterTemp > HEATER_FLOAT_TEMP )
		{
			SysStat.pHtrS->HtrErr |= HEATER_TEMP_OVER_ERR;
			SysStat.pHtrS->heaterFloat = 1;
		}
		SysStat.pHtrS->adActTemp = get_actual_temp();
	}

	if(HOT_MODE == ph.wm)
	{
		into_heater();
	}
	if(ph.pwr.PwrOn)
	{
		if(POWER_MODE == ph.wm)
		{
			POWER_MODE_CTR;
			POWER_HEATER_OFF;
			SysStat.pPwrStat->pvs.pwmu = 0;
			SysStat.pPwrStat->pcs.pwmi = 0;
			//LastUduty = 0;
			//LastIduty = 0;
			pwm_u_duty(0);
			pwm_i_duty(0);
			pwm_change_io();
			real_delay_ms(1000);
			open_power();
			//SysStat.pPwrStat->PwrOn = POWER_ON;
		}
		else
		{
			SysStat.pHtrS->HtrOn = HEATER_ON;
			SysStat.pHtrS->heating = HEATING;
			POWER_HEATER_ON;
		}
	}
	while(1)
	{
		if((0==openHandleIrq) &&(SysStat.Tmr2CntMs > 30000))
		{
			open_heater_handle_irq();
			openHandleIrq = 1;
		}
		scan_encoder();
		task();
		IWDG_Feed();
		
		if(HAL_ERROR == check_ID())
		{
			portect_right();
		}
	}
	
}

#if 0
void Get_SerialNum(void) 
{ 
  u32 Device_Serial0, Device_Serial1, Device_Serial2;	  
  Device_Serial0 = *(vu32*)(0x1FFFF7E8); 
  Device_Serial1 = *(vu32*)(0x1FFFF7EC); 
  Device_Serial2 = *(vu32*)(0x1FFFF7F0);    
} 
#endif


