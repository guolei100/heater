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
#include "heater.h"
#include <stdio.h>
#include "timer.h"
#include "key.h"
#include <math.h>
#include "pid.h"




#define MAX_OFFSET               80
#define PID_TEMP_TAP1            5

#define  BALANCE_THRESHOLD      40

#define KP_PARAM                 25
#define KI_PARAM                 3

#define KD_PARAM                 0

#define UPDATE_TIME_MS          100
#define GROWTH_THRESHOLD        1


  



HEATER_STAT HeaterStat;




//static u8 j = 0;
u16 DpData = 0;

u16 HeatTime = 0;
HEATING_STAT Heating = COOLING;
s16 e1 = 0,e2 = 0,e3 = 0;
s16 uk = 0, duk = 0;
u16 kp = 0, ki = 0,kd = 0;
u16 BaseTimeMs[3][10] = { 
	                   {88,  91,  95,  105, 110, 115, 120, 125, 130, 135},
	                   {140, 146, 152, 158, 164, 170, 177, 184, 191, 198},
	                   {205, 213, 221, 229, 237, 245, 254, 263, 272, 281}
	 
	                  };
volatile static u16 PidThreshold = MAX_OFFSET;

static s16 AdjustEnvVolt = 0;
//static s16 EnvTemp = 0;//environment temperature
//index is heater temp  unit:0.1mv
const int HeatTempVolt[601] = {
0   ,
17   ,
34   ,
52   ,
69   ,
86   ,
104  ,
121  ,
138  ,
155  ,
173  ,
190  ,
208  ,
225  ,
243  ,
260  ,
277  ,
295  ,
313  ,
330  ,
348  ,
365  ,
383  ,
400  ,
418  ,
436  ,
453  ,
471  ,
489  ,
506  ,
524  ,
542  ,
560  ,
577  ,
595  ,
613  ,
631  ,
648  ,
666  ,
684  ,
702  ,
720  ,
738  ,
756  ,
773  ,
791  ,
809  ,
827  ,
845  ,
863  ,
881  ,
899  ,
917  ,
935  ,
953  ,
971  ,
989  ,
1007 ,
1025 ,
1043 ,
1061 ,
1079 ,
1097 ,
1115 ,
1133 ,
1151 ,
1169 ,
1188 ,
1205 ,
1224 ,
1242 ,
1260 ,
1278 ,
1296 ,
1314 ,
1332 ,
1350 ,
1368 ,
1387 ,
1404 ,
1423 ,
1441 ,
1459 ,
1477 ,
1495 ,
1513 ,
1531 ,
1549 ,
1567 ,
1585 ,
1604 ,
1621 ,
1640 ,
1658 ,
1676 ,
1694 ,
1712 ,
1730 ,
1748 ,
1766 ,
1784 ,
1802 ,
1820 ,
1838 ,
1856 ,
1874 ,
1892 ,
1910 ,
1928 ,
1946 ,
1964 ,
1982 ,
1999 ,
2018 ,
2036 ,
2053 ,
2071 ,
2089 ,
2107 ,
2125 ,
2143 ,
2161 ,
2178 ,
2196 ,
2214 ,
2232 ,
2249 ,
2267 ,
2285 ,
2303 ,
2320 ,
2338 ,
2356 ,
2373 ,
2391 ,
2409 ,
2427 ,
2444 ,
2462 ,
2480 ,
2498 ,
2515 ,
2532 ,
2550 ,
2568 ,
2586 ,
2603 ,
2620 ,
2638 ,
2656 ,
2673 ,
2691 ,
2708 ,
2726 ,
2743 ,
2761 ,
2778 ,
2796 ,
2813 ,
2831 ,
2848 ,
2866 ,
2883 ,
2900 ,
2918 ,
2936 ,
2953 ,
2971 ,
2988 ,
3005 ,
3023 ,
3040 ,
3058 ,
3075 ,
3092 ,
3109 ,
3127 ,
3144 ,
3162 ,
3179 ,
3197 ,
3214 ,
3231 ,
3249 ,
3266 ,
3284 ,
3301 ,
3318 ,
3335 ,
3353 ,
3370 ,
3388 ,
3405 ,
3423 ,
3440 ,
3457 ,
3475 ,
3492 ,
3510 ,
3527 ,
3544 ,
3562 ,
3579 ,
3596 ,
3614 ,
3631 ,
3649 ,
3666 ,
3683 ,
3701 ,
3719 ,
3736 ,
3754 ,
3771 ,
3788 ,
3806 ,
3823 ,
3841 ,
3859 ,
3876 ,
3893 ,
3911 ,
3928 ,
3946 ,
3963 ,
3981 ,
3998 ,
4016 ,
4034 ,
4051 ,
4069 ,
4086 ,
4104 ,
4122 ,
4139 ,
4157 ,
4174 ,
4192 ,
4210 ,
4227 ,
4245 ,
4263 ,
4280 ,
4298 ,
4315 ,
4333 ,
4351 ,
4369 ,
4386 ,
4404 ,
4422 ,
4439 ,
4457 ,
4475 ,
4493 ,
4510 ,
4528 ,
4546 ,
4564 ,
4581 ,
4599 ,
4617 ,
4635 ,
4653 ,
4671 ,
4689 ,
4706 ,
4724 ,
4742 ,
4760 ,
4778 ,
4796 ,
4814 ,
4831 ,
4849 ,
4867 ,
4885 ,
4903 ,
4921 ,
4939 ,
4957 ,
4975 ,
4993 ,
5011 ,
5029 ,
5047 ,
5065 ,
5083 ,
5101 ,
5118 ,
5137 ,
5155 ,
5172 ,
5191 ,
5209 ,
5226 ,
5245 ,
5263 ,
5281 ,
5299 ,
5317 ,
5335 ,
5353 ,
5371 ,
5389 ,
5407 ,
5425 ,
5443 ,
5461 ,
5479 ,
5498 ,
5516 ,
5534 ,
5552 ,
5570 ,
5588 ,
5606 ,
5624 ,
5642 ,
5661 ,
5679 ,
5697 ,
5715 ,
5733 ,
5751 ,
5770 ,
5788 ,
5806 ,
5824 ,
5842 ,
5861 ,
5878 ,
5897 ,
5915 ,
5933 ,
5951 ,
5969 ,
5988 ,
6006 ,
6024 ,
6042 ,
6060 ,
6079 ,
6097 ,
6115 ,
6134 ,
6152 ,
6170 ,
6188 ,
6206 ,
6225 ,
6243 ,
6261 ,
6279 ,
6298 ,
6316 ,
6334 ,
6353 ,
6371 ,
6389 ,
6408 ,
6426 ,
6444 ,
6462 ,
6481 ,
6499 ,
6517 ,
6536 ,
6554 ,
6572 ,
6590 ,
6609 ,
6627 ,
6645 ,
6664 ,
6682 ,
6700 ,
6718 ,
6737 ,
6755 ,
6774 ,
6792 ,
6810 ,
6829 ,
6847 ,
6865 ,
6884 ,
6902 ,
6921 ,
6939 ,
6957 ,
6975 ,
6994 ,
7012 ,
7031 ,
7049 ,
7067 ,
7086 ,
7104 ,
7123 ,
7141 ,
7159 ,
7178 ,
7196 ,
7214 ,
7233 ,
7252 ,
7270 ,
7288 ,
7307 ,
7325 ,
7343 ,
7362 ,
7380 ,
7399 ,
7417 ,
7436 ,
7454 ,
7472 ,
7491 ,
7509 ,
7528 ,
7546 ,
7565 ,
7583 ,
7602 ,
7620 ,
7639 ,
7657 ,
7675 ,
7694 ,
7712 ,
7731 ,
7749 ,
7768 ,
7786 ,
7805 ,
7823 ,
7842 ,
7860 ,
7879 ,
7897 ,
7916 ,
7934 ,
7953 ,
7971 ,
7990 ,
8008 ,
8027 ,
8045 ,
8064 ,
8082 ,
8101 ,
8119 ,
8138 ,
8156 ,
8175 ,
8193 ,
8212 ,
8230 ,
8249 ,
8267 ,
8286 ,
8304 ,
8323 ,
8342 ,
8360 ,
8379 ,
8397 ,
8416 ,
8434 ,
8453 ,
8471 ,
8490 ,
8508 ,
8527 ,
8545 ,
8564 ,
8582 ,
8601 ,
8619 ,
8638 ,
8656 ,
8675 ,
8693 ,
8712 ,
8731 ,
8749 ,
8768 ,
8786 ,
8805 ,
8824 ,
8842 ,
8861 ,
8879 ,
8898 ,
8916 ,
8935 ,
8953 ,
8972 ,
8990 ,
9009 ,
9028 ,
9046 ,
9065 ,
9083 ,
9102 ,
9121 ,
9139 ,
9158 ,
9176 ,
9195 ,
9213 ,
9232 ,
9250 ,
9269 ,
9287 ,
9306 ,
9325 ,
9343 ,
9362 ,
9381 ,
9399 ,
9418 ,
9436 ,
9455 ,
9473 ,
9492 ,
9510 ,
9529 ,
9548 ,
9566 ,
9585 ,
9604 ,
9622 ,
9641 ,
9659 ,
9678 ,
9696 ,
9715 ,
9733 ,
9752 ,
9770 ,
9789 ,
9808 ,
9826 ,
9845 ,
9864 ,
9882 ,
9901 ,
9919 ,
9938 ,
9956 ,
9975 ,
9993 ,
10012,
10030,
10049,
10067,
10086,
10105,
10123,
10142,
10161,
10179,
10198,
10216,
10235,
10253,
10272,
10290,
10309,
10327,
10346,
10364,
10383,
10401,
10420,
10439,
10457,
10476,
10494,
10513,
10531,
10550,
10568,
10587,
10606,
10624,
10643,
10661,
10680,
10698,
10717,
10735,
10754,
10772,
10791,
10809,
10828,
10846
};

//size > 3 must
int  get_index(int value,const int a[],int size)
{
	int low,high,mid,index = 0;
	//int x,y,z;

	if((NULL == a) || (size <= 3))
	{
		return -1;
	}

	low = 0;
	high = size-1;
	mid=(low+high)>>1;
	while(low<high)
	{
		if(value < a[mid])
		{
			high = mid - 1;
		}
		else
		{
			low = mid + 1;
		}
		mid = (low+high)>>1;
	}
	if(low == high)
	{
		if(0 == low)
		{
			index = 1;
		}
		else if( low == (size-1) )
		{
			index  = size-2;
		}
		else
		{
			index  = low;
		}
		if(value < a[index])
		{
			index = (value < ((a[index]+a[index-1])>>1))?(index-1):index;
		}
		else
		{
			index = (value < ((a[index]+a[index+1])>>1))?index:(index+1);
		}
		
	}
	else//low > high
	{
		
		if(value < a[high])
		{
			index = 0;
			if(high)index = high-1;	
		}
		else
		{
			if(value < a[low])
				index = (value < ((a[low]+a[high])>>1))?high:low;
			else
			{
				index = low;
				if(low != size-1)
					index = (value < ((a[low]+a[low+1])>>1))?low:(low+1);
			}
		}
		
		
	}
	
	return index;
	
}


void heater_display(HEATER *heater)
{
	//u8 UpdisplayCnt = 4;
	//u8 DowndisplayCnt = 4;
	u8 i = 0,j = 0,z=100;

	//static u16 LastTemp = 0;
	u16 curTemp;
	

		
	if(NULL == heater)return ;

	for( i=0;i<8;i++)
	{
		SysStat.pHtrS->HtrLed.SegLedStat[i] = LED_ON;
	}

	key_inactive(1500);

	
	if(HEATER_ADJUST == SysStat.pHtrS->mode)
	{
		SysStat.pHtrS->HtrLed.SegLedStat[0] = LED_FLASH_SLOW;
		SysStat.pHtrS->HtrLed.SegLedStat[1] = LED_FLASH_SLOW;
		SysStat.pHtrS->HtrLed.SegLedStat[2] = LED_FLASH_SLOW;
	}
	
	
	// display unit
	SysStat.pHtrS->HtrLed.data[3] = segmcode[12];

	curTemp = SysStat.pHtrS->DispTemp;
	
#ifdef DISPLAY_THRESHOLD
	if(HEATER_ON == SysStat.pHtrS->HtrOn) 
	{
		if(SysStat.pHtrS->DispTemp > (THRESHOLD_OFFSET+SysStat.pHtrS->adActTemp))
		{
			curTemp = THRESHOLD_OFFSET+SysStat.pHtrS->adActTemp;
		}
	}
#endif

	
	//display temperature
	for(i=0; i<2; i++)
	{
		DpData = (0==i)?heater->SetTemp:curTemp;
		z = 100;
		for(j=0; j<3; j++)
		{
			if(0 == j )
			{
				SysStat.pHtrS->HtrLed.data[j+i*4] = segmcode[DpData/100];
				if(DpData <100)
				{
					SysStat.pHtrS->HtrLed.SegLedStat[j+i*4] = LED_OFF;
					SysStat.pHtrS->HtrLed.data[j+i*4] = 0;//display nothing
				}
			}
			else if(1 == j)
			{
				SysStat.pHtrS->HtrLed.data[j+i*4] = segmcode[DpData/10];
				if((DpData <10) && (LED_OFF == SysStat.pHtrS->HtrLed.SegLedStat[j-1+i*4]))
				{
					SysStat.pHtrS->HtrLed.SegLedStat[j+i*4] = LED_OFF;
					SysStat.pHtrS->HtrLed.data[j+i*4] = 0;
				}
			}
			else
			{
				SysStat.pHtrS->HtrLed.data[j+i*4] = segmcode[DpData]|0x80;
			}
			DpData %= z;
			z /= 10;
			
		}
	}
	
	
	if(HEATER_OFF == SysStat.pHtrS->HtrOn)
	{
		SysStat.pHtrS->HtrLed.data[6] &= ~0x80;//加热点关闭显示
		SysStat.pHtrS->HtrLed.data[7] = segmcode[15];
		
		SysStat.pHtrS->HtrLed.SegLedStat[7] = LED_FLASH_SLOW;
		SysStat.pHtrS->HtrLed.FlashCnt[7] = DISPLAY_ON;
		
	}
	else
	{
		if(HEATER_NO_ERR == SysStat.pHtrS->HtrErr )
		{
			SysStat.pHtrS->HtrLed.data[7] = segmcode[12];;
			// display unit
			if( HEATING == Heating ) 
				SysStat.pHtrS->HtrLed.data[6] |= 0x80;
			else
				SysStat.pHtrS->HtrLed.data[6] &= ~0x80;
		}
	}



	if(HEATER_CUR_OVER_ERR & SysStat.pHtrS->HtrErr )
	{
		SysStat.pHtrS->HtrLed.data[7] = segmcode[10];
		SysStat.pHtrS->HtrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.FlashCnt[7] = DISPLAY_ON;
	}
	if(HEATER_TEMP_BOARD_ERR & SysStat.pHtrS->HtrErr )
	{
		SysStat.pHtrS->HtrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.data[7] = segmcode[12];
		SysStat.pHtrS->HtrLed.FlashCnt[7] = DISPLAY_ON;
	}
	if(HEATER_PWM_FREQ_ERR & SysStat.pHtrS->HtrErr )
	{
		SysStat.pHtrS->HtrLed.data[7] = segmcode[15];
		SysStat.pHtrS->HtrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.FlashCnt[7] = DISPLAY_ON;
	}
	//display "S-E"
	if(HEATER_TEMP_OVER_ERR & SysStat.pHtrS->HtrErr )
	{
		SysStat.pHtrS->HtrLed.data[4] = segmcode[5];
		SysStat.pHtrS->HtrLed.data[5] = segmcode[18];
		SysStat.pHtrS->HtrLed.data[6] = segmcode[14];
		SysStat.pHtrS->HtrLed.data[7] = segmcode[15];
		
		SysStat.pHtrS->HtrLed.SegLedStat[4] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.SegLedStat[5] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.SegLedStat[6] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.FlashCnt[4] = DISPLAY_ON;
		SysStat.pHtrS->HtrLed.FlashCnt[5] = DISPLAY_ON;
		SysStat.pHtrS->HtrLed.FlashCnt[6] = DISPLAY_ON;
		SysStat.pHtrS->HtrLed.FlashCnt[7] = DISPLAY_ON;
	}
	
	
	//display "H-E"
	if(HEATER_TEMP_LOW_ERR & SysStat.pHtrS->HtrErr )
	{
		SysStat.pHtrS->HtrLed.data[4] = segmcode[19];
		SysStat.pHtrS->HtrLed.data[5] = segmcode[18];
		SysStat.pHtrS->HtrLed.data[6] = segmcode[14];
		SysStat.pHtrS->HtrLed.data[7] = segmcode[15];
		SysStat.pHtrS->HtrLed.SegLedStat[4] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.SegLedStat[5] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.SegLedStat[6] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.SegLedStat[7] = LED_FLASH_FAST;
		SysStat.pHtrS->HtrLed.FlashCnt[4] = DISPLAY_ON;
		SysStat.pHtrS->HtrLed.FlashCnt[5] = DISPLAY_ON;
		SysStat.pHtrS->HtrLed.FlashCnt[6] = DISPLAY_ON;
		SysStat.pHtrS->HtrLed.FlashCnt[7] = DISPLAY_ON;
	}
	
	//close display when in sleep	
	if(HEATER_SLEEPED == SysStat.pHtrS->sleep)
	{
		for( i=0;i<8;i++)
		{
			SysStat.pHtrS->HtrLed.data[i] = 0x00;
		}
	}
	else if(HEATER_HALF_SLEEP == SysStat.pHtrS->sleep)
	{
		for( i=0;i<8;i++)
		{
			if(i < 4 )
				SysStat.pHtrS->HtrLed.data[i] = 0x00;
			else
			{
				SysStat.pHtrS->HtrLed.data[i] = segmcode[18];
				SysStat.pHtrS->HtrLed.SegLedStat[i] = LED_FLASH_SLOW;
			}
		}
	}
	
	pSegLed = &SysStat.pHtrS->HtrLed;
	
}



/********************************************************************************
*fun name:          judge_threshold 
*fun description:   judge growth threshold
*input param:       temp:  
                          temperature
                          
*output param:
*return value:      0:  balance
                    TEMP_RAISE:    raise
                    TEMP_REDUCE:   reduce
*remark:        creat 2017-10-20  guolei
********************************************************************************/
s16 judge_threshold(s16 setTemp,s16 curTemp)
{
	static u32 CurTick = 0;
	static s16 lastTemp = 0;
	static u16 cnt = 0;
	
	static s16 res = 0;


		if( (SysStat.Tmr2CntMs-CurTick) >= UPDATE_TIME_MS)
		{
			if(abs(curTemp - lastTemp) >= GROWTH_THRESHOLD)
			{
				cnt++;
				if(cnt >= 2) 
				{
					
					if(curTemp > lastTemp)
					{
						res = 1;
					}
					else
					{
						res = -1;
						printf("ct:%d\r\n",curTemp);
					}
					cnt = 0;
				}
				else
				{
					if( (curTemp < lastTemp) && ((lastTemp - curTemp) > 1) )
					{
						printf("ct_1:%d\r\n",curTemp);
						res = -1;
					}
				}
			}
			else
			{
				if(cnt)
				{
					cnt--;
				}
				else
				{
					res = 0;
				}
			}
			printf("%d %d %d %d\r\n",res,HeatTime,SysStat.Tmr2CntMs/100,curTemp);
			lastTemp = curTemp;
			CurTick = SysStat.Tmr2CntMs;
		}


	return res;
}






/********************************************************************************
*fun name:          pid_init 
*fun description:   pid control init
*input param:      
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void pid_init(void)
{
#if 1	
	e1 = 0 ;
	e2 = 0 ;
	e3 = 0 ;
	kp = KP_PARAM;//FULL_TIME_HEAT/MAX_OFFSET;
	ki = KI_PARAM; 
	kd = KD_PARAM;	
	uk = 0;
	duk = 0;
#else
	e1 = 0 ;
	e2 = 0 ;
	e3 = 0 ;
	kp = 10;//FULL_TIME_HEAT/MAX_OFFSET;
	ki = 3; 
	kd = 0;	
	uk = 0;
	duk = 0;
#endif	
}


void new_pid_init(void)
{
	SetTunings(2.5,  0.5, 8);
//	SetMode(AUTOMATIC);
	SetOutputLimits(0,FULL_TIME_HEAT);
	SetSampleTime(FULL_TIME_HEAT);
}

void new_pid_run(void)
{
	static s8 OutScope = 0;
	

	s16 CTemp = SysStat.pHtrS->CurTemp;
	s16 STemp = SysStat.pHtrS->adActTemp;
	s16 add = 0;
	s16 e = 0;
	
	

	add = judge_threshold(STemp,CTemp);
	//printf("%d %d %d %d\r\n",add,HeatTime,SysStat.Tmr2CntMs/100,CTemp);
	e = STemp - CTemp;
	
	if( -1 == add )//明显下降
	{
		PidThreshold = PID_TEMP_TAP1;
	}
	else if( 0 == add )
	{
		//PidThreshold = MAX_OFFSET;
	}
	else if( 1 == add )//明显上升
	{
		if(abs(e) < 30)
		{
			PidThreshold = MAX_OFFSET;
			HeatTime = 0;
			return;
		}
		
	}
	
	if((STemp - CTemp) >= PidThreshold)
	{
		HeatTime = FULL_TIME_HEAT;
		Output = HeatTime;
		OutScope = 1;
		SetMode(!OutScope);
	}
	else if(CTemp - STemp >= 10)
	{
		HeatTime = 0;//FULL_TIME_HEAT/2;
		Output = HeatTime;
		OutScope = 1;
		SetMode(!OutScope);
	}
	else
	{
		if(1 == OutScope)
		{
			if(STemp > CTemp)
			{
				HeatTime = 0;
				Output = 0;
			}
		}
		OutScope = 0;
		PidThreshold = MAX_OFFSET;
		SetMode(!OutScope);
		HeatTime = Compute(STemp,CTemp,0);
	}
	if(( -1 == add ) && (e > 0) )
	{
		HeatTime = 200;
	}
		
	//HeatTime = uk;
}


/********************************************************************************
*fun name:          calc_actual_temp 
*fun description:   calc actual temp
*input param:       
*output param:
*return value:  
*remark:        creat 2017-7-7  guolei
********************************************************************************/
u16 calc_actual_temp(void)
{
	s16 t1,t2,t3,c1,c2,c3;
	u16 adActTemp;
	
	t1 = adjustDat.htrJ.adjTemp1;
	t2 = adjustDat.htrJ.adjTemp2;
	c1 = adjustDat.htrJ.adjActTemp1;
	c2 = adjustDat.htrJ.adjActTemp2;
	t3 = ph.htr.SetTemp;
	

	if((0==t1)||(0==t2)||(0==c1)||(0==c2))
	{
		adActTemp = ph.htr.SetTemp;
	}
	else
	{
			
		c3 = (c2-c1)*(t3-t1)/(t2-t1)+c1;
		adActTemp = c3;
	}
	return adActTemp;
}

/********************************************************************************
*fun name:          get_actual_temp 
*fun description:   get actual_temp
*input param:       pheater
*output param:
*return value:  
*remark:        creat 2017-7-7  guolei
********************************************************************************/
u16 get_actual_temp(void)
{
	u16 adActTemp;

	if(HEATER_NORMAL == SysStat.pHtrS->mode)
	{
		adActTemp = calc_actual_temp();
	}
	else
	{
		adActTemp = ph.htr.SetTemp;
	}
	return adActTemp;
}




/********************************************************************************
*fun name:          pid_run 
*fun description:   pid control heater
*input param:       pheater
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/

void pid_run(HEATER *pheater)
{
	static u32 PidCnt = 0;

	static s8 OutScope = 0;
//	static u16 BalanceCnt = 0;
	
	static u16 LastSTemp = 0;
	
	//static u32 BalanceTimeStamp = 0;
	//u32 BalanceTimeMs = 0,
//	u32 BalThr;
	//static u16 AccumulativeHeatTime = 0;

//	static s16 BaseOut = 0;
	s16 CTemp = SysStat.pHtrS->CurTemp;
	s16 STemp = SysStat.pHtrS->adActTemp;
	


	//HeatTime = 80;
	//return ;
	
	if(LastSTemp != STemp)
	{
		LastSTemp = STemp;
//		BaseOut = 0;
		//AccumulativeHeatTime = 0;
		
	}
	if(NULL == pheater)return ;

	PidCnt++;
	//PidCnt is 800  pid_run run per 106ms
	//PidCnt is 400  pid_run run per 53ms
	//PidCnt is 200  pid_run run per 26.5ms
	//PidCnt is 100  pid_run run per 13.3ms
	if(PidCnt<20)return;//20
	
	PidCnt = 0;

#if 1
	if( judge_threshold(STemp,CTemp) )
	{
		PidThreshold = PID_TEMP_TAP1;
	}
#endif

	
	if((STemp - CTemp) >= PidThreshold)
	{
		HeatTime = FULL_TIME_HEAT;
		pid_init();//init pid para when out pid
		OutScope = 1;
		uk = HeatTime;
	}
	else if(CTemp - STemp >= PidThreshold)
	{
		HeatTime = 0;//FULL_TIME_HEAT/2;
		pid_init();//init pid para when out pid
		OutScope = 1;
		uk = 0;
	}
	else
	{
		OutScope = 0;
		PidThreshold = MAX_OFFSET;
	}
	e1 = STemp - CTemp;
	if(0 == OutScope)
	{
#if 0		
		if( (-10 <= e1) && (e1 <= 10) )
		{
			ki = KI_PARAM>>1;
		}
		else
		{
			ki = KI_PARAM;
		}
#endif		
		duk = (kp*(e1-e2)+ki*e1+kd*(e1-e2*2+e3))/10;
		PID_DEBUG("t=%-4d e1=%-4d e2=%-4d e3=%-4d duk=%-4d uk=%-4d\r\n",CTemp,e1,e2,e3,duk,uk);
		uk += duk;//不累计历史偏差，只累计上一次偏差
		if(uk < 0)
		{
			uk = 0;
		}
		else if(uk > FULL_TIME_HEAT)
		{
			uk = FULL_TIME_HEAT;
		}
#if 0		
		if((e1 == e2)&& (e2 == e3))
		{
			BalanceCnt++;
		}
		else
		{
			BalanceCnt = 0;
			
		}
		
		if(abs(e1) > 20)
		{
			BalThr = 3;//time= BalThr*80ms
		}
		else if(abs(e1) > 15)
		{
			BalThr = 4;
		}
		else if(abs(e1) > 10)
		{
			BalThr = 6;
		}
		else if(abs(e1) > 8)
		{
			BalThr = 8;
		}
		else if(abs(e1) > 6)
		{
			BalThr = 20;
		}
		else if(abs(e1) > 4)
		{
			BalThr = 40;
		}
		else if(abs(e1) > 3)
		{
			BalThr = 50;
		}
		else 
		{
			BalThr = 60;
		}
		//if(BALANCE_THRESHOLD <= BalanceCnt)
		if(BalThr <= BalanceCnt)
		{
			PID_DEBUG("BaseOut=%d\r\n",BaseOut);
#if 1		
			if(CTemp < STemp)
			{
					
				if((STemp-CTemp) > 10)
				{
					BaseOut += ((STemp-CTemp)/2);
				}
				else if((STemp-CTemp) > 5)
				{
					BaseOut += ((STemp-CTemp)/4);
				}
				else
				{
					BaseOut++;
				}				
			}
			else if(STemp < CTemp)
			{
				if(BaseOut)BaseOut--;
			}
#endif			
		}
		else
		{
			if(abs(STemp-CTemp) < 10)
			{
				BaseOut = BaseTimeMs[STemp/100 -2][(STemp/10)%10]-20;
				PID_DEBUG("Bt=%d\r\n",BaseOut);
			}
			
		}
		
		
		//若要维持一个温度值，这个值用以抵消散去的热量
		//Stemp scope:200~450
		//BaseOut = 0;//BaseTimeMs[STemp/100 -2][(STemp/10)%10]-20;
		uk += BaseOut;
		if(uk > FULL_TIME_HEAT)
		{
			uk = FULL_TIME_HEAT;
		}
		else if(uk < 0)
		{
			uk = 0;
		}
		HeatTime = uk;
	}
	else
	{
		BaseOut =10;
	}
#endif	
	HeatTime = uk;
}
	e3 = e2;
	e2 = e1;
	
	
	
}


/********************************************************************************
*fun name:          heater_sleep_detect 
*fun description:   when PB8 level not change in 10 minute
*input param:       pheater: heater
*                   AdValue  heater temp ad value
*output param:
*return value:  
*remark:        creat 2016-12-27  guolei
********************************************************************************/
void heater_sleep_detect(HEATER*pheater)
{
	static u32 Timer2CntMs = 0;
	u32 TimerMs;
	static u16 SetTemp = 0;
	//static u8 LastStat = 0xff;
	//u8 stat;

	//if(COOLING == SysStat.pHtrS->heating)return ;

	//stat = GET_HEATER_SLEEP_IO;
	//if(0xff == LastStat)
	
	//if((LastStat != stat)|| (HEATER_WAKING == SysStat.pHtrS->sleep))
	if(HEATER_WAKING == SysStat.pHtrS->sleep)
	{
		
		//recover set temperature in half sleep
		pheater->SetTemp = SetTemp;
		SysStat.pHtrS->adActTemp = get_actual_temp();
		Timer2CntMs = SysStat.Tmr2CntMs;
		//LastStat = stat;
	}
	
	if(SysStat.pHtrS->intoSleep)
	{
		if(SysStat.Tmr2CntMs >= HEATER_HALF_SLEEP_MS)
		{
			Timer2CntMs =  SysStat.Tmr2CntMs - HEATER_HALF_SLEEP_MS;
			
		}
		else
		{
			__disable_irq();
			SysStat.Tmr2CntMs += (HEATER_HALF_SLEEP_MS-SysStat.Tmr2CntMs);
			Timer2CntMs = 0;
			__enable_irq();
		}
		SysStat.pHtrS->intoSleep = 0;
	}
	if(SysStat.Tmr2CntMs >= Timer2CntMs)
	{
		TimerMs = SysStat.Tmr2CntMs - Timer2CntMs;
	}
	else
	{
		TimerMs = 0xffffffff - Timer2CntMs+SysStat.Tmr2CntMs;
	}

	

	if(TimerMs > HEATER_SLEEP_MS)
	{
		if(HEATER_SLEEPED != SysStat.pHtrS->sleep)
		{
			SysStat.pHtrS->sleep = HEATER_SLEEPED;
			pheater->SetTemp = SetTemp;
			SysStat.pHtrS->adActTemp = get_actual_temp();
		}
		SysStat.pHtrS->heating = COOLING;
	}
	else if(TimerMs >= HEATER_HALF_SLEEP_MS)
	{
		if(HEATER_HALF_SLEEP != SysStat.pHtrS->sleep)
		{
			SetTemp = pheater->SetTemp;
			if(HEATER_OFF == SysStat.pHtrS->HtrOn)
			{
				SysStat.pHtrS->sleep = HEATER_SLEEPED;
			}
			else
			{
				pheater->SetTemp = HEATER_HALF_SLEEP_TEMP;
				SysStat.pHtrS->adActTemp = get_actual_temp();
				SysStat.pHtrS->sleep = HEATER_HALF_SLEEP;
			}
			
			SysStat.pHtrS->heating = HEATING;
		}
	}
	else
	{
		SysStat.pHtrS->sleep = HEATER_WAKEN;
		SysStat.pHtrS->heating = HEATING;
		SetTemp = pheater->SetTemp;
		//SetTemp = SysStat.pHtrS->adActTemp;
	}
	

	
	
}


/********************************************************************************
*fun name:          test_heating 
*fun description:   control heater temp  temporary
*input param:       pheater: heater
*                   AdValue  heater temp ad value
*output param:
*return value:  
*remark:        creat 2016-12-27  guolei
********************************************************************************/
#if 0
u8 Kp = FULL_TIME_HEAT/MAX_OFFSET;
void test_heating(HEATER*pheater)
{
	//static u8 stat = 0;
	//static u16 LastHeatTime = HeatTime;
	s16 CTemp = SysStat.pHtrS->CurTemp;
	s16 STemp = pheater->SetTemp;
	s16 OffsetTemp = STemp - CTemp;
	static u32  PidCnt = 0;

	PidCnt++;
	if(PidCnt<800)return;
	if(OffsetTemp > MAX_OFFSET)
	{
		HeatTime = FULL_TIME_HEAT;
	}
	else if(OffsetTemp < -MAX_OFFSET )
	{
		HeatTime = 0;
	}
	else 
	{
		if(OffsetTemp < 0 )OffsetTemp = - OffsetTemp;
		HeatTime = Kp*OffsetTemp;
	}
	//PID_DEBUG("t:%d\r\n",HeatTime);
	
#if 0
	if(COOLING == SysStat.pHtrS->heating)return ;
	if(SysStat.>pHtrS->CurTemp > pheater->ht.SetTemp+5)
	{
		SysStat.pHtrS->heating = COOLING;
		stat = 1;
	}
	
	if((stat) && (SysStat.pHtrS->CurTemp > pheater->ht.SetTemp-5))
	{
		SysStat.pHtrS->heating = COOLING;
	}

	if(SysStat.pHtrS->CurTemp < pheater->ht.SetTemp-5)
	{
		stat = 0;
	}
#endif	
	
}
#endif



void adjust_heater_temp(s8 temp)
{
	AdjustEnvVolt = HeatTempVolt[temp];
	//EnvTemp = temp;
}
/********************************************************************************
*fun name:          heat_alarm 
*fun description:   judge heater alarm
*input param:       no
*                   
*output param:
*return value:  
*remark:        creat 2017-7-2  guolei
********************************************************************************/
void heat_alarm(void)
{
	static u32 TempOverCnt = 0;
	static u32 MinTempTime = 0;
	static u32 NormalTemp;
	

	//judge alarm which display "S-E" when heater heating
	if( SysStat.pHtrS->CurTemp > HEATER_FLOAT_TEMP )
	{
		SysStat.pHtrS->CurTemp = NormalTemp;
		TempOverCnt++;
		if(TempOverCnt >= 160)//
		{
			SysStat.pHtrS->HtrErr |= HEATER_TEMP_OVER_ERR;
		}
	}
	else if(SysStat.pHtrS->CurTemp > ALARM_TEMP)
	{
		TempOverCnt++;
		if(TempOverCnt >= 8000)//80000----->10600ms
		{
			SysStat.pHtrS->HtrErr |= HEATER_TEMP_OVER_ERR;
			TempOverCnt = 0;
		}
	}
	else
	{
		NormalTemp = SysStat.pHtrS->CurTemp;
		TempOverCnt = 0;
		
		SysStat.pHtrS->HtrErr &= ~HEATER_TEMP_OVER_ERR;
	}

	//judge alarm which current over
	if(SysStat.pHtrS->TCurOCnt)
	{
		if( (SysStat.Tmr2CntMs - SysStat.pHtrS->CurOvertime ) < 500)
		{
			return ;//close heat(in EXTI3_IRQHandler )  but not do nothing when happen current over 
		}
		if( (SysStat.Tmr2CntMs-SysStat.pHtrS->alarmClock) > 1500 )
		{
			if(SysStat.pHtrS->TCurOCnt >= 3)
			{
				SysStat.pHtrS->HtrErr |= HEATER_CUR_OVER_ERR;
			}
			else
			{
				SysStat.pHtrS->TCurOCnt = 0;
				SysStat.pHtrS->alarmClock = 0;
				SysStat.pHtrS->CurOvertime = 0;
			}
		}
	}
	
	//judge alarm which board temp high
	if(SysStat.InTemp > MAX_INTER_TEMP)
	{
		SysStat.pHtrS->HtrErr |= HEATER_TEMP_BOARD_ERR;
	}
	else
	{
		SysStat.pHtrS->HtrErr &= ~HEATER_TEMP_BOARD_ERR;
	}

	//judge alarm which input pwm
	judge_pwm_err(&ph);
		if(PWM_FREQ_OVER == SysStat.PwmFreqErr )
			SysStat.pHtrS->HtrErr |= HEATER_PWM_FREQ_ERR;

		//SysStat.pHtrS->HtrErr &= ~HEATER_PWM_FREQ_ERR;

		//judge alarm which display "H-E"
	if(SysStat.pHtrS->CurTemp <= HEATER_MIN_TEMP)
	{
		if(MinTempTime)
		{
			if((SysStat.Tmr2CntMs-MinTempTime) >= HEATER_TEMP_LOW_TIMER)
				SysStat.pHtrS->HtrErr |= HEATER_TEMP_LOW_ERR;
		}
		else
		{
			MinTempTime = SysStat.Tmr2CntMs;
		}
		if(HEATER_SLEEPED == SysStat.pHtrS->sleep)
		{
			MinTempTime = 0;
		}
	}
	else
	{
		MinTempTime = 0;
	}
	if(HEATER_OFF == SysStat.pHtrS->HtrOn) 
	{
		MinTempTime = 0;
	}

}
/********************************************************************************
*fun name:          test_heater_alarm 
*fun description:   test heater alarm
*input param:       no
*                   
*output param:
*return value:  
*remark:        creat 2017-7-2  guolei
********************************************************************************/
void test_heater_alarm(void)
{
#ifdef  TEST_HEATER
	static u32 TestTimems = 0;
#endif	
	
#ifdef  TEST_HEATER
	if((SysStat.Tmr2CntMs - TestTimems) > 70000)
	{
		TestTimems = SysStat.Tmr2CntMs;
		SysStat.pHtrS->sleep = HEATER_WAKEN;
	}
	else if((SysStat.Tmr2CntMs - TestTimems) > 60000)
	{
		SysStat.pHtrS->sleep = HEATER_SLEEPED;
	}
	else if((SysStat.Tmr2CntMs - TestTimems) > 50000)
	{
		SysStat.pHtrS->HtrErr = HEATER_CUR_OVER_ERR;
	}
	else if((SysStat.Tmr2CntMs - TestTimems) > 40000)
	{
		SysStat.pHtrS->HtrErr = HEATER_TEMP_LOW_ERR;
	}
	else if((SysStat.Tmr2CntMs - TestTimems) > 30000)
	{
		SysStat.pHtrS->HtrErr = HEATER_TEMP_OVER_ERR;
	}
	else if((SysStat.Tmr2CntMs - TestTimems) > 20000)
	{
		SysStat.pHtrS->HtrErr = HEATER_PWM_FREQ_ERR;
	}
	else if((SysStat.Tmr2CntMs - TestTimems) > 10000)
	{
		SysStat.pHtrS->HtrErr = HEATER_TEMP_BOARD_ERR;
	}				
#endif

}

/********************************************************************************
*fun name:          test_heater_alarm 
*fun description:   test heater alarm
*input param:       no
*                   
*output param:
*return value:  
*remark:        creat 2017-7-2  guolei
********************************************************************************/
s16 get_heater_temp(u16 *pAdValue)
{
	s16 value = 0;
	u32 temp = 0;
	s16 AdVolt = 0;//unit :0.1mv
	
	
	
	u32 vref = 12000;//unit:0.1mv
	
	////calc heater temp
	//temp = vref*50*pAdValue[0]/pAdValue[2];
	
	//calc AD volt(unit:0.1mv)
	temp = pAdValue[0]*vref/pAdValue[2];

	AdVolt = temp;

	value = AdjustEnvVolt + AdVolt;
	//get temperature of current tempertature between to no heat  计算当前温度值与没有加热时之差
	value = get_index(value,HeatTempVolt,sizeof(HeatTempVolt)/sizeof(HeatTempVolt[0]));

	
	//芯片内部温度至少比真正环境温度少4度
	value -= 4;
	if(value < 0 )value = 0;
	
	return value;
}


/********************************************************************************
*fun name:          calc_display_temp 
*fun description:   calc diplay temp
*input param:       no
*                   
*output param:
*return value:  
*remark:        creat 2017-7-16  guolei
********************************************************************************/
u16 calc_display_temp(u16 curTemp,u16 AdTemp,u16 lastDispTemp)
{
	u16 curDispTemp = 0;
	static u8 cnt = 0;

	
	if(curTemp < AdTemp)
	{	
		if(lastDispTemp <= curTemp)
		{
			curDispTemp = curTemp;
		}
		else
		{
			if( curTemp >= lastDispTemp-3)
			{
				curDispTemp = curTemp;
			}
			else
			{
				cnt++;
				if(cnt >= 5)
				{
					curDispTemp = lastDispTemp + ((AdTemp-curTemp)>>4);
					if(curDispTemp > AdTemp)
					{
						curDispTemp = AdTemp;
					}
					cnt = 0;
				}
				else
				{
					curDispTemp = lastDispTemp;
				}
			}
#if 0			
			if((lastDispTemp -((AdTemp-curTemp)>>4)) <= curTemp)
			{
				curDispTemp = curTemp;
			}
			else
			{
				curDispTemp = lastDispTemp -((AdTemp-curTemp)>>4);
			}
#endif			
		}
	}
	else if(curTemp == AdTemp)
	{
		if(lastDispTemp < AdTemp)
		{
			cnt++;
			if( cnt >= 30 )
			{
				if(SysStat.InTemp <= lastDispTemp)
				{
					curDispTemp = lastDispTemp - 1;
				}
				else
				{
					curDispTemp = lastDispTemp;
				}
				cnt = 0;
			}
			else
			{
				curDispTemp = lastDispTemp;
			}
		}
		else
		{
			curDispTemp = curTemp;
		}
	}
	else
	{
		curDispTemp = curTemp;
	}
	return curDispTemp;
}


/********************************************************************************
*fun name:          heater_ctrl 
*fun description:   control heater work
*input param:       pheater: heater
*                   pAdValue[0]  heater  ad value
*                   pAdValue[2]  vref ad value
*output param:
*return value:  
*remark:        creat 2016-12-05  guolei
********************************************************************************/
void heater_ctrl(HEATER*pheater,u16 *pAdValue,u8 update)
{
	u16 temGap = 0;
	//u8 flash = 0;
	static u32 timeStamp = 0;
	
	
	if(NULL == pheater)return ;
	
	HEATER_MODE_CTR;
	if(pheater->SetTemp < HEATER_HALF_SLEEP_TEMP)
	{
		pheater->SetTemp = HEATER_HALF_SLEEP_TEMP;
	}

	SysStat.pHtrS->CurTemp = get_heater_temp(pAdValue);
	SysStat.pHtrS->AdTemp = SysStat.pHtrS->CurTemp;
	if(0xffff == SysStat.pHtrS->DispTemp)
	{
		SysStat.pHtrS->DispTemp = SysStat.pHtrS->CurTemp;
	}

	

	if( (HEATER_NORMAL == SysStat.pHtrS->mode) && \
		(HEATER_ON == SysStat.pHtrS->HtrOn) && \
		(adjustDat.htrJ.adjActTemp1) && \
		(adjustDat.htrJ.adjActTemp2) && \
		(SysStat.pHtrS->CurTemp < HEATER_FLOAT_TEMP))
	{
		if(SysStat.pHtrS->adActTemp > pheater->SetTemp)
		{
			temGap = SysStat.pHtrS->adActTemp - pheater->SetTemp;
		}
		else
		{
			temGap = pheater->SetTemp - SysStat.pHtrS->adActTemp;
		}
		if(SysStat.pHtrS->CurTemp > temGap)
		{
			SysStat.pHtrS->CurTemp -= temGap;
		}
		
	}
	if( ((SysStat.Tmr2CntMs - timeStamp) > 100) || (0 ==timeStamp) )
	{
#ifdef OPEN_ADJUST_TEMP_DISPLAY
		
		if(SysStat.pHtrS->sleep >= HEATER_HALF_SLEEP)
		{
			SysStat.pHtrS->DispTemp = calc_display_temp(SysStat.pHtrS->CurTemp,
			                            SysStat.pHtrS->AdTemp,
			                            SysStat.pHtrS->DispTemp);
		}
		else
		{
			SysStat.pHtrS->DispTemp = SysStat.pHtrS->AdTemp;
		}
#else

	SysStat.pHtrS->DispTemp = SysStat.pHtrS->AdTemp;

#endif
		timeStamp = SysStat.Tmr2CntMs;
	}
#if 0
	//temp gap of current temp and settemp than THRESHOLD_OFFSET 
	if(SysStat.pHtrS->CurTemp > pheater->ht.SetTemp)
	{
		if(SysStat.pHtrS->CurTemp - pheater->ht.SetTemp > THRESHOLD_OFFSET )
		{
			flash = 1;
		}
	}
	else
	{
		if(pheater->ht.SetTemp - SysStat.pHtrS->CurTemp > THRESHOLD_OFFSET )
		{
			flash = 1;
		}
	}
	
	if(flash ||((SysStat.Tmr2CntMs  - timeStamp) > 5000) || (0==timeStamp) )
	{
		heater_display(pheater);
		timeStamp = SysStat.Tmr2CntMs;
	}
#endif
	heater_display(pheater);

	if(HEATER_NORMAL == SysStat.pHtrS->mode)
	{
#if OPEN_HEATER_SLEEEP
		heater_sleep_detect(pheater);
#endif
#if OPEN_HEATER_ALARM		
		heat_alarm();
#endif
		if(HEATER_OFF == SysStat.pHtrS->HtrOn) 
		{
			//timeStamp = 0;
			POWER_HEATER_OFF;
			return ;
		}
	}

	SysStat.pHtrS->CurTemp += temGap;
	//test_heater_alarm();
	if((HEATER_NO_ERR ==SysStat.pHtrS->HtrErr) && update)
	{
#ifdef  OLD_PID		
		pid_run(pheater);
#else		
		new_pid_run();
#endif
	}
		
}


/********************************************************************************
*fun description:   heater param of adjust init
*input param:    
*output param:
*return value:
********************************************************************************/
void heater_adjust_parm_init(void)
{
	adjustDat.htrJ.adjTemp1 = ADJUST_TEMP1;
	adjustDat.htrJ.adjActTemp1 = ACTUAL_TEMP1_DEFAULT;
	adjustDat.htrJ.adjTemp2 = ADJUST_TEMP2;
	adjustDat.htrJ.adjActTemp2 = ACTUAL_TEMP2_DEFAULT;
}


/********************************************************************************
*fun description:   PA6 and PA7 config pwm
*input param:    
*output param:
*return value:
********************************************************************************/
void heater_init(HEATER *pheater)
{
	if(NULL==pheater)return;
	pheater->SetTemp = HEATER_HALF_SLEEP_TEMP;
	SysStat.pHtrS->CurTemp = 20;
	POWER_HEATER_OFF;
}













/********************************************************************************
*fun name:          heater_stat_init 
*fun description:   init heater stat
*input param:       pheater: pheater param address
*output param:
*return value:  
*remark:        creat 2017-01-06  guolei
********************************************************************************/
void heater_stat_init(HEATER *pheater)
{
	u8 i = 0;
	
	if(NULL == pheater)return ;

	//HEATER_SET
	if(pheater->SetTemp > HEATER_SET_MAX_TEMP)
		pheater->SetTemp = HEATER_SET_MAX_TEMP;

	//HEATER_STAT
	SysStat.pHtrS->CurTemp = 0;
	
	SysStat.pHtrS->HtrErr = HTR_NORMAL;
	SysStat.pHtrS->HtrOn = HEATER_OFF;
	SysStat.pHtrS->heating = COOLING;
	SysStat.pHtrS->sleep = HEATER_WAKEN;
	SysStat.pHtrS->mode = HEATER_NORMAL;
	SysStat.pHtrS->key2Cnt = 0;
	SysStat.pHtrS->TCurOCnt = 0;
	SysStat.pHtrS->CurOvertime = 0;
	SysStat.pHtrS->key2Tick = 0;
	SysStat.pHtrS->DispTemp = 0xffff;
	SysStat.pHtrS->intoSleep = 0;

	for(i=0;i<8;i++)
	{
		SysStat.pHtrS->HtrLed.data[i] = 0x00;
		SysStat.pHtrS->HtrLed.SegLedStat[i] = LED_OFF;
	}
#ifdef  OLD_PID	
	pid_init();
#else	
	new_pid_init();
#endif
}



