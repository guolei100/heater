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
//#include <math.h>
#include "pid.h"



#define    GAIN     1000


/*working variables*/ 
unsigned long lastTime = 0; 
int Input, Output, Setpoint; 
int ITerm, lastInput; 
static int kp, ki, kd; 
int SampleTime = 1000;
int outMin, outMax; 
u8 inAuto = 0; 
  
 
  

u16 Compute(int SetTemp,int CurTemp) 
{ 
   
   unsigned long now = SysStat.Tmr2CntMs;//millis(); 
   int error;
   int dInput;
   int timeChange = (now - lastTime); 

	if(!inAuto) return 0; 
	Setpoint = SetTemp;
	Input = CurTemp;
	if(timeChange>=SampleTime) 
	{ 
	  /*Compute all the working error variables*/ 
	  error = Setpoint - Input; 
	  
	  ITerm += (ki * error); 
	  
	  if(ITerm> outMax) ITerm= outMax; 
	  else  if(ITerm<  outMin)  ITerm = outMin;
	  
	  dInput = (Input - lastInput);

	  /*Compute PID Output*/ 
	  Output = kp * error + ITerm- kd * dInput;
	  Output /= 1000;
	  if(Output> outMax) Output = outMax;
	  else if(Output < outMin) Output = outMin; 

	  /*Remember some variables for next time*/ 
	  lastInput = Input; 
	  lastTime = now; 
	} 
	return (u16)(Output);
} 
  
void SetTunings(int  Kp, int  Ki, int Kd) 
{ 
	
#if 0	
	int  SampleTimeInSec  = ((int)SampleTime)/1000; 
	
	kp = Kp; 
	ki = Ki * SampleTimeInSec; 
	kd = Kd / SampleTimeInSec; 
#endif
	kp = Kp; 
	ki = Ki; 
	kd = Kd; 
} 
  
void SetSampleTime(int NewSampleTimeMs) 
{ 
	if (NewSampleTimeMs > 0) 
	{ 

	  //int  ratio   = GAIN*NewSampleTimeMs/SampleTime; 
	 
	  ki = ki*NewSampleTimeMs/SampleTime;
	  
	  //kd = kd *GAIN*GAIN;
	  
	  kd = kd*SampleTime/NewSampleTimeMs; 
	  SampleTime  =  (unsigned long)NewSampleTimeMs; 
	} 
} 
  
void SetOutputLimits(int  Min, int Max) 
{ 
	if(Min > Max) return; 
	outMin = Min; 
	outMax = Max; 

	if(Output > outMax) Output = outMax; 
	else if(Output < outMin) Output = outMin; 

	if(ITerm> outMax) ITerm= outMax; 
	else if(ITerm< outMin) ITerm= outMin;
} 


void Initialize(void) 
{ 
	lastInput = Input; 
	ITerm = Output; 
	if(ITerm> outMax) ITerm= outMax; 
	else if(ITerm< outMin) ITerm= outMin;


}

  
void SetMode(int Mode) 
{ 
	u8 newAuto = (Mode == AUTOMATIC); 
	
	if(newAuto && !inAuto) 
	{  /*we just went from manual to auto*/
	    Initialize(); 
	} 
	inAuto = newAuto; 
} 
  














