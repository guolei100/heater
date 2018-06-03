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


/*working variables*/ 
unsigned long lastTime = 0; 
float Input, Output, Setpoint; 
float ITerm, lastInput; 
static float kp, ki, kd; 
int SampleTime = 1000;
float outMin, outMax; 
u8 inAuto = 0; 
  
 
  

u16 Compute(float SetTemp,float CurTemp) 
{ 
   
   unsigned long now = SysStat.Tmr2CntMs;//millis(); 
   float error;
   float dInput;
   int timeChange = (now - lastTime); 

	if(!inAuto) return; 
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
	  if(Output> outMax) Output = outMax;
	  else if(Output < outMin) Output = outMin; 

	  /*Remember some variables for next time*/ 
	  lastInput = Input; 
	  lastTime = now; 
	} 
	return (u16)(Output);
} 
  
void SetTunings(float  Kp, float  Ki, float Kd) 
{ 
	float  SampleTimeInSec  = ((float)SampleTime)/1000; 
	
	kp = Kp; 
	ki = Ki * SampleTimeInSec; 
	kd = Kd / SampleTimeInSec; 
} 
  
void SetSampleTime(int NewSampleTimeMs) 
{ 
	if (NewSampleTimeMs > 0) 
	{ 

	  float  ratio   = (float)NewSampleTimeMs/(float)SampleTime; 
	  ki *= ratio; 
	  kd /= ratio; 
	  SampleTime  =  (unsigned long)NewSampleTimeMs; 
	} 
} 
  
void SetOutputLimits(float  Min, float Max) 
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
  














