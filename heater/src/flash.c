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
#include "flash.h"
#include <stdio.h>
#include "stm32f10x_flash.h"
#include "key.h"







/********************************************************************************
*fun name:          erase_all_data 
*fun description:   erase all saved data   flash start adress of FLASH_USER_START_ADDR-1024
*input param:       sum: sum of PWR_HTR
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
void erase_all_data(void)
{
	u8 i = 0;
	u32 addr = 0;

	//FLASH_DEBUG("erase flash\r\n");
	FLASH_Unlock();
	for(i=0; i<(SYS_PARAM_SECTIONS);i++)
	{
		addr = (FLASH_USER_START_ADDR+i*1024);
		if(FLASH_COMPLETE != FLASH_ErasePage(addr))
		{
			FLASH_DEBUG("erase flash fail\r\n");
		}
		else
		{
			//FLASH_DEBUG("erase flash sucessfully\r\n");
		}
	}
	FLASH_Lock();
}


/********************************************************************************
*fun name:          erase_program 
*fun description:   erase program when read from other chips
*input param:      
*output param:
*return value:  
*remark:        creat 2018-05-13  guolei
********************************************************************************/
void erase_program(void)
{
	u8 i = 0;
	u32 addr = 0;

	//FLASH_DEBUG("erase flash\r\n");
	FLASH_Unlock();
	//for(i=0; i<(CODE_SECTIONS);i++)
	//从程序的中间区域擦除
	for(i=(CODE_SECTIONS); i>0;i--)
	{
		addr = (FLASH_BASE+i*1024);
		if(FLASH_COMPLETE != FLASH_ErasePage(addr))
		{
			FLASH_DEBUG("erase flash fail\r\n");
		}
		else
		{
			//FLASH_DEBUG("erase flash sucessfully\r\n");
		}
	}
	FLASH_Lock();
}



/********************************************************************************
*fun name:          erase_adjust_data 
*fun description:   erase adjust data   flash start adress of FLASH_ADJUST_VALUE_START_ADDR
*input param:       
*output param:
*return value:  
*remark:        creat 2017-07-18  guolei
********************************************************************************/
void erase_adjust_data(void)
{
	
	FLASH_Unlock();
	FLASH_ErasePage(ADJ_PARAM_START_ADDR);
	FLASH_Lock();
}

/********************************************************************************
*fun description:   read data from Flash for user
*input param:        
*output param:
*return value:
********************************************************************************/
void usr_data_read(u32 *pDat,u32 address_t,u32 len)
{
	 int i;
	 //u32 address_t;

	 if(NULL == pDat) return;
	 
	 //address_t = FLASH_USER_START_ADDR+offset;
	 for(i=0;i<len/4 ;i++)
	 {
	  pDat[i] = *(__IO u32 *)address_t;
	  address_t += 4;
	 }
}
/********************************************************************************
*fun name:          read_half_word 
*fun description:   read len bytes from flash per half word 
*input param:       address: read from flash start address 
*                   len: read bytes
*output param:      pDat:  data read from flash saved
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
void read_half_word(u16 *pDat,u32 address,u32 len)
{
	 int i;


	 if(NULL == pDat) return;
	 
	 //address_t = FLASH_USER_START_ADDR+offset;
	 for(i=0;i<len/2 ;i++)
	 {
	  pDat[i] = *(__IO u16 *)address;
	  address += 2;
	 }
}


/********************************************************************************
*fun name:          write_sum 
*fun description:   write sum to flash adress of FLASH_USER_START_ADDR-1024
*input param:       sum: sum of PWR_HTR
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
#if 0
int write_sum(u16 sum, int relocate)
{
	static u32 address = FLASH_SAVE_CNT_START_ADDR;
	int ret = 0;

	/*
		sum start addr   ---------
                           a sector for save sum
		                 ---------
		                   a sector for save sum
sysetem parm save addr   ---------
		                  a sector for save sum
		                 ---------
                        
	*/
	if(((address+sizeof(sum)) > FLASH_USER_START_ADDR) || relocate)
	{
		//FLASH_DEBUG("flash full\r\n");
		address = FLASH_SAVE_CNT_START_ADDR;
		ret = 1;
	}
	
	 //Unlock the Flash to enable the flash control register access
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	
	FLASH_ProgramHalfWord(address,sum);
	SaveCntAddr = address;
	address += sizeof(sum);
	FLASH_Lock();
	
	return ret;
}
#endif
/********************************************************************************
*fun description:   write user data to in Flash
*input param:       
*output param:
*return value:
********************************************************************************/
void usr_data_write(u32 *pDat,u32 address_t,u32 len)
{
	 int i;
	 //u32 address_t;

	 if(NULL == pDat)return ;
	 //Unlock the Flash to enable the flash control register access
	 FLASH_Unlock();
	 FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	 // Erase the FLASH pages
	 //usr_data_erase();  //FLASH_ErasePage(FLASH_USER_START_ADDR);
	 // Program the user Flash area word by word
	 //address_t = FLASH_USER_START_ADDR+offset;
	 for(i=0;i<len/4 ;i++)
	 {
		  FLASH_ProgramWord(address_t, pDat[i]);
		  address_t += 4;
	 }
	 // Lock the Flash to disable the flash control register access (recommended
	 //     to protect the FLASH memory against possible unwanted operation)
	 FLASH_Lock();
}

void set_flash_read_out_protection(void)
{
   if (FLASH_GetReadOutProtectionStatus()!=SET) 
   { 
        /* 会擦除Flash */
        FLASH_Unlock(); //写保护时可以不用这句话，可用可不用
        FLASH_ReadOutProtection(ENABLE); 
    }

}

