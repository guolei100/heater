#ifndef __FLASH_H__
#define __FLASH_H__

/**********************************
*****
*****  code  section
*****
***********************************
*****
*****  adjust param section
*****
***********************************
*****
*****  system param section
*****
***********************************/

#define TOTAL_SECTIONS     64
#define PAGE_SIZE                 ((u32)0x00000400)  
  
#define SYS_PARAM_SECTIONS       30
#define ADJ_PARAM_SECTIONS       1

#define CODE_SECTIONS            (TOTAL_SECTIONS-(SYS_PARAM_SECTIONS+ADJ_PARAM_SECTIONS))

//0x08008400
#define FLASH_USER_START_ADDR   ((u32)(FLASH_BASE+ (CODE_SECTIONS+ADJ_PARAM_SECTIONS)*1024))


#define ADJ_PARAM_START_ADDR    ((u32)(FLASH_BASE+ CODE_SECTIONS*1024))

#define FLASH_USER_END_ADDR     ((u32)(FLASH_BASE+ TOTAL_SECTIONS*1024)) 

#define U8_UNWR_VALUE         0xff
#define U16_UNWR_VALUE        0xffff
#define U32_UNWR_VALUE        0xffffffff




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






/********************************************************************************
*fun description:   erase user data
*input param:    
*output param:
*return value:
********************************************************************************/
extern void usr_data_erase(void);
/********************************************************************************
*fun name:          erase_all_data 
*fun description:   erase all saved data   flash start adress of FLASH_USER_START_ADDR-1024
*input param:       sum: sum of PWR_HTR
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
extern void erase_all_data(void);


/********************************************************************************
*fun name:          erase_program 
*fun description:   erase program when read from other chips
*input param:      
*output param:
*return value:  
*remark:        creat 2018-05-13  guolei
********************************************************************************/
extern void  erase_program(void);


/********************************************************************************
*fun name:          write_sum 
*fun description:   write sum to flash adress of FLASH_USER_START_ADDR-1024
*input param:       sum: sum of PWR_HTR
*output param:
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
extern int write_sum(u16 sum, int relocate);
/********************************************************************************
*fun description:   read data from Flash for user
*input param:       
*output param:
*return value:
********************************************************************************/
extern void usr_data_read(u32 *pDat,u32 offset,u32 len);

/********************************************************************************
*fun name:          read_half_word 
*fun description:   read len bytes from flash per half word 
*input param:       address: read from flash start address 
*                   len: read bytes
*output param:      pDat:  data read from flash saved
*return value:  
*remark:        creat 2017-01-15  guolei
********************************************************************************/
extern void read_half_word(u16 *pDat,u32 address,u32 len);
/********************************************************************************
*fun description:   write user data to in Flash
*input param:       
*output param:
*return value:
********************************************************************************/
extern void usr_data_write(u32 *pDat,u32 offset,u32 len);


extern void set_flash_read_out_protection(void);
extern void test_flash(void);

#endif
