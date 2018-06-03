#ifndef   __ENCRYPT_H__
#define   __ENCRYPT_H__

#include "stddef.h"
#include "stdint.h"


#include "flash.h"

#define FLASH_SECTOR3_SIZE PAGE_SIZE//0x4000 /* Sector 3 size: 16 Kbytes */
#define ADDR_FLASH_SECTOR_3 (FLASH_USER_START_ADDR- FLASH_SECTOR3_SIZE*2)/* Base @ of Sector 3, 16 Kbytes */
// chipID 地址
#define UID0 0X1F
#define UID1 0XFF
//STMF4XX
//#define UID2 0X7A
//#define UID3 0X10
#define UID2 0XF7
#define UID3 0XE8

// chipID 地址在 crc_tmp_data 中的存储坐标
#define UID0_OFFSET 0x23
#define UID1_OFFSET 0x33
#define UID2_OFFSET 0x43
#define UID3_OFFSET_1 0x53
#define UID3_OFFSET_2 0x63
#define UID3_OFFSET_3 0x73
// 通用密码
#define eID0 0xEF
#define eID1 0x3E
#define eID2 0x05
#define eID3 0x69
#define eID4 0xAB
#define eID5 0xC7
#define eID6 0xE5
#define eID7 0x31
#define eID8 0x05
#define eID9 0x59
#define eID10 0xA3
#define eID11 0x07// 通用密码在 crc_tmp_data 中的存储坐标
#define eID0_OFFSET 0x09
#define eID1_OFFSET 0x19
#define eID2_OFFSET 0x29
#define eID3_OFFSET 0x39
#define eID4_OFFSET 0x49
#define eID5_OFFSET 0x59
#define eID6_OFFSET 0x0A
#define eID7_OFFSET 0x1A
#define eID8_OFFSET 0x2A
#define eID9_OFFSET 0x3A
#define eID10_OFFSET 0x4A
#define eID11_OFFSET 0x5A



typedef enum{  
	HAL_OK       = 0x00U,
	HAL_ERROR    = 0x01U,  
	HAL_BUSY     = 0x02U,  
	HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


void calculate_eCODE(u8 *pUID, u8 *peCODE);
void encrypt_code_save(void);
void read_ID(u32 *pUID_dat32);
HAL_StatusTypeDef check_ID(void);


/* end of file */









#endif
