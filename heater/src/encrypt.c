
#include "stm32f10x.h"

#include "encrypt.h"




// 密码存储区
const unsigned char encrypt_code[FLASH_SECTOR3_SIZE] __attribute__((at(ADDR_FLASH_SECTOR_3))) = {
eID0, eID1, eID2, eID3, eID4, eID5, eID6, eID7, eID8, eID9, eID10, eID11,
};
// 通用密码和 CRC 计算数据
const unsigned char crc_tmp_data[128]={
// 0 1 2 3 4 5 6 7 8 9 A B C D E F
/*0*/ 0x39, 0x18, 0xbb, 0x5a, 0xdd, 0x21, 0xff, 0xee, 0x62, eID0, eID6, 0x14, 0x66, 0x77, 0x44, 0x55,
/*1*/ 0x2b, 0x90, 0x21, 0x32, 0x42, 0x54, 0x67, 0x76, 0xea, eID1, eID7, 0x89, 0xfe, 0xef, 0x2c, 0xcd,
/*2*/ 0x00, 0x21, 0x22, UID0, 0x44, 0x55, 0xc6, 0x77, 0x88, eID2, eID8, 0xbb, 0xcc, 0x3a, 0xce, 0x2c,
/*3*/ 0x31, 0x10, 0x32, UID1, 0x55, 0x44, 0xf7, 0x66, 0x34, eID3, eID9, 0x01, 0xc3, 0x67, 0x54, 0x05,
/*4*/ 0xaa, 0xbb, 0x88, UID2, 0xf7, 0xff, 0xcc, 0x8d, 0x33, eID4, eID10,0x00, 0xd7, 0x66, 0x55, 0xb4,
/*5*/ 0x5b, 0xaa, 0x99, UID3, 0xdf, 0xf7, 0xdd, 0xfe, 0xff, eID5, eID11,0xcf, 0xbb, 0x3a, 0x99, 0x78,
/*6*/ 0x88, 0xa9, 0xca,UID3+4,0xdc, 0x2d, 0xfe, 0x6f, 0x2c, 0xa1, 0x30, 0x20, 0x50, 0x25, 0x76, 0x3a,
/*7*/ 0x48, 0x55, 0x87,UID3+8,0x08, 0x61, 0x22, 0x23, 0x66, 0x2c, 0x4c, 0x55, 0x2c, 0x33, 0x00, 0x41,
};
// 本机 ID 存储变量
volatile union {
u32 dat32[3];
u8 dat8[12];
}mcuID;
// 本机密码存储变量
volatile u8 eCODE[12];
/*
函数名称: calculate_eCODE
功 能: 计算本机密码
(本函数当前没有进行复杂的算法设计,只简单地进行按位取反)
输 入: pUID,UID 数组指针
输 出: peCODE,本机密码
返 回: 无*/
void calculate_eCODE(u8 *pUID, u8 *peCODE)
{
	peCODE[0] = ~pUID[0];
	peCODE[1] = ~pUID[1];
	peCODE[2] = ~pUID[2];
	peCODE[3] = ~pUID[3];
	peCODE[4] = ~pUID[4];
	peCODE[5] = ~pUID[5];
	peCODE[6] = ~pUID[6];
	peCODE[7] = ~pUID[7];
	peCODE[8] = ~pUID[8];
	peCODE[9] = ~pUID[9];
	peCODE[10]= ~pUID[10];
	peCODE[11]= ~pUID[11];
}
/*
函数名称: encrypt_code_save
功 能: 保存密码到 FLASH 中
输 入: 无
返 回: 无
*/
void encrypt_code_save(void)
{
#if  0
	u32 PAGEError = 0;
	u32 write_addr;
	u32 i;
	u8 *pbuf;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.Sector = FLASH_SECTOR_3;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange= FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	if (HAL_OK == HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)) {
	write_addr = ADDR_FLASH_SECTOR_3;
	pbuf = (u8 *)eCODE;
	for (i=0; i<12; i++) {
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, write_addr + i, pbuf[i]);
	}
	}
	HAL_FLASH_Lock();
#endif	
	//u32 *pdata = (u8 *)eCODE;
	FLASH_Unlock();
	
	if(FLASH_COMPLETE != FLASH_ErasePage(ADDR_FLASH_SECTOR_3))
	{
		//FLASH_DEBUG("erase flash fail\r\n");
	}
	else
	{
		//FLASH_DEBUG("erase flash sucessfully\r\n");
	}
	FLASH_Lock();
	usr_data_write((u32 *)(eCODE),ADDR_FLASH_SECTOR_3,sizeof(eCODE));
}
/*
函数名称: read_ID
功 能: 读取 ID
输 入: 无
输 出: 读到的 UID 存入 pUID_dat32 指向的存储空间
返 回: 无
*/
void read_ID(u32 *pUID_dat32)
{
	u32 mcuID_add;u32 tmp[4];
	// 31:0
	// 为了更安全,不使用用立即数访问 UID,
	// 所以将 UID 的地址分散在 crc_tmp_data 中,然后通过读取 crc_tmp_data 合成 UID 地址
	tmp[0] = *(__IO u8*)&crc_tmp_data[UID0_OFFSET];
	tmp[1] = *(__IO u8*)&crc_tmp_data[UID1_OFFSET];
	tmp[2] = *(__IO u8*)&crc_tmp_data[UID2_OFFSET];
	tmp[3] = *(__IO u8*)&crc_tmp_data[UID3_OFFSET_1];
	mcuID_add = (tmp[0]<<24) + (tmp[1]<<16) + (tmp[2]<<8) + tmp[3]; // 合成 UID 地址
	pUID_dat32[0] = *(__IO u32*)(mcuID_add); // 读取 UID[31:0]
	// 63:32
	tmp[0] = *(__IO u8*)&crc_tmp_data[UID0_OFFSET];
	tmp[1] = *(__IO u8*)&crc_tmp_data[UID1_OFFSET];
	tmp[2] = *(__IO u8*)&crc_tmp_data[UID2_OFFSET];
	tmp[3] = *(__IO u8*)&crc_tmp_data[UID3_OFFSET_2];
	mcuID_add = (tmp[0]<<24) + (tmp[1]<<16) + (tmp[2]<<8) + tmp[3];
	pUID_dat32[1] = *(__IO u32*)(mcuID_add);
	// 95:64
	tmp[0] = *(__IO u8*)&crc_tmp_data[UID0_OFFSET];
	tmp[1] = *(__IO u8*)&crc_tmp_data[UID1_OFFSET];
	tmp[2] = *(__IO u8*)&crc_tmp_data[UID2_OFFSET];
	tmp[3] = *(__IO u8*)&crc_tmp_data[UID3_OFFSET_3];
	mcuID_add = (tmp[0]<<24) + (tmp[1]<<16) + (tmp[2]<<8) + tmp[3];
	pUID_dat32[2] = *(__IO u32*)(mcuID_add);
}
/*
函数名称: check_ID
功 能: 校验 ID
输 入: 无
返 回: HAL_OK,校验成功;HAL_ERROR,失败
*/
HAL_StatusTypeDef check_ID(void)
{
	// Step1.读取本机 ID,存入 mcuID 联合体
	read_ID((u32*)mcuID.dat32);
	// Step2.使用本机 ID 计算本机密码,存入 eCODE 数组
	calculate_eCODE((u8*)mcuID.dat8, (u8*)eCODE);
	// Step3.对比 FLASH 中存储的密码是否等于通用密码
	if((crc_tmp_data[eID0_OFFSET] == *(__IO u8*)&encrypt_code[0])
	&& (crc_tmp_data[eID1_OFFSET] == *(__IO u8*)&encrypt_code[1])
	&& (crc_tmp_data[eID2_OFFSET] == *(__IO u8*)&encrypt_code[2])
	&& (crc_tmp_data[eID3_OFFSET] == *(__IO u8*)&encrypt_code[3])
	&& (crc_tmp_data[eID4_OFFSET] == *(__IO u8*)&encrypt_code[4])
	&& (crc_tmp_data[eID5_OFFSET] == *(__IO u8*)&encrypt_code[5])
	&& (crc_tmp_data[eID6_OFFSET] == *(__IO u8*)&encrypt_code[6])
	&& (crc_tmp_data[eID7_OFFSET] == *(__IO u8*)&encrypt_code[7])
	&& (crc_tmp_data[eID8_OFFSET] == *(__IO u8*)&encrypt_code[8])
	&& (crc_tmp_data[eID9_OFFSET] == *(__IO u8*)&encrypt_code[9])
	&& (crc_tmp_data[eID10_OFFSET]== *(__IO u8*)&encrypt_code[10])
	&& (crc_tmp_data[eID11_OFFSET]== *(__IO u8*)&encrypt_code[11])
	) {// 如果密码等于通用密码,则将计算的结果写入 FLASH
	encrypt_code_save();
	return HAL_BUSY;
	} else
	// Step4.对比计算的本机密码是否等于 FLASH 中存储的密码
	if((eCODE[0] == *(__IO u8*)&encrypt_code[0])
	&& (eCODE[1] == *(__IO u8*)&encrypt_code[1])
	&& (eCODE[2] == *(__IO u8*)&encrypt_code[2])
	&& (eCODE[3] == *(__IO u8*)&encrypt_code[3])
	&& (eCODE[4] == *(__IO u8*)&encrypt_code[4])
	&& (eCODE[5] == *(__IO u8*)&encrypt_code[5])
	&& (eCODE[6] == *(__IO u8*)&encrypt_code[6])
	&& (eCODE[7] == *(__IO u8*)&encrypt_code[7])
	&& (eCODE[8] == *(__IO u8*)&encrypt_code[8])
	&& (eCODE[9] == *(__IO u8*)&encrypt_code[9])
	&& (eCODE[10]== *(__IO u8*)&encrypt_code[10])
	&& (eCODE[11]== *(__IO u8*)&encrypt_code[11])
	) {
	// 如果密码校验正确,则不进行任何操作
	return HAL_OK;
	} else {
	// 如果密码校验错位,则关机或者销毁程序
	return HAL_ERROR;
	}
}

