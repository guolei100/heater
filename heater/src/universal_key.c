
#include "stm32f10x.h"
#include "universal_key.h"
#include <string.h>


/* 按键滤波时间50ms, 单位10ms 
 *只有连续检测到50ms状态不变才认为有效，包括弹起和按下两种事件 
 */  
#define BUTTON_FILTER_TIME        2  
#define BUTTON_LONG_TIME         150                /* 持续1秒，认为长按事件 */  


    
#define UNIVERSAL_NOKEY  0xff



/* 
        每个按键对应1个全局的结构体变量。 
        其成员变量是实现滤波和多种按键状态所必须的 
*/  
typedef struct  
{  
        /* 下面是一个函数指针，指向判断按键手否按下的函数 */  
        unsigned char  (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */  
   
        unsigned char  Count;                        /* 滤波器计数器 */  
        unsigned char  FilterTime;                /* 滤波时间(最大255,表示2550ms) */  
        unsigned short LongCount;                /* 长按计数器 */  
        unsigned short LongTime;                /* 按键按下持续时间, 0表示不检测长按 */  
        unsigned char   State;                        /* 按键当前状态（按下还是弹起） */  
        unsigned char  KeyCodeUp;                /* 按键弹起的键值代码, 0表示不检测按键弹起 */  
        unsigned char  KeyCodeDown;        /* 按键按下的键值代码, 0表示不检测按键按下 */  
        unsigned char  KeyCodeLong;        /* 按键长按的键值代码, 0表示不检测长按 */  
        unsigned char  RepeatSpeed;        /* 连续按键周期 */  
        unsigned char  RepeatCount;        /* 连续按键计数器 */  
}BUTTON_T;  
   
typedef enum  
{  
        KEY_NONE = 0,                        /* 0 表示按键事件 */  
   
        KEY_DOWN_Power,                        /* 按键键按下 */  
        KEY_UP_Power,                        /* 按键键弹起 */  
        KEY_LONG_Power,                        /* 按键键长按 */  
          
        KEY_DOWN_Power_TAMPER        /* 组合键，Power键和WAKEUP键同时按下 */  
}KEY_ENUM;  
   
BUTTON_T modeKey1;  
BUTTON_T stepKey2; 


typedef struct
{
	int  WritePoint;				//write pointer
	int  ReadPoint; 				//read pointer
	int  FullFlag; 				//full flag
	int  TotalCnt;               //reiverde total byte from uart
	int  LostCnt;                //lost byts from uart
	unsigned char DataBuf[255];	//cycle buffer
}Q_BUF;



Q_BUF keyBuff;


//是否有按键按下接口函数  
unsigned char  IsKey1DownUser(void)                   
{
		if (0==GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) )
			return 1;
		return 0;
}  


  unsigned char  IsKey2DownUser(void)                   
{
		if (0 == GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) )
			return 1;
		return 0;
}  


void clear_key_buf(void)
{
	keyBuff.WritePoint = 0;
	keyBuff.ReadPoint = 0;
	keyBuff.FullFlag = 0;
	keyBuff.TotalCnt = 0;
	keyBuff.LostCnt = 0;
	memset(keyBuff.DataBuf,0,sizeof(keyBuff.DataBuf));
}
   
 
void  PanakeyVar_Init(void)  
{  
        /* 初始化USER按键变量，支持按下、弹起、长按 */  
        modeKey1.IsKeyDownFunc = IsKey1DownUser;                /* 判断按键按下的函数 */  
        modeKey1.FilterTime = BUTTON_FILTER_TIME;                /* 按键滤波时间 */  
        modeKey1.LongTime = BUTTON_LONG_TIME;                    /* 长按时间 */  
        modeKey1.Count = modeKey1.FilterTime / 2;               /* 计数器设置为滤波时间的一半 */  
        modeKey1.State = 0;                                      /* 按键缺省状态，0为未按下 */  
        modeKey1.KeyCodeDown = MODE_KEY1_DOWN;                   /* 按键按下的键值代码 */  
        modeKey1.KeyCodeUp =MODE_KEY1_UP;                        /* 按键弹起的键值代码 */  
        modeKey1.KeyCodeLong = MODE_KEY1_LONG;                   /* 按键被持续按下的键值代码 */  
        modeKey1.RepeatSpeed = 0;                                /* 按键连发的速度，0表示不支持连发 */  
        modeKey1.RepeatCount = 0;                                /* 连发计数器 */     

		stepKey2.IsKeyDownFunc = IsKey2DownUser;                /* 判断按键按下的函数 */  
        stepKey2.FilterTime = BUTTON_FILTER_TIME;                /* 按键滤波时间 */  
        stepKey2.LongTime = BUTTON_LONG_TIME;                    /* 长按时间 */  
        stepKey2.Count = modeKey1.FilterTime / 2;               /* 计数器设置为滤波时间的一半 */  
        stepKey2.State = 0;                                      /* 按键缺省状态，0为未按下 */  
        stepKey2.KeyCodeDown = STEP_KEY2_DOWN;                   /* 按键按下的键值代码 */  
        stepKey2.KeyCodeUp =STEP_KEY2_UP;                        /* 按键弹起的键值代码 */  
        stepKey2.KeyCodeLong = STEP_KEY2_LONG;                   /* 按键被持续按下的键值代码 */  
        stepKey2.RepeatSpeed = 0;                                /* 按键连发的速度，0表示不支持连发 */  
        stepKey2.RepeatCount = 0;                                /* 连发计数器 */       
}  
void Panakey_Init(void)  
{  
        //PanakeyHard_Init();                /* 初始化按键变量 */  
        PanakeyVar_Init();                /* 初始化按键硬件 */  
		clear_key_buf();
}  



static int write_byte_to_q_buff(u8 wByte)
{
 int status = 0;
 
 
 if ((0 == keyBuff.FullFlag))
 {
	 keyBuff.DataBuf[keyBuff.WritePoint] = wByte;
	 if(++keyBuff.WritePoint >= MAX_RCV_BUF_SIZE)
	 {
		 keyBuff.WritePoint = 0;
	 }

	 if(keyBuff.WritePoint == keyBuff.ReadPoint)
	 {
		 keyBuff.FullFlag = 1;
	 }
 }
 else
 {
	 status = -1;
 }
 return status;  
}
 
 
 /********************************************************************************
 *fun name: 	   read_byte_from_uart_buff 
 *fun description:	 read byte from UartNum uart  buff			
 *input param:		   UartNum :	
 *										 HAL_UART_1 	
 *										 HAL_UART_2
 *				 rByte	which read from uart buff				
 *output param: 	
 *return value: 		  0 		success
 *				   -1		fail   
 *remark:			   creat 2017-03-15  guolei
 ********************************************************************************/
 static int read_byte_from_q_buff(u8 *rByte)
 {
	 int status = 0;
	  
	 
	 if((0 != keyBuff.FullFlag)||(keyBuff.WritePoint != keyBuff.ReadPoint))
	 {
		 *rByte = keyBuff.DataBuf[keyBuff.ReadPoint];
		 if(++keyBuff.ReadPoint >= MAX_RCV_BUF_SIZE)
		 {
			 keyBuff.ReadPoint =0;
		 }
		 keyBuff.FullFlag = 0;
	 }
	 else 
	 {
	 	*rByte = UNIVERSAL_NOKEY;
		 status = -1;
	 }
	 return status;
 }






void Pannelkey_Put(unsigned char keyCode)  
{  
	write_byte_to_q_buff(keyCode);       
  // 定义一个队列 放入按键值          
} 


/* 
********************************************************************************************************* 
*        函 数 名: bsp_DetectButton 
*        功能说明: 检测一个按键。非阻塞状态，必须被周期性的调用。 
*        形    参：按键结构变量指针 
*        返 回 值: 无 
********************************************************************************************************* 
*/  
 void Button_Detect(BUTTON_T *_pBtn)  
{  
	if (_pBtn->IsKeyDownFunc())  
	{  
		//printf("mode key down\r\n");
		if (_pBtn->Count < _pBtn->FilterTime)  
		{  
			_pBtn->Count = _pBtn->FilterTime;  
		}  
		else if(_pBtn->Count < 2 * _pBtn->FilterTime)  
		{  
			_pBtn->Count++;  
		}  
		else  
		{  
			if (_pBtn->State == 0)  
			{  
				_pBtn->State = 1;  

				/* 发送按钮按下的消息 */  
				if (_pBtn->KeyCodeDown > 0)  
				{  
					/* 键值放入按键FIFO */  
					Pannelkey_Put(_pBtn->KeyCodeDown);// 记录按键按下标志，等待释放  
				}  
			}  
			if (_pBtn->LongTime > 0)  
			{  
				if (_pBtn->LongCount < _pBtn->LongTime)  
				{  
					/* 发送按钮持续按下的消息 */  
					if (++_pBtn->LongCount == _pBtn->LongTime)  
					{  
						/* 键值放入按键FIFO */  
						Pannelkey_Put(_pBtn->KeyCodeLong);          
					}  
				}  
				else  
				{  
					if (_pBtn->RepeatSpeed > 0)  
					{  
						if (++_pBtn->RepeatCount >= _pBtn->RepeatSpeed)  
						{  
						    _pBtn->RepeatCount = 0;  
						    /* 常按键后，每隔10ms发送1个按键 */  
						    Pannelkey_Put(_pBtn->KeyCodeDown);          
						}  
					}  
				}  
			}  
		}  
	}  
    else  
	{  
		if(_pBtn->Count > _pBtn->FilterTime)  
		{  
			_pBtn->Count = _pBtn->FilterTime;  
		}  
		else if(_pBtn->Count != 0)  
		{  
			_pBtn->Count--;  
		}  
		else  
		{  
			if (_pBtn->State == 1)  
			{  
				_pBtn->State = 0;  

				/* 发送按钮弹起的消息 */  
				if (_pBtn->KeyCodeUp > 0) /*按键释放*/  
				{  
					/* 键值放入按键FIFO */  
					Pannelkey_Put(_pBtn->KeyCodeUp);          
				}  
			}  
		}  
		_pBtn->LongCount = 0;  
		_pBtn->RepeatCount = 0;  
	}  
}  
//功能说明: 检测所有按键。10MS 调用一次  
void Pannelkey_Polling(void)  
{  
        Button_Detect(&modeKey1);                /* mode 键 */  
		Button_Detect(&stepKey2);
}  
 


u8 read_universal_key(void)
{
	u8 keyValue;
	
	read_byte_from_q_buff(&keyValue);
	return keyValue;
}

































