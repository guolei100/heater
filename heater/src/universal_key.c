
#include "stm32f10x.h"
#include "universal_key.h"
#include <string.h>


/* �����˲�ʱ��50ms, ��λ10ms 
 *ֻ��������⵽50ms״̬�������Ϊ��Ч����������Ͱ��������¼� 
 */  
#define BUTTON_FILTER_TIME        2  
#define BUTTON_LONG_TIME         150                /* ����1�룬��Ϊ�����¼� */  


    
#define UNIVERSAL_NOKEY  0xff



/* 
        ÿ��������Ӧ1��ȫ�ֵĽṹ������� 
        ���Ա������ʵ���˲��Ͷ��ְ���״̬������� 
*/  
typedef struct  
{  
        /* ������һ������ָ�룬ָ���жϰ����ַ��µĺ��� */  
        unsigned char  (*IsKeyDownFunc)(void); /* �������µ��жϺ���,1��ʾ���� */  
   
        unsigned char  Count;                        /* �˲��������� */  
        unsigned char  FilterTime;                /* �˲�ʱ��(���255,��ʾ2550ms) */  
        unsigned short LongCount;                /* ���������� */  
        unsigned short LongTime;                /* �������³���ʱ��, 0��ʾ����ⳤ�� */  
        unsigned char   State;                        /* ������ǰ״̬�����»��ǵ��� */  
        unsigned char  KeyCodeUp;                /* ��������ļ�ֵ����, 0��ʾ����ⰴ������ */  
        unsigned char  KeyCodeDown;        /* �������µļ�ֵ����, 0��ʾ����ⰴ������ */  
        unsigned char  KeyCodeLong;        /* ���������ļ�ֵ����, 0��ʾ����ⳤ�� */  
        unsigned char  RepeatSpeed;        /* ������������ */  
        unsigned char  RepeatCount;        /* �������������� */  
}BUTTON_T;  
   
typedef enum  
{  
        KEY_NONE = 0,                        /* 0 ��ʾ�����¼� */  
   
        KEY_DOWN_Power,                        /* ���������� */  
        KEY_UP_Power,                        /* ���������� */  
        KEY_LONG_Power,                        /* ���������� */  
          
        KEY_DOWN_Power_TAMPER        /* ��ϼ���Power����WAKEUP��ͬʱ���� */  
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


//�Ƿ��а������½ӿں���  
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
        /* ��ʼ��USER����������֧�ְ��¡����𡢳��� */  
        modeKey1.IsKeyDownFunc = IsKey1DownUser;                /* �жϰ������µĺ��� */  
        modeKey1.FilterTime = BUTTON_FILTER_TIME;                /* �����˲�ʱ�� */  
        modeKey1.LongTime = BUTTON_LONG_TIME;                    /* ����ʱ�� */  
        modeKey1.Count = modeKey1.FilterTime / 2;               /* ����������Ϊ�˲�ʱ���һ�� */  
        modeKey1.State = 0;                                      /* ����ȱʡ״̬��0Ϊδ���� */  
        modeKey1.KeyCodeDown = MODE_KEY1_DOWN;                   /* �������µļ�ֵ���� */  
        modeKey1.KeyCodeUp =MODE_KEY1_UP;                        /* ��������ļ�ֵ���� */  
        modeKey1.KeyCodeLong = MODE_KEY1_LONG;                   /* �������������µļ�ֵ���� */  
        modeKey1.RepeatSpeed = 0;                                /* �����������ٶȣ�0��ʾ��֧������ */  
        modeKey1.RepeatCount = 0;                                /* ���������� */     

		stepKey2.IsKeyDownFunc = IsKey2DownUser;                /* �жϰ������µĺ��� */  
        stepKey2.FilterTime = BUTTON_FILTER_TIME;                /* �����˲�ʱ�� */  
        stepKey2.LongTime = BUTTON_LONG_TIME;                    /* ����ʱ�� */  
        stepKey2.Count = modeKey1.FilterTime / 2;               /* ����������Ϊ�˲�ʱ���һ�� */  
        stepKey2.State = 0;                                      /* ����ȱʡ״̬��0Ϊδ���� */  
        stepKey2.KeyCodeDown = STEP_KEY2_DOWN;                   /* �������µļ�ֵ���� */  
        stepKey2.KeyCodeUp =STEP_KEY2_UP;                        /* ��������ļ�ֵ���� */  
        stepKey2.KeyCodeLong = STEP_KEY2_LONG;                   /* �������������µļ�ֵ���� */  
        stepKey2.RepeatSpeed = 0;                                /* �����������ٶȣ�0��ʾ��֧������ */  
        stepKey2.RepeatCount = 0;                                /* ���������� */       
}  
void Panakey_Init(void)  
{  
        //PanakeyHard_Init();                /* ��ʼ���������� */  
        PanakeyVar_Init();                /* ��ʼ������Ӳ�� */  
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
  // ����һ������ ���밴��ֵ          
} 


/* 
********************************************************************************************************* 
*        �� �� ��: bsp_DetectButton 
*        ����˵��: ���һ��������������״̬�����뱻�����Եĵ��á� 
*        ��    �Σ������ṹ����ָ�� 
*        �� �� ֵ: �� 
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

				/* ���Ͱ�ť���µ���Ϣ */  
				if (_pBtn->KeyCodeDown > 0)  
				{  
					/* ��ֵ���밴��FIFO */  
					Pannelkey_Put(_pBtn->KeyCodeDown);// ��¼�������±�־���ȴ��ͷ�  
				}  
			}  
			if (_pBtn->LongTime > 0)  
			{  
				if (_pBtn->LongCount < _pBtn->LongTime)  
				{  
					/* ���Ͱ�ť�������µ���Ϣ */  
					if (++_pBtn->LongCount == _pBtn->LongTime)  
					{  
						/* ��ֵ���밴��FIFO */  
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
						    /* ��������ÿ��10ms����1������ */  
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

				/* ���Ͱ�ť�������Ϣ */  
				if (_pBtn->KeyCodeUp > 0) /*�����ͷ�*/  
				{  
					/* ��ֵ���밴��FIFO */  
					Pannelkey_Put(_pBtn->KeyCodeUp);          
				}  
			}  
		}  
		_pBtn->LongCount = 0;  
		_pBtn->RepeatCount = 0;  
	}  
}  
//����˵��: ������а�����10MS ����һ��  
void Pannelkey_Polling(void)  
{  
        Button_Detect(&modeKey1);                /* mode �� */  
		Button_Detect(&stepKey2);
}  
 


u8 read_universal_key(void)
{
	u8 keyValue;
	
	read_byte_from_q_buff(&keyValue);
	return keyValue;
}

































