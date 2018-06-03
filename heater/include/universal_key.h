#ifndef   __UNIVERSAL_KEY_H__
#define   __UNIVERSAL_KEY_H__



#define MAX_RCV_BUF_SIZE         255

#define MODE_KEY1_DOWN         0x01
#define MODE_KEY1_LONG         0x04
#define MODE_KEY1_UP           0x10

#define STEP_KEY2_DOWN         0x02
#define STEP_KEY2_LONG         0x08
#define STEP_KEY2_UP           0x20
#define UNIVERSAL_NOKEY        0xff
 

//功能说明: 检测所有按键。10MS 调用一次  
extern void Pannelkey_Polling(void);    
 
extern u8 read_universal_key(void);
extern void clear_key_buf(void);
extern void  PanakeyVar_Init(void)  ;








#endif
