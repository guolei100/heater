#ifndef   __UNIVERSAL_TIMER_H__
#define   __UNIVERSAL_TIMER_H__

#include "stddef.h"
#include "stdint.h"



/******************************************************************************
* �ļ����ƣ�SoftTimer.h
* ����ժҪ�������ʱ��ģ��ͷ�ļ�
* ����˵������������TimersInit����������ú����ṩ"1ms��ϵͳʱ��"��"���ϵͳms��"��
*          Ȼ���ڸ��Ե�Ӧ��ģ���е���CreatTimer������ʱ�����ú������صĵ�ַΪ�ö�
*          ʱ���ĵ�ַ��������������ɾ����ʱ����㣬�����Ʊ��ܡ�������ѭ����ִ��
*          ProcessTimer�����Ը��¶�ʱ��ʱ�䡣
* ��ǰ�汾��V1.00
* �� �ߣ�David Han, Ian
* ������ڣ�2015��2��20��
******************************************************************************/
  

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_VALUE_8_BIT  0xFF       /* 16bit���ms�� */
#define MAX_VALUE_16_BIT 0xFFFF     /* 16bit���ms�� */
#define MAX_VALUE_24_BIT 0xFFFFFF   /* 16bit���ms�� */
#define MAX_VALUE_32_BIT 0xFFFFFFFF /* 16bit���ms�� */

#define SW_ERROR               (-1) /* ����ʧ�ܷ���ֵ */
#define SW_OK                   (0) /* �����ɹ�����ֵ */

typedef int (*TMRCALLBACK)(void *pArg);
typedef u32 (*TMRSOURCE)(void);

typedef enum{
	TIMER_STOP   = 0,
	TIMER_RUNNING
}TIMER_STAT;
/****************************************************
* �ṹ����TIMER
* ������ �����ʱ�����ݽṹ
* ������
     u8        periodic;        SINGLE       ���δ���
                                   PERIOIC      ���ڴ���
     u32       start;           0~0xFFFFFFFF ��ʱ����ʼʱ��
     u32       now;             0~0xFFFFFFFF ��ʱ����ǰʱ��
     u32       elapse;          0~0xFFFFFFFF ��ʱ���ѹ�ʱ��
     u32       timeout;         0~0xFFFFFFFF ��ʱ����ʱʱ��
     TMRCALLBACK  pfTimerCallback;              ��ʱ������ִ�еĻص�����
     void        *pArg;                         �ص������Ĳ���
* ���ߣ� David Han, Ian
* ����: 2015-2-20
****************************************************/
typedef struct _TIMER
{
    u8      	 periodic;         /* ���δ���/���ڴ��� */
	TIMER_STAT   timeStat;
    u32      	 start;           /* ��ʱ����ʼʱ�� */
    u32     	 now;             /* ��ʱ����ǰʱ�� */
    u32     	 elapse;          /* ��ʱ���ѹ�ʱ�� */
    u32      	 timeout;         /* ��ʱ����ʱʱ�� */
    TMRCALLBACK  pfTimerCallback; /* ��ʱ������ִ�еĻص����� */
    void         *pArg;            /* �ص������Ĳ��� */
} TIMER;



/****************************************************
* �ṹ����TIMER_TABLE
* ������ ��ʱ���б�(����)
* ������
          TIMER                data; ����ʱ��������� 
          struct _TIMER_TABLE* next; ��һ����ʱ������ַ 
* ���ߣ� David Han, Ian
* ����: 2015-2-20
****************************************************/
typedef struct _TIMER_TABLE 
{ 
    TIMER                data; /* ����ʱ��������� */
    struct _TIMER_TABLE* next; /* ��һ����ʱ������ַ */
}TIMER_TABLE; 

/*************************************************************************
* �������ƣ�int timer_init(TMRSOURCE pfTimer, u32 dwMaxTime)
* ����˵������ʼ�������ʱ��ģ��
* ���������TMRSOURCE pfTimer    ϵͳ1msʱ�Ӻ���
           u32    dwMaxTime  ʱ�Ӻ������ms��
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int timer_init(TMRSOURCE pfTimer, u32 dwMaxTime);


/*************************************************************************
* �������ƣ�TIMER_TABLE* creat_timer(u32 dwTimeout, u8 ucPeriodic, TMRCALLBACK pfTimerCallback, void *pArg)
* ����˵�������������������ʱ��
* ���������u32         dwTimeout  0~0xFFFFFFFF ��ʱʱ��
           u8          ucPeriodic SINGLE      ���δ���
                                     PERIODIC    ���ڴ���
           TMRCALLBACK    pfTimerCallback        ��ʱ����ʱ�ص�����
            void         *pArg                   �ص���������
            
* �����������
* �� �� ֵ������ʧ�� : NULL
           �����ɹ� : ��ʱ��ģ��ָ��
* ����˵���������궨ʱ���󷵻ض�ʱ�����ĵ�ַ���ĵ�ַ����������ɾ���ö�ʱ��
**************************************************************************/
TIMER_TABLE* creat_timer(u32 dwTimeout, u8 ucPeriodic, TMRCALLBACK pfTimerCallback, void *pArg);


/*************************************************************************
* �������ƣ�int stop_timer(TIMER_TABLE* ptNode)
* ����˵����ֹͣ��ʱ�����
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
extern int stop_timer(TIMER_TABLE* ptNode);
/*************************************************************************
* �������ƣ�int start_timer(TIMER_TABLE* ptNode)
* ����˵����ֹͣ��ʱ�����
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
extern int start_timer(TIMER_TABLE* ptNode);

/*************************************************************************
* �������ƣ�int set_timer_time(TIMER_TABLE* ptNode)
* ����˵�������ö�ʱ����ʱ��
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
extern int set_timer_time(TIMER_TABLE* ptNode, u32 ticks);

/*************************************************************************
* �������ƣ�int reset_timer(TIMER_TABLE* ptNode)
* ����˵����������ʱ�����
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int reset_timer(TIMER_TABLE* ptNode);


/*************************************************************************
* �������ƣ�int process_timer(void)
* ����˵�������¶�ʱ�����
* �����������
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int process_timer(void);


#ifdef __cplusplus
}
#endif



/* end of file */









#endif
