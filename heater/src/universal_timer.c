
#include "stm32f10x.h"
#include "universal_timer.h"

#define MAX_NUM_TIMERS   3


/******************************************************************************
* �ļ����ƣ�SoftTimer.c
* ����ժҪ�������ʱ��ģ��
* ����˵������������TimersInit����������ú����ṩ"1ms��ϵͳʱ��"��"���ϵͳms��"��
*         Ȼ���ڸ��Ե�Ӧ��ģ���е���CreatTimer������ʱ�����ú������صĵ�ַΪ�ö�
*         ʱ���ĵ�ַ��������������ɾ����ʱ����㣬�����Ʊ��ܡ�������ѭ����ִ��
*         ProcessTimer�����Ը��¶�ʱ��ʱ�䡣
* ��ǰ�汾��V1.00
* �� �ߣ�  David Han, Ian
* ������ڣ�2015��2��20��
******************************************************************************/

static TIMER_TABLE* sg_ptTimeTableHead = NULL;             /* �����ͷ       */
static TMRSOURCE    sg_pfSysClk        = NULL;             /* ϵͳ1msʱ�Ӻ��� */
static long int     sg_dwTimeMaxValue  = MAX_VALUE_16_BIT; /* ���ms��       */


static  TIMER_TABLE timerList[MAX_NUM_TIMERS];

static TIMER_TABLE  timerHead;

/*************************************************************************
* �������ƣ�int timer_init(TMRSOURCE pfTimer, u32 dwMaxTime)
* ����˵������ʼ�������ʱ��ģ��
* ���������TMRSOURCE pfTimer  ϵͳ1msʱ�Ӻ���
           u32   dwMaxTime ʱ�Ӻ������ms��
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int timer_init(TMRSOURCE pfTimer, u32 dwMaxTime)
{
    if (NULL == pfTimer)
    {
        return SW_ERROR; /* ���ע�ắ���Ƿ�Ϊ��ָ�� */
    }
    
    sg_ptTimeTableHead = &timerHead; /* ����ͷ��� */
    if (NULL == sg_ptTimeTableHead)
    { 
        return SW_ERROR; /* ����Ƿ�����ɹ� */
    }

    /* ����ɹ�����г�ʼ�� */
    sg_ptTimeTableHead->next = NULL;               /* �¸�����ַ�ÿ�     */
    sg_pfSysClk              = (TMRSOURCE)pfTimer; /* ע��ϵͳ1msʱ�Ӻ���  */
    sg_dwTimeMaxValue        = dwMaxTime;          /* ȷ��ʱ�Ӻ������ms�� */
    
    return SW_OK;
}

/*************************************************************************
* �������ƣ�TIMER_TABLE* creat_timer(u32 dwTimeout, u8 ucPeriodic, TMRCALLBACK pfTimerCallback, void *pArg)
* ����˵�������������������ʱ��
* ���������u32       dwTimeout  0~0xFFFFFFFF ��ʱʱ��
            u8       ucPeriodic  SINGLE      ���δ���
                                   PERIODIC    ���ڴ���
           TMRCALLBACK pfTimerCallback         ��ʱ����ʱ�ص�����
           void       *pArg                    �ص���������
            
* �����������
* �� �� ֵ������ʧ�� : NULL
           �����ɹ� : ��ʱ��ģ��ָ��
* ����˵���������궨ʱ���󷵻ض�ʱ�����ĵ�ַ���ĵ�ַ����������ɾ���ö�ʱ��
**************************************************************************/
TIMER_TABLE* creat_timer(u32 dwTimeout, u8 ucPeriodic, TMRCALLBACK pfTimerCallback, void *pArg)
{
    TIMER_TABLE* ptTimerNode;
    TIMER_TABLE* ptFind;
	static u8 timerIndex = 0;
    if (NULL == sg_ptTimeTableHead)
    {
        return NULL; /* �������ͷ�ڵ��Ƿ���� */
    }

    /* ����ͷ����Ѿ����� */
  
    if (timerIndex >= MAX_NUM_TIMERS)
    {
        return NULL; /* ����Ƿ�����ɹ� */
    }
	ptTimerNode = &timerList[timerIndex]; /* ���붨ʱ����� */
	timerIndex++;

    /* �������ɹ� */
    ptTimerNode->next                 = NULL;                    /* �¸�����ַ�ÿ� */
	ptTimerNode->data.timeStat        = TIMER_STOP;
    ptTimerNode->data.periodic        = ucPeriodic;              /* ����/���ڴ��� */
    ptTimerNode->data.start           = sg_pfSysClk();           /* ��ȡ��ʱ��ʼʱ�� */
    ptTimerNode->data.now             = ptTimerNode->data.start; /* ��ȡ��ǰʱ�� */
    ptTimerNode->data.elapse          = 0;                       /* �Ѿ�����ʱ�� */
    ptTimerNode->data.timeout         = dwTimeout;               /* ��ʱʱ�� */
    ptTimerNode->data.pfTimerCallback = pfTimerCallback;         /* ע�ᶨʱ�����ص����� */
    ptTimerNode->data.pArg            = pArg;                    /* �ص��������� */

    /* ��������Ķ�ʱ��������ӽ������� */
    ptFind = sg_ptTimeTableHead; /* ��������ͷ��� */
    while(NULL != ptFind->next)  /* �����ǰ��㲻��ĩβ���*/
    {
        ptFind = ptFind->next;   /* ����һ�����ĵ�ַ��Ϊ��ǰ���������� */
    }
    /* �ҵ�ĩβ��� */
    ptFind->next= ptTimerNode;   /* �������������ӵ�ĩβ��� */

    return ptTimerNode;          /* �����ɹ����������������ַ(����ɾ����������ʱ) */
}

/*************************************************************************
* �������ƣ�int stop_timer(TIMER_TABLE* ptNode)
* ����˵����ֹͣ��ʱ�����
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int stop_timer(TIMER_TABLE* ptNode)
{

    if (ptNode)
    {
		ptNode->data.timeStat = TIMER_STOP;
    }
	else
    {
        return SW_ERROR; /* ��鶨ʱ������Ƿ�Ϊ�� */
    }
    return SW_OK;                    
}

/*************************************************************************
* �������ƣ�int start_timer(TIMER_TABLE* ptNode)
* ����˵����ֹͣ��ʱ�����
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int start_timer(TIMER_TABLE* ptNode)
{

    if (ptNode)
    {
		ptNode->data.start           = sg_pfSysClk();           /* ��ȡ��ʱ��ʼʱ�� */
    	ptNode->data.now             = ptNode->data.start; /* ��ȡ��ǰʱ�� */
		ptNode->data.timeStat = TIMER_RUNNING;
    }
	else
    {
        return SW_ERROR; /* ��鶨ʱ������Ƿ�Ϊ�� */
    }
    return SW_OK;                    
}

/*************************************************************************
* �������ƣ�int set_timer_time(TIMER_TABLE* ptNode)
* ����˵�������ö�ʱ����ʱ��
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int set_timer_time(TIMER_TABLE* ptNode, u32 ticks)
{

    if (ptNode)
    {
		ptNode->data.timeout  = ticks;         
    }
	else
    {
        return SW_ERROR; /* ��鶨ʱ������Ƿ�Ϊ�� */
    }
    return SW_OK;                    
}


/*************************************************************************
* �������ƣ�int reset_timer(TIMER_TABLE* ptNode)
* ����˵����������ʱ�����
* ���������TIMER_TABLE* ptNode ��ʱ������ַ
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
           SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int reset_timer(TIMER_TABLE* ptNode)
{
    if (NULL == ptNode)
    {
        return SW_ERROR;                /* ��鶨ʱ������Ƿ�Ϊ�� */
    }

    ptNode->data.start = sg_pfSysClk(); /* ���¶�ʱ����ʼʱ�� */
    return SW_OK;                       /* �����ɹ� */
}


/*************************************************************************
* �������ƣ�int process_timer(void)
* ����˵�������¶�ʱ�����
* �����������
* �����������
* �� �� ֵ��SW_ERROR: ����ʧ��
            SW_OK �����ɹ�
* ����˵������
**************************************************************************/
int process_timer(void)
{
    TIMER_TABLE* ptFind;
	
    if (NULL == sg_ptTimeTableHead)
    { 
        return SW_ERROR; /* ����Ƿ�����ɹ� */
    }
    ptFind = sg_ptTimeTableHead->next;    /* �ҵ���һ����Ч��� */
    while(ptFind)                         /* �������ĩβ��� */
    { 
		if(TIMER_RUNNING == ptFind->data.timeStat)
		{
        	ptFind->data.now = sg_pfSysClk(); /* ����ʱ�� */

	        /* ����˿�ʱ������ʼʱ���ʱ��� */
	        if(ptFind->data.now >= ptFind->data.start)
	        {
	            ptFind->data.elapse = ptFind->data.now - ptFind->data.start;
	        }
	        else
	        {
	            ptFind->data.elapse = sg_dwTimeMaxValue - ptFind->data.start + ptFind->data.now;
	        }
	        
	        if(ptFind->data.elapse >= ptFind->data.timeout)          /* ���ʱ����ڵ����趨�ļ�ʱʱ�� */
	        {
	            
	            if(ptFind->data.periodic)
	            {
	                reset_timer(ptFind);                              /* ����������Դ�����������ʱ�� */
	            }
	            else
	            {                                                    /* ����ǵ��δ�����ɾ����ʱ�� */ 
	                stop_timer(ptFind); 
	            }  
				if(ptFind->data.pfTimerCallback)                     /* ���Ѿ�ע���˺Ϸ��Ļص����� */
	            {
	                ptFind->data.pfTimerCallback(ptFind->data.pArg); /* ִ�лص����� */
	            }    
	        }
		}
       	ptFind = ptFind->next;                                   /* ����������һ����ʱ����� */
		
    }
    return SW_OK;                                                /* �����ɹ� */
}

/* end of file */





