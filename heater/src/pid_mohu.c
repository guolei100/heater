/********************************************************************
ģ�� PID �����¿�ϵͳ������� C �������
********************************************************************/
#include<reg52.h>
#define uchar unsigned char
#define uint unsigned int
#define PULSE 200
#define number 0.035
sbit SDO = P2^0;
sbit SDI = P2^1;
sbit CS = P2^2;
sbit CLK = P2^3;
sbit EOC = P2^4;
sbit RS = P2^5;
sbit RW = P2^6;
sbit EN = P2^7;
sbit KEY1= P3^0;
sbit KEY2= P3^1;
sbit KEY3= P3^2;
sbit KEY4= P3^3;
sbit KEY5= P3^4;
sbit IN1 = P3^5;
sbit IN2 = P3^6;
sbit ENA = P3^7;
uchar flag;
uchar flag_start;
float S_temp=60.0;
float P_temp=20.0;
float Kp;
float Ki;
float Kd;
float Err=0.0;
float Last_Err=0.0;
float D_Err=0.0;
float Sum_Err=0.0;
float U=0.0;
/******************************
�������ܣ���ʱ
******************************/
void delay_ms(uchar z)
{uchar i;
uchar j;
for(i=z;i>0;i--)
for(j=360;j>0;j--);
}
void delay_us(uchar z)
{
uchar i;
for(i=z;i>0;i--);
}











/************************************
�������ܣ� PID �ļ���
**********************************/
void PID_Calculate()
{
	Err = S_temp - P_temp;
	Sum_Err += Err;
	D_Err = Err - Last_Err;
	Last_Err = Err;
	U=Kp*Err+Ki*Sum_Err+Kd*D_Err;
	U=(int)U;
	if(U>=0)
	{
		if(U>=200)
			U=200;
		flag=1;
	}
	else
	{
		U=-U;
		if(U>=200)
			U=200;
		flag=0;
	}
}
/***********************************************
�������ܣ� PID ���� Kp �ļ���
************************************************/
float fuzzy_kp(float e, float ec) //e,ec����ʾ�����仯��
{
	float Kp_calcu;
	uchar num,pe,pec;
	float code eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0}; //��� E ��ģ������
	float code ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};//���仯�� EC ��ģ������
	float eFuzzy[2]={0.0,0.0}; //��������� E �������̶�
	float ecFuzzy[2]={0.0,0.0}; //���������仯�� EC �������̶�
	float code kpRule[4]={0.0,8.0,16.0,24.0}; //Kp ��ģ���Ӽ�
	float KpFuzzy[4]={0.0,0.0,0.0,0.0}; //������ Kp �������̶�
	int code KpRule[7][7]= //Kp ��ģ�����Ʊ�
	{
		3,3,3,3,3,3,3,
		2,2,2,2,1,2,2,
		1,1,1,1,1,1,1,
		1,1,0,1,0,1,1,
		0,0,1,0,0,1,0,
		0,1,0,1,0,0,2,
		3,3,3,3,3,3,3
	};
	/***** ��� E ������������ *****/
	if(e<eRule[0])
	{
		eFuzzy[0] =1.0;
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]);
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]);
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]);
		pe = 2;
	}
	else if(eRule[3]<=e && e<eRule[4])
	{ 
		eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]);
		pe = 3;
	}
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]);
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =5;
	}
	eFuzzy[1] =1.0 - eFuzzy[0];
	/***** ���仯�� EC ������������ *****/
	if(ec<ecRule[0])
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]);
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]);
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]);
		pec = 2 ;
	}else if(ecRule[3]<=ec && ec<ecRule[4])
	{ 
		ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
		pec=3;
	}
	else if(ecRule[4]<=ec && ec<ecRule[5])
	{ 
		ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
		pec=4;
	}
	else if(ecRule[5]<=ec && ec<ecRule[6])
	{
		ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
		pec=5;
	}
	else
	{
		ecFuzzy[0] =0.0;
		pec = 5;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0];
	/********* ��ѯģ������� *********/
	num =KpRule[pe][pec];
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KpRule[pe][pec+1];
	KpFuzzy[num] += eFuzzy[0]*ecFuzzy[1];
	num =KpRule[pe+1][pec];
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KpRule[pe+1][pec+1];
	KpFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	/********* �� Ȩ ƽ �� �� �� ģ �� *********/
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]
	+KpFuzzy[3]*kpRule[3];
	return(Kp_calcu);
}
/***********************************************
�������ܣ� PID ���� Ki �ļ���
************************************************/
float fuzzy_ki(float e, float ec)
{
	float Ki_calcu;
	uchar num,pe,pec;
	float code eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};
	float code ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};
	float eFuzzy[2]={0.0,0.0};
	float ecFuzzy[2]={0.0,0.0};float code kiRule[4]={0.00,0.01,0.02,0.03};
	float KiFuzzy[4]={0.0,0.0,0.0,0.0};
	int code KiRule[7][7]=
	{
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		2,0,0,0,0,0,1,
		3,3,3,3,3,3,3
	};
	/***** ��������������� *****/
	if(e<eRule[0])
	{
		eFuzzy[0] =1.0;
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]);
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]);
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]);
		pe = 2;
	}
	else if(eRule[3]<=e && e<eRule[4])
	{ 
		eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]);
		pe = 3;
	}
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]);
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =5;
	}
	eFuzzy[1] =1.0 - eFuzzy[0];
	/***** ���仯������������ *****/
	if(ec<ecRule[0])
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]);
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]);
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]);
		pec = 2 ;
	}
	else if(ecRule[3]<=ec && ec<ecRule[4])
	{ 
		ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
		pec=3;
	}
	else if(ecRule[4]<=ec && ec<ecRule[5])
	{ 
		ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
		pec=4;
	}
	else if(ecRule[5]<=ec && ec<ecRule[6])
	{ 
		ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
		pec=5;
	}
	else
	{
		ecFuzzy[0] =0.0;
		pec = 5;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0];
	/*********** ��ѯģ������� ***************/
	num =KiRule[pe][pec];
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KiRule[pe][pec+1];
	KiFuzzy[num] += eFuzzy[0]*ecFuzzy[1];
	num =KiRule[pe+1][pec];
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KiRule[pe+1][pec+1];
	KiFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
	/******** �� Ȩ ƽ �� �� �� ģ �� ********/
	Ki_calcu=KiFuzzy[0]*kiRule[0]+KiFuzzy[1]*kiRule[1]+KiFuzzy[2]*kiRule[2]
	+KiFuzzy[3]*kiRule[3];
	return(Ki_calcu);
	}
/***********************************************
�������ܣ� PID ���� Kd �ļ���
************************************************/
float fuzzy_kd(float e, float ec)
{
	float Kd_calcu;
	uchar num,pe,pec;
	float code eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};
	float code ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};
	float eFuzzy[2]={0.0,0.0};
	float ecFuzzy[2]={0.0,0.0};
	float code kdRule[4]={0.0,1.0,2.0,3.0};
	float KdFuzzy[4]={0.0,0.0,0.0,0.0};
	int code KdRule[7][7]=
	{
		3,3,3,2,2,2,2,
		2,2,2,1,1,1,1,
		1,1,2,1,1,2,1,
		1,1,0,1,0,1,1,
		1,1,0,0,0,1,1,
		2,2,1,0 ,1,1,1,
		3,3,3,3,2,3,2
	};
	/***** ��������������� *****/
	if(e<eRule[0])
	{
		eFuzzy[0] =1.0;
		pe = 0;
	}
	else if(eRule[0]<=e && e<eRule[1])
	{
		eFuzzy[0] = (eRule[1]-e)/(eRule[1]-eRule[0]);
		pe = 0;
	}
	else if(eRule[1]<=e && e<eRule[2])
	{
		eFuzzy[0] = (eRule[2] -e)/(eRule[2]-eRule[1]);
		pe = 1;
	}
	else if(eRule[2]<=e && e<eRule[3])
	{
		eFuzzy[0] = (eRule[3] -e)/(eRule[3]-eRule[2]);
		pe = 2;
	}
	else if(eRule[3]<=e && e<eRule[4])
	{
		eFuzzy[0] = (eRule[4]-e)/(eRule[4]-eRule[3]);
		pe = 3;
	}
	else if(eRule[4]<=e && e<eRule[5])
	{
		eFuzzy[0] = (eRule[5]-e)/(eRule[5]-eRule[4]);
		pe = 4;
	}
	else if(eRule[5]<=e && e<eRule[6])
	{
		eFuzzy[0] = (eRule[6]-e)/(eRule[6]-eRule[5]);
		pe = 5;
	}
	else
	{
		eFuzzy[0] =0.0;
		pe =5;
	}
	eFuzzy[1] =1.0 - eFuzzy[0];
/***** ���仯������������ *****/
	if(ec<ecRule[0])
	{
		ecFuzzy[0] =1.0;
		pec = 0;
	}
	else if(ecRule[0]<=ec && ec<ecRule[1])
	{
		ecFuzzy[0] = (ecRule[1] - ec)/(ecRule[1]-ecRule[0]);
		pec = 0 ;
	}
	else if(ecRule[1]<=ec && ec<ecRule[2])
	{
		ecFuzzy[0] = (ecRule[2] - ec)/(ecRule[2]-ecRule[1]);
		pec = 1;
	}
	else if(ecRule[2]<=ec && ec<ecRule[3])
	{
		ecFuzzy[0] = (ecRule[3] - ec)/(ecRule[3]-ecRule[2]);
		pec = 2 ;
	}
	else if(ecRule[3]<=ec && ec<ecRule[4])
	{ 
		ecFuzzy[0] = (ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
		pec=3;
	}
	else if(ecRule[4]<=ec && ec<ecRule[5])
	{ 	
		ecFuzzy[0] = (ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
		pec=4;
	}
	else if(ecRule[5]<=ec && ec<ecRule[6])
	{
		ecFuzzy[0] = (ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
		pec=5;
	}
	else
	{
		ecFuzzy[0] =0.0;
		pec = 5;
	}
	ecFuzzy[1] = 1.0 - ecFuzzy[0];
/*********** ��ѯģ������� *************/
	num =KdRule[pe][pec];
	KdFuzzy[num] += eFuzzy[0]*ecFuzzy[0];
	num =KdRule[pe][pec+1];
	KdFuzzy[num] += eFuzzy[0]*ecFuzzy[1];
	num =KdRule[pe+1][pec];
	KdFuzzy[num] += eFuzzy[1]*ecFuzzy[0];
	num =KdRule[pe+1][pec+1];
	KdFuzzy[num] += eFuzzy[1]*ecFuzzy[1];
/******** ��Ȩƽ������ģ�� ********/
	Kd_calcu=KdFuzzy[0]*kdRule[0]+KdFuzzy[1]*kdRule[1]+KdFuzzy[2]*kdRule[2]
	+KdFuzzy[3]*kdRule[3];
	return(Kd_calcu);
}




/********* ������ **********/
void main(void)
{
while(1)
{



	PID_Calculate();
	Kp=fuzzy_kp(Err/5,D_Err); //E �������� 5
	Ki=fuzzy_ki(Err/5,D_Err);
	Kd=fuzzy_kd(Err/5,D_Err);

}

