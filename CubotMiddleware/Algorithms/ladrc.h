#ifndef _LADRC_H
#define _LADRC_H
#include "stdint.h"

#define square(a) ((a)*(a))
#define cube(a) ((a)*(a)*(a))
/**
   *@Brief  ����ΪLADRCϵͳ����
   *@WangShun  2022-07-03  ע��
   */
typedef struct LADRC
{
    double v1,v2;         //�������ֵ
    double r;             //�ٶ�����
    double h;             //���ֲ���
    double z1,z2,z3;      //�۲������
		double w0,beta;				//�����������
    double wc,b0,u;    //�۲������� ���������� ϵͳ���� ���������
		double Kp,Kd,Ki;
		double u0;
	double Kipart,Kdpart,Kppart;
}LADRC_NUM;

/*
	wu = 2*3.1415/Pu;
    ku = 4*h/3.1415*a;

	wc = 2.3997wu - 0.4731;
	w0 = 0.7332wu + 3.5070;
	b0 = 3.6105wu + 4.8823;
*/
typedef struct Auto_Tuning 
{
	float Pu; //�̵�ʵ���������
	float a;  //�̵�ʵ�������ֵ
	float h;  //ָ�������ֵ
	float Wu; //ϵͳ�ٽ�Ƶ��
	float Kp; //ϵͳ�ٽ��ֵ
}AuTu;

/**
   *@Brief  ����Ϊ��Ҫ�����Ĳ���
   *@WangShun  2022-07-03  ע��
   */
extern  LADRC_NUM M6020_Angle_Yaw;
extern  LADRC_NUM M6020_Angle_Yaw1;
extern LADRC_NUM  M3508_Angle;
extern LADRC_NUM  M3508_Speed; 
extern LADRC_NUM  M6020_Angle;
extern LADRC_NUM  M6020_Angle_ZOH;
/**
   *@Brief  ����ΪLADRC��غ���
   *@WangShun  2022-07-03  ע��
   */
void LADRC_Init(LADRC_NUM *LADRC_TYPE1);
void LADRC_REST(LADRC_NUM *LADRC_TYPE1);
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect);
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,int16_t FeedBack);
void LADRC_LF(LADRC_NUM *LADRC_TYPE1,float y,float y_dot);
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,float* Expect,float* RealTimeOut);
void LADRC_ZOH_Loop(LADRC_NUM *LADRC_TYPE1,double Expect_Value,double RealTimeOut,double Measure,double Measure_D);
#endif
