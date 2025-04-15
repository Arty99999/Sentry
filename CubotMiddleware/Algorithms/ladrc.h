#ifndef _LADRC_H
#define _LADRC_H
#include "stdint.h"

#define square(a) ((a)*(a))
#define cube(a) ((a)*(a)*(a))
/**
   *@Brief  以下为LADRC系统参数
   *@WangShun  2022-07-03  注释
   */
typedef struct LADRC
{
    double v1,v2;         //最速输出值
    double r;             //速度因子
    double h;             //积分步长
    double z1,z2,z3;      //观测器输出
		double w0,beta;				//反馈增益参数
    double wc,b0,u;    //观测器带宽 控制器带宽 系统参数 控制器输出
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
	float Pu; //继电实验输出周期
	float a;  //继电实验输出幅值
	float h;  //指令输出幅值
	float Wu; //系统临界频率
	float Kp; //系统临界幅值
}AuTu;

/**
   *@Brief  以下为需要整定的参数
   *@WangShun  2022-07-03  注释
   */
extern  LADRC_NUM M6020_Angle_Yaw;
extern  LADRC_NUM M6020_Angle_Yaw1;
extern LADRC_NUM  M3508_Angle;
extern LADRC_NUM  M3508_Speed; 
extern LADRC_NUM  M6020_Angle;
extern LADRC_NUM  M6020_Angle_ZOH;
/**
   *@Brief  以下为LADRC相关函数
   *@WangShun  2022-07-03  注释
   */
void LADRC_Init(LADRC_NUM *LADRC_TYPE1);
void LADRC_REST(LADRC_NUM *LADRC_TYPE1);
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect);
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,int16_t FeedBack);
void LADRC_LF(LADRC_NUM *LADRC_TYPE1,float y,float y_dot);
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,float* Expect,float* RealTimeOut);
void LADRC_ZOH_Loop(LADRC_NUM *LADRC_TYPE1,double Expect_Value,double RealTimeOut,double Measure,double Measure_D);
#endif
