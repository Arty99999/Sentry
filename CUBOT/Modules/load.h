#ifndef LOAD_H
#define LOAD_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "devices.h"
#include "vision.h"

typedef struct  
{
	int16_t Target_Firing_Rate;   		//拨弹盘转速
	int32_t  Shoot_sum_cut;				//摩擦轮缓启动计数
	uint8_t SuperHeatMode;				//超热量模式标志位 1为进入超热量模式
	uint16_t Angle_inversion;			
	volatile long int Shoot_Target_Sum;      //目标发射弹丸总数  //应当打的弹数
	uint8_t Crowd_back_Flag;                 //反转标志位	Crowd_back_Flag=0正常，Crowd_back_Flag=1卡弹
	uint8_t Stop_Back_Flag;									//热量判断标志位 0为热量足
//	uint8_t BulletCap_Status_Flag;           //开启弹舱盖标志位，1为开
		struct 
	{
		Motor motor[4];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
	}Motors3508;
			struct 
	{
		Motor motor[1];						//< 底盘电机结构体
		BasePID_Object RunPID[4];			//< 速度控制PID参数
	}Motors2006;
	
}Shoot_Info;

typedef struct            //摩擦轮拨弹盘信息
{
	float Target_Speed[3];       //0和1分别是左右摩擦轮的转速，2是拨弹盘的转速
	float Angle_Plate;			//电机已经转过的转角
	float Angle_Target;     //实际在没卡弹情况下应当转的角度，包含已经转过的角度和即将转过的角度	
	float Oneshoot_Angle;     //电机在单发情况下的需要转的转角
	float ThreeAngle;				//电机在三发情况下的需要转的转角
	float Angle;						//电机在连发情况下应当转的转角，包含过去已经转过的转角
	uint16_t  Inversion_Cnt;  //拨弹盘卡弹计数
	float Current_1[4];				//最终发送给电机的数据
	float delta_angle;				//速度积分得来的每时每刻转动的角度
}Friction_Load_Info;

//底盘小陀螺模式结构体，我也不知道为啥在这
typedef struct
{
	uint8_t Roll_Flag;
	uint8_t Roll_Flag1;
	uint8_t Rock_Flag;
	uint8_t Remake_Flag;
	uint8_t Chassis_State;
	float Chassis_Power;
	float Chassis_Power_Buff;
	float Chassis_Out_Sum;
	float Chassis_Allowed_Out;
}Chassis_Attitude_Info;

struct Pid_Info
{	
	float Kp_Pid;//比例系数
	float Ki_Pid;//积分系数
	float Kd_Pid;//微分系数  
	float E0_Pid;//差值
	float Last_E0_Pid;
	float Last_Last_E0_Pid;
	float Ep_Part_Pid;//比例部分控制量
	float Ei_Part_Pid;//积分部分控制量
	float Ed_Part_Pid;//微分部分控制量
	float Out_Pid;//最后输出值
	float Target_Pid; //目标值
	float Feedback_Pid;//反馈值
};
extern Shoot_Info Shoot;
extern Friction_Load_Info Booster;
extern Friction_Load_Info Motor_Booster;
extern uint16_t Friction_Start_Cnt;
extern struct Pid_Info Pid_Load;
extern struct Pid_Info Pid_Friction;

void LoadInit(Shoot_Info* shoot, BasePID_Object friction_pid, BasePID_Object load_pid, CanNumber canx);
//void Friction_Load_Fire_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,Trace* Info_Vision);
void Shoot_Fire_Mode_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot);
float Friction_Load_Pid_Control(BasePID_Object* base_pid,float Feedback,int16_t target,uint8_t who);

#endif
