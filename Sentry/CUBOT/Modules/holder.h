#ifndef HOLD_H
#define HOLD_H
#include "stm32h7xx_hal.h"
#include "devices.h"
#include "pid.h"
#include "brain.h"
#include "ET08.h"
extern float a12;
struct Holder_Motor_Info
{
	float Target_Angle;

	float GYRO_Angle;
	float GYRO_Angle_speed;
	
  float Can_Angle;
	int16_t Can_Angle_speed;//RPM
	float Sensitivity;
	float MouseSensitivity;
	DualPID_Object PID;
};

typedef struct 
{
	struct Holder_Motor_Info Pitch;
	struct Holder_Motor_Info Yaw;
	struct Holder_Motor_Info Yaw1;
	struct 
	{
		Motor motor[3];						
		BasePID_Object turnPID[10];		//< 转向角度控制结构体
		float FeedbackAngle[3];
		float HolderCanAngle[3];
	}Motors6020;
	struct{
	uint32_t pitch_time;	
	uint32_t yaw1_time;	
	float pitch_sense;	
	float yaw1_sense;		
	}Cruise_Mode;
	struct{
	uint16_t Lock_cnt;	
	uint8_t Flag_Fllow;	
	}Yaw_Fllow_Mode;
	float up_litmit;
	float down_litmit;
	float right_litmit;
	float left_litmit;
}Holder_t;

typedef enum {
    HURT_IDLE = 0,      
    HURT_ATTACKED,     //3508下拉    
    HURT_DOUBT,     //舵机合拢

}
Hurt_state;
extern  Holder_t Holder;
//void HolderYawChassisInit(Holder_t* holder, CanNumber canx);
//void HolderReset(Holder_t* holder);
void HolderInit(Holder_t* holder,DualPID_Object* pitch_pid ,DualPID_Object* yaw_pid,DualPID_Object* yaw1_pid,CanNumber canx);
void HolderGetRemoteData(Holder_t* holder, RC_Ctrl_ET* rc_ctrl,Brain_t* brain) ;
void thinchicken_feedback_control(void);

#endif
