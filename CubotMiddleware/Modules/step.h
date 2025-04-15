#ifndef __STEP_H
#define __STEP_H

#include "devices.h"
#include "pid.h"

typedef struct 
{
	uint8_t StepFlag;
	uint8_t Step1FinishFlag;
	uint8_t Step1_StartFinishFlag;
	uint8_t Step2FinishFlag;
	Motor motor[2];
	BasePID_Object StartPID[2];
	BasePID_Object PushPID[2];	
	BasePID_Object ResetPID[2];
	float limit_angle[2];				
	float target_angle[2];			//目标角度
	float angle_out[2];
	float first_angle[2];			//存储初始角度
	float angle[2];					//角度累加后的当前角度
	float last_angle[2];
	uint16_t Start_speed;
	uint16_t Step2_cnt;
	
}Step_t;


float Step_Pid(BasePID_Object* base_pid,float Feedback,float target);
void stepinit(Step_t * step,BasePID_Object step_start_pid,BasePID_Object step_push_pid,BasePID_Object step_reset_pid,CanNumber canx);
void StepGetData_Double(Step_t * step,RC_Ctrl* rc_ctrl);
void StepAnglePlus(Motor* motor,float* angle,float* last_angle);
float Step_Reset_Pid(BasePID_Object* base_pid,float Feedback,float target);
float Step_Pid_Start(BasePID_Object* base_pid,float Feedback,float target);
float Step_Pid_Push(BasePID_Object* base_pid,float Feedback,float target);
void StepGetData_Single(Step_t * step,RC_Ctrl* rc_ctrl);

#endif
