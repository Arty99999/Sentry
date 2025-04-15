#ifndef __PID_H_
#define __PID_H_

#include "stm32h7xx_hal.h"
#include "referee.h"



/**
  * @brief  单环PID 
  */
typedef struct 
{	
	float Kp, Ki, Kd;
	float Error;
	float iError;//累积误差
	float KpPart, KiPart, KdPart;
	
	float KiPartDetachment;
	float LastError;
	float LastlastError;
	float Out;
}BasePID_Object;


/**
  * @brief  双环PID
  */
typedef struct 
{
	BasePID_Object* ShellPID;
	BasePID_Object* CorePID;
}DualPID_Object;
 

/**
  * @brief  		限幅
  * @param[in]	input		输入量
  * @param[in]	output	输出量
  */
float AmplitudeLimit(float input,float amplitude);


/**
  * @brief 单环PID初始化
  */
void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach);


/**
  * @brief 双环PID初始化
  */
void DualPID_Init(DualPID_Object* dual_pid, BasePID_Object* ShellPID,BasePID_Object* CorePID);



/**
  * @brief 单环比例积分速度控制
  */
int16_t BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);






/**
  * @brief 单环比例微分角度控制, 角速度不依靠IMU数据，仅靠编码器进行角度控制
  */
float  BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle);
#endif

