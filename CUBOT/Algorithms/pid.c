#include "pid.h"
#include "driver_timer.h"

#include "dr16.h"
#include "filter.h"
#include "brain.h"
#include "hardware_config.h"



/**
  * @brief  限幅
  */
float AmplitudeLimit(float input,float amplitude)
{
	if(input<-amplitude)
		return -amplitude;
	else if(input>amplitude)
		return amplitude;
	else return input;
}


/**
  * @brief 单环PID初始化
  */
void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach)
{
	base_pid->KiPartDetachment = detach;
	
	base_pid->Kp = kp;
	base_pid->Ki = ki;
	base_pid->Kd = kd;

	base_pid->KpPart = 0;
	base_pid->KiPart = 0;
	base_pid->KdPart = 0;
	
}

/**
  * @brief 双环PID初始化
  */
void DualPID_Init(DualPID_Object* dual_pid, BasePID_Object* ShellPID,BasePID_Object* CorePID)
{
	dual_pid->ShellPID = ShellPID;
	dual_pid->CorePID=CorePID;
}


/**
  * @brief 单环比例积分速度控制
  */
int16_t BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	
if (fabs(base_pid->Error) > base_pid->KiPartDetachment) base_pid->KiPart = 0;
 
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->Out = base_pid->KpPart + base_pid->KiPart+base_pid->KdPart;
	base_pid->LastError=base_pid->Error;
	return base_pid->Out;
}


/**
  * @brief 单环比例积分角度控制   
  */
float  BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;

  if (fabs(base_pid->Error) > base_pid->KiPartDetachment) base_pid->KiPart = 0;
	
	base_pid->KdPart = (-1) * base_pid->Kd * feedback_angle;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	base_pid->LastError=base_pid->Error;
	return base_pid->Out;
}





float BasePID_YawAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->iError += base_pid->Error;
	base_pid->KiPart=base_pid->iError*base_pid->Ki;
//	base_pid->KiPartDetachment = 0.8;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->iError = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0))
	{
		base_pid->iError = 0;
	}

	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
//	if(base_pid->KdPart<kdpartlimit)base_pid->KdPart=0;
//	else if(base_pid->KdPart>-kdpartlimit)base_pid->KdPart=0;
	base_pid->LastError = base_pid->Error;

	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief yaw速度环  双环内环
  */
int32_t BasePID_YawSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief pitch角度环  双环外环 增量式PID
  */
int32_t BasePID_increment_PitchAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart=(base_pid->Error-base_pid->LastError)*base_pid->Kp;
	base_pid->KiPart=base_pid->Error * base_pid->Ki;
	base_pid->KdPart= base_pid->Kd * (base_pid->Error - 2*base_pid->LastError+base_pid->LastlastError);

	base_pid->LastlastError=base_pid->LastError;
	base_pid->LastError = base_pid->Error;

	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}


/**
  * @brief 功率控制PID
  */
int32_t BasePID_PowerControl_Vmax(BasePID_Object* base_pid,float target_power)
{
	base_pid->Error = target_power-referee2022.power_heat_data.chassis_power;
	base_pid->KpPart = base_pid->Error*base_pid->Kp;
	
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KdPart = base_pid->Kd * (base_pid->Error -base_pid->LastError);

	base_pid->LastError = base_pid->Error;

	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}



float BasePID_BaseControl(BasePID_Object* base_pid, float Chassis_Power_Buffer, float Chassis_Power)
	{
	if(Chassis_Power_Buffer>=20)
	{
	float Target_Power_Buff=20;       //缓冲能量剩余目标值为20
  base_pid->Error = -Target_Power_Buff+Chassis_Power_Buffer;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	if(base_pid->Error > base_pid->KiPartDetachment)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment))
	{
		base_pid->KiPart = 0;
	}
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	return base_pid->Out;
  }
	else
	return 0;
}
	