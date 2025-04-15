#include "pid.h"
#include "driver_timer.h"
#include "init.h"
#include "dr16.h"
#include "filter.h"
#include "brain.h"
#include "hardware_config.h"
BasePID_Object pid_step_start_Speed;
BasePID_Object pid_step_Push_Angle;
BasePID_Object pid_step_reset_Angle;

BasePID_Object pid_speed;

BasePID_Object pid_angle;
BasePID_Object pid_yawreset;
BasePID_Object pid_pitchreset;

BasePID_Object pid_yaw_angle;
BasePID_Object pid_yaw_speed;
BasePID_Object pid_yaw_vision_angle;
BasePID_Object pid_yaw_vision_speed;
BasePID_Object pid_yaw_vision_angle1;
BasePID_Object pid_yaw_vision_speed1;

BasePID_Object pid_follow;
BasePID_Object pid_base;
BasePID_Object pid_power; 
BasePID_Object pid_pitch_angle;
BasePID_Object pid_pitch_speed;
BasePID_Object pid_pitch_vision_angle;
BasePID_Object pid_pitch_vision_speed;
BasePID_Object pid_pitch_vision_angle1;
BasePID_Object pid_pitch_vision_speed1;
BasePID_Object pid_pitch_vision_angle2;
BasePID_Object pid_pitch_vision_speed2;

BasePID_Object pid_load;
BasePID_Object pid_friction;
BasePID_Object pid_friction1;
BasePID_Object pid_shoot;
BasePID_Object pid_shoot1;
BasePID_Object pid_ChassisPower;
BasePID_Object pid_ChassisPowerVmax;
BasePID_Object pid_ChassisPowerOmegamax;

BasePID_Object pid_yawreset_angle;
BasePID_Object pid_yawreset_speed;

BasePID_Object pid_camara_speed;
BasePID_Object pid_camara_angle;

BasePID_Object run_pid;





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
void DualPID_Init(DualPID_Object* dual_pid, float shell_kp, float shell_ki, float shell_kd, float core_kp, float core_ki, float core_kd, float detach)
{
	dual_pid->ShellPID->KiPartDetachment = detach;
	dual_pid->CorePID->KiPartDetachment = detach;
	
	dual_pid->ShellPID->Kp = shell_kp;
	dual_pid->ShellPID->Ki = shell_ki;
	dual_pid->ShellPID->Kd = shell_kd;
	
	dual_pid->CorePID->Kp = core_kp;
	dual_pid->CorePID->Kp = core_ki;	
	dual_pid->CorePID->Kp = core_kd;
	
	dual_pid->ShellPID->KpPart = 0;
	dual_pid->ShellPID->KiPart = 0;
	dual_pid->ShellPID->KdPart = 0;
	
	dual_pid->CorePID->KpPart = 0;
	dual_pid->CorePID->KiPart = 0;
	dual_pid->CorePID->KdPart = 0;

}


/**
  * @brief 单环比例积分速度控制
  */
float BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
	
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
	
	base_pid->Out = base_pid->KpPart + base_pid->KiPart;
	
	return base_pid->Out;
}

/**
  * @brief 单环比例积分角度控制   
  */

int32_t BasePID_HoldControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed)
{
	base_pid->Error = target_angle - feedback_angle;
	if(tim14.ErrorTime == 20000&&base_pid->Error>3.5f){
		base_pid->Error=3.5f;
	}
	if(tim14.ErrorTime == 20000&&base_pid->Error<-3.5){
		base_pid->Error=-3.5;
	}
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	if(tim14.HolderTime < 15000){
	base_pid->KiPart = 0;
	}
	base_pid->KiPartDetachment = 1000;
	if(base_pid->Error > base_pid->KiPartDetachment)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * feedback_angle_speed;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief 单环比例积分角度控制   
  */
int32_t BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed)
{
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	if(tim14.HolderTime < 15000){
	base_pid->KiPart = 0;
	}
	base_pid->KiPartDetachment = 1000;
	if(base_pid->Error > base_pid->KiPartDetachment)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * feedback_angle_speed;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}


/**
  * @brief 跟随
  */
int32_t BasePID_AngleControlFollow(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed)
{
	base_pid->Error = target_angle - feedback_angle;
	
	if(base_pid->Error>60)
		base_pid->Error=60;
	if(base_pid->Error<-60)
		base_pid->Error=-60;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;

	if(tim14.HolderTime < 15000)
		base_pid->KiPart = 0;

	base_pid->KiPartDetachment = 1000;
	if(base_pid->Error > base_pid->KiPartDetachment)
	{
		base_pid->KiPart = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment))
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * feedback_angle_speed;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}


/**
  * @brief pitch角度环  双环外环
  */
float BasePID_PitchAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle)
{

	
	base_pid->Error = target_angle - feedback_angle;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->iError += base_pid->Error ;
	
//	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->iError  = 0;
	}
	else if(base_pid->Error < -(base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->iError  = 0;
	}
		if(base_pid->iError >= 400)//限位
    	base_pid->iError = 400;
    	else if(base_pid->iError <= -400)
    	base_pid->iError = -400;	
			
	base_pid->KiPart = base_pid->iError * base_pid->Ki;
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
	
	return base_pid->Out;
}

/**
  * @brief pitch速度环  双环内环
  */
int32_t BasePID_PitchSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed)
{
	base_pid->Error = target_speed - feedback_speed;
//	if((base_pid->Error>-0.1)&&(base_pid->Error<0.1))
//		base_pid->Error=0;
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
	base_pid->KiPart += base_pid->Error * base_pid->Ki;
	base_pid->KiPartDetachment = 50;
	if(base_pid->Error > base_pid->KiPartDetachment||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	else if((base_pid->Error < -base_pid->KiPartDetachment)||rc_Ctrl.isOnline == 0)
	{
		base_pid->KiPart = 0;
	}
	
	base_pid->KdPart = (-1) * base_pid->Kd * (base_pid->Error - base_pid->LastError);
	base_pid->LastError = base_pid->Error;
	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
//	if((Brain.FrameType ==1)&&(Vision_Info.Hit_Mode==2))
//			base_pid->Out = LPFilter(base_pid->Out ,&LPF_pitch_vision);
//	else
//	    base_pid->Out = LPFilter(base_pid->Out ,&LPF_pitch_speed);
	return base_pid->Out;
}


float kdpartlimit=10;
/**
  * @brief yaw角度环  双环外环
  */
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
	