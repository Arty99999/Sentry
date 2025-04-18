#ifndef __PID_H_
#define __PID_H_

#include "stm32h7xx_hal.h"
#include "referee.h"



/**
  * @brief  ����PID 
  */
typedef struct 
{	
	float Kp, Ki, Kd;
	float Error;
	float iError;//�ۻ����
	float KpPart, KiPart, KdPart;
	float Out;
	float KiPartDetachment;
	float LastError;
	float LastlastError;
}BasePID_Object;


/**
  * @brief  ˫��PID
  */
typedef struct 
{
	BasePID_Object* ShellPID;
	BasePID_Object* CorePID;
}DualPID_Object;
 

/**
  * @brief  		�޷�
  * @param[in]	input		������
  * @param[in]	output	�����
  */
float AmplitudeLimit(float input,float amplitude);


/**
  * @brief ����PID��ʼ��
  */
void BasePID_Init(BasePID_Object* base_pid, float kp, float ki, float kd, float detach);


/**
  * @brief ˫��PID��ʼ��
  */
void DualPID_Init(DualPID_Object* dual_pid, float shell_kp, float shell_ki, float shell_kd, float core_kp, float core_ki, float core_kd, float detach);



/**
  * @brief �������������ٶȿ���
  */
float BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);



/**
  * @brief �������������ٶȿ���, ���ٶȲ�����IMU���ݣ��������������нǶȿ���
  */
int32_t BasePID_AngleControlWithoutIMU(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);


/**
  * @brief ��������΢�ֽǶȿ���, ���ٶȲ�����IMU���ݣ��������������нǶȿ���
  */
int32_t BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);
int32_t BasePID_AngleControlFollow(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);
int32_t BasePID_HoldControl(BasePID_Object* base_pid, float target_angle, float feedback_angle, float feedback_angle_speed);
float BasePID_PitchAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle);
int32_t BasePID_PitchSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);
float BasePID_YawAngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle);
int32_t BasePID_YawSpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);
int32_t BasePID_PowerControl_Vmax(BasePID_Object* base_pid,float target_power);
float BasePID_BaseControl(BasePID_Object* base_pid, float Chassis_Power_Buffer, float Chassis_Power);
extern BasePID_Object pid_step_start_Speed;
extern BasePID_Object pid_step_Push_Angle;
extern BasePID_Object pid_step_reset_Angle;

extern BasePID_Object pid_speed;

extern BasePID_Object pid_angle;
extern BasePID_Object pid_yawreset;
extern BasePID_Object pid_pitchreset;

extern BasePID_Object pid_yaw_angle;
extern BasePID_Object pid_yaw_speed;
extern BasePID_Object pid_yaw_vision_angle;
extern BasePID_Object pid_yaw_vision_speed;
extern BasePID_Object pid_yaw_vision_angle1;
extern BasePID_Object pid_yaw_vision_speed1;

extern BasePID_Object pid_follow;
extern BasePID_Object pid_base;
extern BasePID_Object pid_power; 
extern BasePID_Object pid_pitch_angle;
extern BasePID_Object pid_pitch_speed;
extern BasePID_Object pid_pitch_vision_angle;
extern BasePID_Object pid_pitch_vision_speed;
extern BasePID_Object pid_pitch_vision_angle1;
extern BasePID_Object pid_pitch_vision_speed1;
extern BasePID_Object pid_pitch_vision_angle2;
extern BasePID_Object pid_pitch_vision_speed2;

extern BasePID_Object pid_load;
extern BasePID_Object pid_friction;
extern BasePID_Object pid_friction1;
extern BasePID_Object pid_shoot;
extern BasePID_Object pid_shoot1;
extern BasePID_Object pid_ChassisPower;
extern BasePID_Object pid_ChassisPowerVmax;
extern BasePID_Object pid_ChassisPowerOmegamax;

extern BasePID_Object pid_yawreset_angle;
extern BasePID_Object pid_yawreset_speed;

extern BasePID_Object pid_camara_speed;
extern BasePID_Object pid_camara_angle;

extern BasePID_Object run_pid;
#endif

