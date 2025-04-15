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
	
	float KiPartDetachment;
	float LastError;
	float LastlastError;
	float Out;
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
void DualPID_Init(DualPID_Object* dual_pid, BasePID_Object* ShellPID,BasePID_Object* CorePID);



/**
  * @brief �������������ٶȿ���
  */
int16_t BasePID_SpeedControl(BasePID_Object* base_pid, float target_speed, float feedback_speed);






/**
  * @brief ��������΢�ֽǶȿ���, ���ٶȲ�����IMU���ݣ��������������нǶȿ���
  */
float  BasePID_AngleControl(BasePID_Object* base_pid, float target_angle, float feedback_angle);
#endif

