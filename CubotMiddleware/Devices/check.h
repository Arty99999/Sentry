#ifndef __CHECK_H
#define __CHECK_H

#include "stdint.h"
#include "referee.h"
#include "dr16.h"
#include "motor.h"
typedef	 struct 
	{	
	 uint16_t* Online;
	 uint8_t size_Online;
	 uint16_t* Offline;
	 uint8_t size_Offline;
	}Check_Motor;
typedef struct
{
	struct
	{
	  uint8_t Check_receiver;//Ӣ����Ϊ����
	  uint8_t Check_vision;//�Ӿ�
	  uint8_t Check_referee;//����ϵͳ
		uint8_t Check_lidar;//�״�

		uint16_t Check_receiver_cnt;//Ӣ����Ϊ����
	  uint16_t Check_vision_cnt;//�Ӿ�
	  uint16_t Check_referee_cnt;//����ϵͳ
		uint16_t Check_lidar_cnt;//�״�
	}Check_Usart;
	
  Check_Motor Check_Can1;
	Check_Motor Check_Can2;
}Check_Robot_State;

typedef struct
{
	uint16_t Vision_FPS;
	uint16_t Lidar_FPS;
	uint16_t Receiver_FPS;
	uint16_t Referee_FPS;
	uint16_t Gyro_In_FPS;
	uint16_t Gyro_Out_FPS;
	uint16_t Camera_FPS;
	
	uint16_t Receiver_cnt;
	uint16_t Referee_cnt;
	uint16_t Vision_cnt;
  uint16_t Lidar_cnt;
	uint16_t Gyro_In_cnt;
	uint16_t Gyro_Out_cnt;
	uint16_t Camera_cnt;
}FPS;

void RobotOnlineState(Check_Robot_State *CheckRobotState,RC_Ctrl *rc_ctrl);
void FPS_Check(FPS * fps);
void Motor_CheckFPS();
void  Motor_Check(Check_Motor* check,CAN_Object canx) ;

extern Check_Robot_State check_robot_state;
extern FPS tim14_FPS;
#endif  
