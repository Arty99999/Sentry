#ifndef __CHECK_H
#define __CHECK_H

#include "stdint.h"
#include "referee.h"
#include "ET08.h"
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
	  uint8_t Check_receiver;//英文意为接收
	  uint8_t Check_vision;//视觉
	  uint8_t Check_referee;//裁判系统
		uint8_t Check_lidar;//雷达

		uint16_t Check_receiver_cnt;//英文意为接收
	  uint16_t Check_vision_cnt;//视觉
	  uint16_t Check_referee_cnt;//裁判系统
		uint16_t Check_lidar_cnt;//雷达
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

void RobotOnlineState(Check_Robot_State *CheckRobotState,RC_Ctrl_ET* rc_ctrl_et);
void FPS_Check(FPS * fps);
void Motor_CheckFPS();
void  Motor_Check(Check_Motor* check,CAN_Object canx) ;

extern Check_Robot_State check_robot_state;
extern FPS tim14_FPS;
#endif  
