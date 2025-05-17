#ifndef BRAIN_H__
#define BRAIN_H__
#include "stm32h7xx.h"
#include "usart.h"
#include "dr16.h"


#define Brain_rxBufferLengh 50
#define Lidar_rxBufferLengh 50
  

/**
  * @brief    内核的回调函数，用于在结算数据后实时执行指令
  * @param[in]  
  */
//typedef uint8_t (*Brain_CoreCallback)(Holder_t* holder, RC_Ctrl* rc_ctrl); 

typedef enum
{
	BRAIN_TO_ROBOT_CMD   = 1,	 //< 0b0001
	BRAIN_TO_ROBOT_HINT  = 2,	 //< 0b0010
	ROBOT_TO_BRAIN_QUEST = 3,  //< 0b0011
	ROBOT_TO_BRAIN_LOG   = 4,	 //< 0b0100
	ROBOT_TO_BRAIN_CMD   = 5
}BrainFrameType;
typedef enum
{
	Autoaim   = 0,             
  Outpost =1

}
Brain_mode;
typedef enum
{
	Cruise   = 0,             
  Lock =1,
	Change=2

}Brain_Autoaim_mode;   
typedef enum
{
  Lidar_home=1,
	Lidar_Outpost=2,
	Lidar_Patrol=3,
	Lidar_Fortress=4
	
}Brain_Lidar_mode;
typedef enum
{
	None=0,
	Found=1,
	Wait=2
}Brain_All_See_mode;  
typedef struct
{ 
	uint8_t FrameType;
	uint8_t FrameCoreID;
}CubotBrain_t;   
typedef struct
{
	
	struct{
		Brain_Autoaim_mode mode;
		Brain_Autoaim_mode Last_mode;
		uint16_t mode_cnt[3];
		CubotBrain_t Brain_Data;
		Brain_mode  Mode;
		float Pitch_add;
		float Yaw_add;
		float Distance;
		uint8_t camara_num;
		uint8_t fire_flag;
		float Limit;
		
		float Use_Can_angle;
		float Send_Can_angle[256];
		
		float Use_Gyro_angle;
		float Send_Gyro_angle[256];
		uint8_t Ignore_armorNumber;
			uint8_t IsFire;
	}Autoaim;
	 struct{
		Brain_Lidar_mode mode;
		uint16_t mode_cnt[2];
		CubotBrain_t Brain_Data;
		uint8_t	movemode;
		int16_t vx;
    int16_t vy;	
    float angle_to_lidar;
    uint8_t Arrive;
	}Lidar;
	 struct{
		 Brain_All_See_mode mode;
		uint16_t mode_cnt[3];
		 uint8_t Find_size;
		 uint8_t Camera_Index[10];
		 uint8_t armorNumber[10];
		 float Pitch_add[10];
		float Yaw_add[10];
		float Distance[10];//mm
	}All_See;
	
}Brain_t;
  


void RobotToBrain(Brain_t* Brain);
extern Brain_t Brain;
uint8_t Brain_Autoaim_callback(uint8_t * recBuffer, uint16_t len);
uint8_t Brain_Lidar_callback(uint8_t * recBuffer, uint16_t len);
uint8_t Brain_Camera_callback(uint8_t * recBuffer, uint16_t len);
void Brain_Autoaim_DataUnpack(Brain_t* Brain ,uint8_t * recBuffer);//解包自瞄数据
void Brain_Lidar_DataUnpack(Brain_t* Brain ,uint8_t * recBuffer);//解包雷达数据
void Brain_Camera_DataUnpack(Brain_t* Brain ,uint8_t * recBuffer);//解包雷达数据
void Brain_Camera_DataUnpack_New(Brain_t* Brain ,uint8_t * recBuffer);
void RobotToBrain_Autoaim(float yaw,Brain_t* brain);//发给自瞄	
extern 	UART_RxBuffer uart2_buffer;
extern 	UART_RxBuffer uart5_buffer;
void RobotToBrain_Lidar(Brain_t* Brain);//发给雷达
void Change_BrainMode(Brain_t* Brain);
#endif

