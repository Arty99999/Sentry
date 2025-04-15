#ifndef INIT_H
#define INIT_H
#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "user_lib.h"
#define Yaw_Integ_Sense 0.009f
#define Pitch_Integ_Sense 0.0001f
#define BMI088_Wait_Time 3000

#define FRICTION_SPEED_LOW 4850
#define FRICTION_SPEED_MIDLE 6100
#define FRICTION_SPEED_HIGH 7350

extern int32_t Clock_time;

#define ROCK_TIME_SIN 45.0f	 //修改此值调整摇摆的速度
#define  Yaw_Base_Can_Angle    650 //云台正中时yaw的编码器值
#define  Pitch_Base_Can_Angle 2893    //云台正中时pitch的编码器值
#define YAW_ENCODER_FULL_RANGE  8192 //编码器满量程
#define PITCH_ENCODER_FULL_RANGE 8192
#define Chassis_ENCODER_FULL_RANGE 8192
#define Chassis_Base_Can_Angle1 6821
#define Chassis_Base_Can_Angle2 7583
#define Chassis_Base_Can_Angle3 5480
#define Chassis_Base_Can_Angle4 4921
#define K_Code_Pitch  0.043945f   //  360/8192
#define K_Code_Yaw  0.043945f   //  360/8192
#define FRI_LEFT 0					// in "Motor_Booster.Target_Speed[0]" staies the target angle speed for left friction
#define FRI_RIGHT 1					//
#define LOADER 2						// 
#define FIRE_ONE_ONETIME 1
#define FIRE_THREE_ONETIME 3
#define  PITCH_NEGTIVE_LIMIT_ANGLE 39.0f		//云台下方角度限制
#define  PITCH_POSITIVE_LIMIT_ANGLE -30.0f	//云台上方角度限制

#endif
