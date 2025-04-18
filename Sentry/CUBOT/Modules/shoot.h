#ifndef SHOOT_H
#define SHOOT_H

#include "stm32h7xx_hal.h"
#include "motor.h"
#include "pid.h"
#include "brain.h"
typedef struct
{
	struct
	{
		Motor motor2006;
		BasePID_Object RunPID;
		
		float Delta_Angle;
		float Plate_Angle;
		float Target_Angle;
		float Angle_Sense;
		float Plate_Out;
		uint32_t Shoot_Cut;
		uint16_t Fire_Rate;
		uint8_t  Fire_Divider;
		uint8_t  Fire_Margin;

		int16_t ShootNum;
		uint8_t  Shoot_rest_flag;
		uint8_t heat_status;
	}Shoot_Plate;
	
	struct
	{
		Motor motor3508[2];		
		BasePID_Object Friction_PID[2];			
		
		float Friction_Target_Speed[2];
		float Friction_Out[2];
		int16_t Friction_Speed[2];	
		int16_t Friction_Start;
	}Friction_Wheel;
	
}Ammo_Booster;


void AmmoBoosterInit(Ammo_Booster *ammo_booster,BasePID_Object* friction_pid, BasePID_Object* friction_pid1,BasePID_Object* load_pid);

void ShootPlateControl(Ammo_Booster *ammo_booster,Brain_t* brain);
void FrictionWheelControl(Ammo_Booster *ammo_booster);

extern Ammo_Booster AmmoBooster;

#endif
