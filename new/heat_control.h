#ifndef __HEAT_CONTROL_H
#define __HEAT_CONTROL_H
#include "stm32h7xx_hal.h"
typedef struct 
{
	float shoot_recover;
	volatile float on_time_heat;//实时热量
	uint8_t flag_referee;
	uint8_t cooling_times;
	uint8_t fire_mode;
	uint8_t over_heat;	
	uint8_t heat_status;
}Heat_Info;
void Heat_Control(void);
uint16_t Get_Max_Heat(uint8_t Robot_Level);
uint16_t Get_Heat_Cold_Speed(uint8_t Robot_Level);
extern  Heat_Info Muzzle;
#define FIRE_ONE_ONETIME 1
#define FIRE_THREE_ONETIME 3
#endif

