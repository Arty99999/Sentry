#ifndef CONTROLLOGIC_H_
#define CONTROLLOGIC_H_
#include "stm32h7xx_hal.h"

#include "driver.h"

#include "motor.h"



#include "pid.h"

uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer);

uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer);


void TIM14_Task(void);
void TIM13_Task(void);
extern  uint16_t Reset_Count;
extern int asd;
extern float sens_pitch_change;
extern int time_cut;
extern int16_t ThisSecond;

extern int16_t roll_time;



#endif



