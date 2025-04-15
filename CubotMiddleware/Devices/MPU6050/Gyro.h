#ifndef GYRO_H
#define GYRO_H
#include "stm32h7xx_hal.h"
#include "holder.h"
#include "user_lib.h"
#define HWT_rxBufferLengh 100
typedef struct 
{
  int32_t gyro_w_yaw;  
	int32_t gyro_w_pitch;
	float yaw_angle;
	float yaw_angle_last;
	float yaw_speed;
  float pitch_angle;
	float pitch_speed;
  float sens_pitch;
	float sens_yaw;
	float cqhlp;
  uint32_t cnt;
}gyro_data_t;

typedef struct
{
	float MpuAngle1;
	float CanAngle1;
	float MpuAngle2;
	float CanAngle2;
	float MpuAngle3;
  float CanAngle3;
	float MpuAngleChange;
	float CanAngleChange;
	float SenceBili;
}GetAngle;

typedef struct
{
	GetAngle Yaw;
	GetAngle Pitch;	
	uint16_t mpuDebugTime;
	uint8_t  DebugBeginFlag;
	uint8_t  DebugFinishFlag;
}MpuDebug;



uint8_t Gyro_callback(uint8_t * recBuffer, uint16_t len);
void angle_deal(uint8_t *angle_receive,Holder_t* holder);
void HWT906_Data_Deal(uint8_t *data_receive,Holder_t* holder);
void HolderAngleChange(Holder_t *holder);

	//extern  UART_RxBuffer uart4_buffer;


#endif
