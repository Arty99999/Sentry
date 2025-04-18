#include "hardware_config.h"
#include "stm32h7xx_hal.h"
#include "driver.h"

#include "brain.h"
#include "bmi088.h"
#include "shoot.h"
#include "mpu6050.h"
#include "all_chassis.h"
#include "ins.h"
#include "dr16.h"
#include "pid.h"
#include "holder.h"
#include "referee.h"
#include "brain.h"
#include "check.h"

#include "shoot.h"
#include "check.h"

#include "control_logic.h"
BasePID_Object pid_yaw_angle;
BasePID_Object pid_yaw_speed;
BasePID_Object pid_yaw1_angle;
BasePID_Object pid_yaw1_speed;


BasePID_Object pid_follow;

BasePID_Object pid_pitch_angle;
BasePID_Object pid_pitch_speed;


BasePID_Object pid_load;
BasePID_Object pid_friction;
BasePID_Object pid_friction1;


BasePID_Object run_pid;

BasePID_Object pid_base;

void MPU_Init_(void);
uint8_t Vofa_callback(uint8_t * recBuffer, uint16_t len);
void MPU_Init_(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();//???I-Cache
  SCB_EnableDCache();//???D-Cache   
	SCB->CACR|=1<<2;   //???D-Cache?д,?粻????,???????п???????????????	
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;//????????? 1
	MPU_Config.BaseAddress= 0x24000000;//???????????
	MPU_Config.Size=MPU_REGION_SIZE_512KB;//?????????512k
	MPU_Config.SubRegionDisable=0x00;//?????????
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;//??????????????level0
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;//?????????&???????????
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;//??????????
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;//????????
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;//????cache
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;//?????????? DMA??????????????
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}	
uint8_t vofa_callback(uint8_t * recBuffer, uint16_t len)
{
	return 0;
}


void HardwareConfig(void)
{
	
  DualPID_Object pid_yaw={.ShellPID=&pid_yaw_angle,.CorePID=&pid_yaw_speed};
  DualPID_Object pid_yaw1={.ShellPID=&pid_pitch_angle,.CorePID=&pid_pitch_speed};
  DualPID_Object pid_pitch={.ShellPID=&pid_yaw1_angle,.CorePID=&pid_yaw1_speed};
	
	//< 推荐先进行设备初始化，再进行硬件初始化
	MPU_Init_();
	DR16Init(&rc_Ctrl);
	
  DWT_Init(480);


	
	BasePID_Init(&pid_follow,60,0 ,5,0);//-50，0，-10，0
	
	
	
	
	BasePID_Init(&pid_load,8,0 ,0, 0);
	BasePID_Init(&pid_friction,10,1.5 , 2, 0);
	BasePID_Init(&pid_friction1,10,1.5 , 2, 0);
	

	BasePID_Init(&pid_yaw_angle, 2,0.02 ,-100,  3);//-4 -0.01,300   0
	BasePID_Init(&pid_yaw_speed, 2000, 0, 0,  100);//1
	
	BasePID_Init(&pid_yaw1_angle, 5,0,0,  2);//7
	BasePID_Init(&pid_yaw1_speed, 1500, 10, 0,  2);//6
	
	BasePID_Init(&pid_pitch_angle, 6, 0.04, 0, 2.5);//2
	BasePID_Init(&pid_pitch_speed,1300,0, 0,0);//3
	
	BasePID_Init(&run_pid,10,0,0,0);



  HolderInit(&Holder,&pid_pitch,&pid_yaw,&pid_yaw1,CAN1);
  AllChassisInit(&allchassis,&run_pid,&pid_follow,&pid_base);
	AmmoBoosterInit(&AmmoBooster,&pid_friction,&pid_friction1,&pid_load);
	
	
	UARTx_Init(&huart2, Brain_Autoaim_callback);
	
	UARTx_Init(&huart5, Brain_Lidar_callback);

	
	UARTx_Init(&huart3, Referee_callback);

	
	UARTx_Init(&huart1, DR16_callback);

	
	UARTx_Init(&huart4, Brain_Camera_callback);  

	UARTx_Init(&huart7, Vofa_callback);

	
//	
	
	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CAN_Open(&can1);
	
	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can2);
	

	
	INS_Init(&bmi088.bmi088_Data); // 
  MPU6050_Init(&mpu6050.mpu6050_Data);
	



		TIMx_Init(&htim14, TIM14_Task);
	TIM_Open(&tim14);

	TIMx_Init(&htim13, TIM13_Task);
	TIM_Open(&tim13);
}

uint8_t Vofa_callback(uint8_t * recBuffer, uint16_t len)
{
	//if ((recBuffer[23]==0x3E && recBuffer[1]==0x54 ) ||recBuffer[4]==0x3C) FLAG_1=1;
	//if (recBuffer[2]==0x53 && recBuffer[7]==0x4F)  FLAG_Send=1;
	//HAL_UART_Transmit_DMA(&huart8, recBuffer, len);
	return 0;
}

