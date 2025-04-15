/**@file Gyro.c
* @brief    板级支持包，串口管理器配置文件，用户回调重定义
* @details  主要包括构建串口管理器，提供串口初始化和用户回调重定义
* @author      RyanJiao  any question please send mail to 1095981200@qq.com

* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>创建初始版本
* </table>
*
**********************************************************************************
*/

/**********************************************************************************
 ==============================================================================
						  How to use this driver
 ==============================================================================

	添加 MPU6050.h

	1. 调用MPU6050_Init()
  2. 把 Gyro_Get_Data() 放在1ms的中断里面
  

  ********************************************************************************/


#include "Gyro.h"
#include "mpu6050.h"
#include "driver_timer.h"
#include "kalman.h"
#include "holder.h"
#include "hardware_config.h"
#include "filter.h"
#include "check.h"
float k;
MpuDebug mpudebug =
{
	.mpuDebugTime=0,
	.DebugBeginFlag=0,
	.DebugFinishFlag=0 
};

/**
  *@brife  usart5接受串口陀螺仪
	*/
uint8_t HWT_recData[HWT_rxBufferLengh];

	UART_RxBuffer uart4_buffer={
		.Data = HWT_recData,
		.DataSize = HWT_rxBufferLengh
	};

gyro_data_t  gyro_data=
{
	.sens_yaw  =0.52f,  	// yaw   角速度积分为角度的值  内置模块使用
	.sens_pitch=0.6783f,  	//0.6783 pitch 角速度积分为角度的值  未用
  .cqhlp = 10
};

gyro_data_t  gyro_data2=
{
	.sens_yaw  =0.3143f,		//0.3580,  // yaw   角速度积分为角度的值   未用
	.sens_pitch=0.2812f,  	//0.3051758 pitch 角速度积分为角度的值   外置模块使用
  .cqhlp = 10
};

void MPU_Init(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();//使能I-Cache
  SCB_EnableDCache();//使能D-Cache   
	SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题	
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;//保护区编号 1
	MPU_Config.BaseAddress= 0x24000000;//保护区基地址
	MPU_Config.Size=MPU_REGION_SIZE_512KB;//设置保护区512k
	MPU_Config.SubRegionDisable=0x00;//禁止子区域
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;//设置类型扩展域为level0
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;//全访问（特权&用户都可访问）
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;//允许指令访问
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;//允许共享
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;//允许cache
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;//不允许缓冲 DMA模式下要设为不允许
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}	


//void Gyro_Init(void)
//{
//	//>上电
//		MPU6050_PowerOn();
//	//>初始化
//	  MPU6050_Init();
//		MPU6050_Init2();
//	//>计数为 0
//	  gyro_data.cnt=0;
//		gyro_data2.cnt=0;
//	IMU_Boot(&sensor,&mpuAngle,1);
//	IMU_Boot(&sensor2,&mpuAngle2,2);
//}


//void Gyro_Reset(void)
//{
//	//>下电
//	  MPU6050_PowerOff();
//	  HAL_Delay(50);
//	
//	//>上电初始化
//    Gyro_Init();

//}

/*
 *测速数据 
 *yaw静态角速度   
 *pitch静态角速度 
 */

//>放在1ms的定时器中断中
//float yaw_data_filter_mpu[5]={0};
//float pitch_data_filter_mpu[5]={0};
//int8_t yaw_filter_mpu_cnt=0;
//int8_t pitch_filter_mpu_cnt=0;
//void Gyro_Get_Data(void){
//	//>数据更新次数累加
//  gyro_data.cnt++;
//	
//  //>MPU6050数据更新
//	MPU6050_RawDataUpdate(&mpuAngle,1);

//if(gyro_data.cnt>1000){	
//		//>四元数解算出 pitch角度
//	
//	//>角度化为弧度  加速度进行卡尔曼滤波  （有一处修改）
//	PrepareForIMU(&sensor,&mpuAngle);
//	IMUupdate_1(&sensor,&mpuAngle);
//	
//	gyro_data.gyro_w_yaw   = mpuAngle.gyroRaw.z-(int16_t)sensor.gyro.quiet.z;
//	gyro_data.gyro_w_pitch = mpuAngle.gyroRaw.x-(int16_t)sensor.gyro.quiet.x;

//	if(abs(gyro_data.gyro_w_yaw)>10){
//			gyro_data.yaw_speed=((gyro_data.gyro_w_yaw)*0.0001*gyro_data.sens_yaw);
//			gyro_data.yaw_angle+= gyro_data.yaw_speed;}
//	
////	if(abs(gyro_data.gyro_w_pitch)>100){		
////			gyro_data.pitch_speed = ((gyro_data.gyro_w_pitch)*0.0001f*gyro_data.sens_pitch);
////	    gyro_data.pitch_angle= gyro_data.pitch_angle + gyro_data.pitch_speed;}
//		}
//}

//void Gyro_Get_Data2(void){
//	//>数据更新次数累加
//  gyro_data2.cnt++;
//	
//  //>MPU6050数据更新
//	MPU6050_RawDataUpdate(&mpuAngle2,2);

//if(gyro_data2.cnt>1000){	
//	
//	//>角度化为弧度  加速度进行卡尔曼滤波  （有一处修改）
//	PrepareForIMU(&sensor2,&mpuAngle2);
//	//>四元数解算出 pitch角度
//	IMUupdate_2(&sensor2,&mpuAngle2);
//	
//	gyro_data2.gyro_w_yaw   = mpuAngle2.gyroRaw.z-(int16_t)sensor2.gyro.quiet.z;
//	gyro_data2.gyro_w_pitch = mpuAngle2.gyroRaw.x-(int16_t)sensor2.gyro.quiet.x;

//	if(abs(gyro_data2.gyro_w_yaw)>10){
//			gyro_data2.yaw_speed=((gyro_data2.gyro_w_yaw)*0.0001*gyro_data2.sens_yaw);
//			gyro_data2.yaw_angle= gyro_data2.yaw_angle + gyro_data2.yaw_speed;}
//		
//	if(abs(gyro_data2.gyro_w_pitch)>10){		
//			gyro_data2.pitch_speed = ((-1.0 * gyro_data2.gyro_w_pitch)*0.0001f*gyro_data2.sens_pitch);
//	    gyro_data2.pitch_angle= gyro_data2.pitch_angle + gyro_data2.pitch_speed;}
////	    gyro_data2.pitch_angle=mpuAngle2.pitch;
////				if(abs(gyro_data2.gyro_w_pitch)>10)
////			gyro_data2.pitch_speed = ((-1.0 * gyro_data2.gyro_w_pitch)*0.0001f*gyro_data2.sens_pitch);
////			k=gyro_data2.gyro_w_pitch/32;
//		}
//}

//void MPU_Get_Data(Holder_t* holder)
//{
//	Gyro_Get_Data();  //板载陀螺仪
////	Gyro_Get_Data2();   //外置IIC陀螺仪模块
//	holder->Yaw.MPU6050_Angle=-gyro_data.yaw_angle;
//	holder->Yaw.MPU6050_Angle_speed=gyro_data.yaw_speed;
//  holder->Yaw.MPU6050_Angle_speed1=holder->Yaw.MPU6050_Angle_speed*150;
////holder->Yaw.MPU6050_Angle_speed1=LPFilter(holder->Yaw.MPU6050_Angle_speed1 ,&LPF_yaw_mpu);  //一阶低通滤波
//	holder->Pitch.MPU6050_Angle=gyro_data2.pitch_angle;
//	holder->Pitch.MPU6050_Angle_speed=gyro_data2.pitch_speed;
//	holder->Pitch.MPU6050_Angle_speed1=holder->Pitch.MPU6050_Angle_speed*150;
////holder->Pitch.MPU6050_Angle_speed1=LPFilter(holder->Pitch.MPU6050_Angle_speed1 ,&LPF_pitch_mpu);   //一阶低通滤波
//  holder->Yaw1.MPU6050_Angle=gyro_data2.yaw_angle;
//	holder->Yaw1.MPU6050_Angle_speed=gyro_data2.yaw_speed;
//  holder->Yaw1.MPU6050_Angle_speed1=holder->Yaw1.MPU6050_Angle_speed*150;
//	
//}

///**
//  




