/**@file Gyro.c
* @brief    �弶֧�ְ������ڹ����������ļ����û��ص��ض���
* @details  ��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
* @author      RyanJiao  any question please send mail to 1095981200@qq.com

* @date        2021-8-23
* @version     V1.1
* @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
**********************************************************************************
* @attention
* Ӳ��ƽ̨: STM32H750VBT \n
* SDK�汾��-++++
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021-8-12  <td>1.0      <td>RyanJiao  <td>������ʼ�汾
* </table>
*
**********************************************************************************
*/

/**********************************************************************************
 ==============================================================================
						  How to use this driver
 ==============================================================================

	��� MPU6050.h

	1. ����MPU6050_Init()
  2. �� Gyro_Get_Data() ����1ms���ж�����
  

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
  *@brife  usart5���ܴ���������
	*/
uint8_t HWT_recData[HWT_rxBufferLengh];

	UART_RxBuffer uart4_buffer={
		.Data = HWT_recData,
		.DataSize = HWT_rxBufferLengh
	};

gyro_data_t  gyro_data=
{
	.sens_yaw  =0.52f,  	// yaw   ���ٶȻ���Ϊ�Ƕȵ�ֵ  ����ģ��ʹ��
	.sens_pitch=0.6783f,  	//0.6783 pitch ���ٶȻ���Ϊ�Ƕȵ�ֵ  δ��
  .cqhlp = 10
};

gyro_data_t  gyro_data2=
{
	.sens_yaw  =0.3143f,		//0.3580,  // yaw   ���ٶȻ���Ϊ�Ƕȵ�ֵ   δ��
	.sens_pitch=0.2812f,  	//0.3051758 pitch ���ٶȻ���Ϊ�Ƕȵ�ֵ   ����ģ��ʹ��
  .cqhlp = 10
};

void MPU_Init(void)
{
	//MUP structure variable define
	MPU_Region_InitTypeDef MPU_Config;
	
	/*-----------Open FPU--------*///High speed FLOAT calculate
	SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
	/*-----------Open Cache------------*/
	SCB_EnableICache();//ʹ��I-Cache
  SCB_EnableDCache();//ʹ��D-Cache   
	SCB->CACR|=1<<2;   //ǿ��D-Cache͸д,�粻����,ʵ��ʹ���п���������������	
	/*-----------Open MPU------------*/
	HAL_MPU_Disable();
	
	MPU_Config.Enable=MPU_REGION_ENABLE;
	MPU_Config.Number=MPU_REGION_NUMBER1;//��������� 1
	MPU_Config.BaseAddress= 0x24000000;//����������ַ
	MPU_Config.Size=MPU_REGION_SIZE_512KB;//���ñ�����512k
	MPU_Config.SubRegionDisable=0x00;//��ֹ������
	MPU_Config.TypeExtField=MPU_TEX_LEVEL0;//����������չ��Ϊlevel0
	MPU_Config.AccessPermission=MPU_REGION_FULL_ACCESS;//ȫ���ʣ���Ȩ&�û����ɷ��ʣ�
	MPU_Config.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;//����ָ�����
	MPU_Config.IsShareable=MPU_ACCESS_SHAREABLE;//������
	MPU_Config.IsCacheable=MPU_ACCESS_CACHEABLE;//����cache
	MPU_Config.IsBufferable=MPU_ACCESS_NOT_BUFFERABLE;//�������� DMAģʽ��Ҫ��Ϊ������
	HAL_MPU_ConfigRegion(&MPU_Config);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}	


//void Gyro_Init(void)
//{
//	//>�ϵ�
//		MPU6050_PowerOn();
//	//>��ʼ��
//	  MPU6050_Init();
//		MPU6050_Init2();
//	//>����Ϊ 0
//	  gyro_data.cnt=0;
//		gyro_data2.cnt=0;
//	IMU_Boot(&sensor,&mpuAngle,1);
//	IMU_Boot(&sensor2,&mpuAngle2,2);
//}


//void Gyro_Reset(void)
//{
//	//>�µ�
//	  MPU6050_PowerOff();
//	  HAL_Delay(50);
//	
//	//>�ϵ��ʼ��
//    Gyro_Init();

//}

/*
 *�������� 
 *yaw��̬���ٶ�   
 *pitch��̬���ٶ� 
 */

//>����1ms�Ķ�ʱ���ж���
//float yaw_data_filter_mpu[5]={0};
//float pitch_data_filter_mpu[5]={0};
//int8_t yaw_filter_mpu_cnt=0;
//int8_t pitch_filter_mpu_cnt=0;
//void Gyro_Get_Data(void){
//	//>���ݸ��´����ۼ�
//  gyro_data.cnt++;
//	
//  //>MPU6050���ݸ���
//	MPU6050_RawDataUpdate(&mpuAngle,1);

//if(gyro_data.cnt>1000){	
//		//>��Ԫ������� pitch�Ƕ�
//	
//	//>�ǶȻ�Ϊ����  ���ٶȽ��п������˲�  ����һ���޸ģ�
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
//	//>���ݸ��´����ۼ�
//  gyro_data2.cnt++;
//	
//  //>MPU6050���ݸ���
//	MPU6050_RawDataUpdate(&mpuAngle2,2);

//if(gyro_data2.cnt>1000){	
//	
//	//>�ǶȻ�Ϊ����  ���ٶȽ��п������˲�  ����һ���޸ģ�
//	PrepareForIMU(&sensor2,&mpuAngle2);
//	//>��Ԫ������� pitch�Ƕ�
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
//	Gyro_Get_Data();  //����������
////	Gyro_Get_Data2();   //����IIC������ģ��
//	holder->Yaw.MPU6050_Angle=-gyro_data.yaw_angle;
//	holder->Yaw.MPU6050_Angle_speed=gyro_data.yaw_speed;
//  holder->Yaw.MPU6050_Angle_speed1=holder->Yaw.MPU6050_Angle_speed*150;
////holder->Yaw.MPU6050_Angle_speed1=LPFilter(holder->Yaw.MPU6050_Angle_speed1 ,&LPF_yaw_mpu);  //һ�׵�ͨ�˲�
//	holder->Pitch.MPU6050_Angle=gyro_data2.pitch_angle;
//	holder->Pitch.MPU6050_Angle_speed=gyro_data2.pitch_speed;
//	holder->Pitch.MPU6050_Angle_speed1=holder->Pitch.MPU6050_Angle_speed*150;
////holder->Pitch.MPU6050_Angle_speed1=LPFilter(holder->Pitch.MPU6050_Angle_speed1 ,&LPF_pitch_mpu);   //һ�׵�ͨ�˲�
//  holder->Yaw1.MPU6050_Angle=gyro_data2.yaw_angle;
//	holder->Yaw1.MPU6050_Angle_speed=gyro_data2.yaw_speed;
//  holder->Yaw1.MPU6050_Angle_speed1=holder->Yaw1.MPU6050_Angle_speed*150;
//	
//}

///**
//  




