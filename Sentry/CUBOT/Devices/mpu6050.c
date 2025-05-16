#include "mpu6050.h"

#include "user_lib.h"
#include "kalman_filter.h"



MPU6050_t mpu6050={.mpu6050_Data.i2cHandler=&hi2c2};
float MPU6050_ACCEL_SEN = MPU6050_ACCEL_4G_SEN;
float MPU6050_GYRO_SEN  = MPU6050_GYRO_1000_SEN;
int flag_1=1;
int16_t mpu6050_cali_count = 0;

// д���ݵ�MPU6050�Ĵ���
static uint8_t MPU6050_WriteByte(uint8_t reg_add, uint8_t reg_dat,I2C_HandleTypeDef *hi2c)
{
    return Sensors_I2C_WriteRegister(MPU6050_ADDRESS, reg_add, 1, &reg_dat,hi2c);
}

// ��MPU6050�Ĵ�����ȡ����
static uint8_t MPU6050_ReadData(uint8_t reg_add, uint8_t *Read, uint8_t num,I2C_HandleTypeDef *hi2c)
{

    return Sensors_I2C_ReadRegister(MPU6050_ADDRESS, reg_add, num, Read,hi2c);
}
static uint8_t MPU6050_ReadData_1(uint8_t reg_add, uint8_t *Read, uint8_t num,I2C_HandleTypeDef *hi2c)
{
    return Sensors_I2C_ReadRegister_1(MPU6050_ADDRESS, reg_add, num, Read,hi2c);
}
static void MPU6050_Calibrate_Offset(IMU_InitData_t *mpu6050_data);
  uint8_t TAddr = 0;
uint8_t MPU6050_Init(IMU_InitData_t *mpu6050_data)
{

    MPU6050_WriteByte(0x6B, 0x01,mpu6050_data->i2cHandler); // ����
    DWT_Delay_s(0.5);
    MPU6050_WriteByte(0x6B, 0x00,mpu6050_data->i2cHandler); // �������״̬0x00
    DWT_Delay_s(0.05);
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &TAddr, 1,mpu6050_data->i2cHandler);
    if (TAddr != 0x68)
        return 1;
    MPU6050_WriteByte(0x6B, 0x00,mpu6050_data->i2cHandler); // �������״̬0x00
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x19, 0x00,mpu6050_data->i2cHandler); // ����Ƶ�ʣ�1KHz��
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1A, 0x03,mpu6050_data->i2cHandler); // ��ͨ�˲�
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1B, 0x10,mpu6050_data->i2cHandler); // ���������̣����Ĵ���0x1B��ֵΪ0x10ʱ������������Ϊ
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1C, 0x09,mpu6050_data->i2cHandler); // ���ٶ����̣����Ĵ���0x1C��ֵΪ0x09ʱ�����ٶ�����Ϊ
    DWT_Delay_s(0.05);

	MPU6050_Calibrate_Offset(mpu6050_data);
	
    return 0;
}


uint8_t MPU6050_Init_1(IMU_InitData_t *mpu6050_data)
{
    uint8_t TAddr = 0;
    MPU6050_WriteByte(0x6B, 0x01,mpu6050_data->i2cHandler); // ����
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x6B, 0x00,mpu6050_data->i2cHandler); // �������״̬0x00
    DWT_Delay_s(0.05);
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &TAddr, 1,mpu6050_data->i2cHandler);
    if (TAddr != 0x68)
        return 1;
    MPU6050_WriteByte(0x6B, 0x00,mpu6050_data->i2cHandler); // �������״̬0x00
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x19, 0x00,mpu6050_data->i2cHandler); // ����Ƶ�ʣ�1KHz��
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1A, 0x03,mpu6050_data->i2cHandler); // ��ͨ�˲�
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1B, 0x10,mpu6050_data->i2cHandler); // ���������̣����Ĵ���0x1B��ֵΪ0x10ʱ������������Ϊ
    DWT_Delay_s(0.05);
    MPU6050_WriteByte(0x1C, 0x09,mpu6050_data->i2cHandler); // ���ٶ����̣����Ĵ���0x1C��ֵΪ0x09ʱ�����ٶ�����Ϊ
    DWT_Delay_s(0.05);


	
    return 0;
}


// ��ֹ��Ư
void MPU6050_Calibrate_Offset(IMU_InitData_t *mpu6050_data)
{
     float startTime;
     uint16_t CaliTimes = 1000;
     float gyroDiff[3], gNormDiff;

    uint8_t accBuf[6]  = {0};
    uint8_t gyroBuf[6] = {0};
    int16_t mpu6050_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = DWT_GetTimeline_s();
    do {
        if (DWT_GetTimeline_s() - startTime > 12) {
            // ����У׼��ʱ��ʹ��Ԥ��ֵУ׼
            mpu6050_data->gyro_offset[0] = MPU6050_GxOFFSET;
            mpu6050_data->gyro_offset[1] = MPU6050_GyOFFSET;
            mpu6050_data->gyro_offset[2] = MPU6050_GzOFFSET;
            mpu6050_data->g_norm         = MPU6050_gNORM;
            break;
        }

        DWT_Delay_s(0.005);
        mpu6050_data->g_norm         = 0;
        mpu6050_data->gyro_offset[0] = 0;
        mpu6050_data->gyro_offset[1] = 0;
        mpu6050_data->gyro_offset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i) {
            MPU6050_ReadData(MPU6050_ACC_OUT, accBuf, 6,mpu6050_data->i2cHandler);
            mpu6050_raw_temp       = (int16_t)(accBuf[0] << 8) | accBuf[1];
            mpu6050_data->accel[0] = mpu6050_raw_temp * MPU6050_ACCEL_SEN;
            mpu6050_raw_temp       = (int16_t)(accBuf[2] << 8) | accBuf[3];
            mpu6050_data->accel[1] = mpu6050_raw_temp * MPU6050_ACCEL_SEN;
            mpu6050_raw_temp       = (int16_t)(accBuf[4] << 8) | accBuf[5];
            mpu6050_data->accel[2] = mpu6050_raw_temp * MPU6050_ACCEL_SEN;

            gNormTemp = sqrtf(mpu6050_data->accel[0] * mpu6050_data->accel[0] +
                              mpu6050_data->accel[1] * mpu6050_data->accel[1] +
                              mpu6050_data->accel[2] * mpu6050_data->accel[2]);
            mpu6050_data->g_norm += gNormTemp;

            MPU6050_ReadData(MPU6050_GYRO_OUT, gyroBuf, 6,mpu6050_data->i2cHandler);
            mpu6050_raw_temp      = (int16_t)(gyroBuf[0] << 8) | gyroBuf[1];
            mpu6050_data->gyro[0] = mpu6050_raw_temp * MPU6050_GYRO_SEN;
            mpu6050_data->gyro_offset[0] += mpu6050_data->gyro[0];
            mpu6050_raw_temp      = (int16_t)(gyroBuf[2] << 8) | gyroBuf[3];
            mpu6050_data->gyro[1] = mpu6050_raw_temp * MPU6050_GYRO_SEN;
            mpu6050_data->gyro_offset[1] += mpu6050_data->gyro[1];
            mpu6050_raw_temp      = (int16_t)(gyroBuf[4] << 8) | gyroBuf[5];
            mpu6050_data->gyro[2] = mpu6050_raw_temp * MPU6050_GYRO_SEN;
            mpu6050_data->gyro_offset[2] += mpu6050_data->gyro[2];

//            if (i == 0) {
//                gNormMax = gNormTemp;
//                gNormMin = gNormTemp;
//                for (uint8_t j = 0; j < 3; ++j) {
//                    gyroMax[j] = mpu6050_data->gyro[j];
//                    gyroMin[j] = mpu6050_data->gyro[j];
//                }
//            } else {
//                if (gNormTemp > gNormMax)
//                    gNormMax = gNormTemp;
//                if (gNormTemp < gNormMin)
//                    gNormMin = gNormTemp;
//                for (uint8_t j = 0; j < 3; ++j) {
//                    if (mpu6050_data->gyro[j] > gyroMax[j])
//                        gyroMax[j] = mpu6050_data->gyro[j];
//                    if (mpu6050_data->gyro[j] < gyroMin[j])
//                        gyroMin[j] = mpu6050_data->gyro[j];
//                }
//            }

//            gNormDiff = gNormMax - gNormMin;
//            for (uint8_t j = 0; j < 3; ++j)
//                gyroDiff[j] = gyroMax[j] - gyroMin[j];
//            if (gNormDiff > 0.5f ||
//                gyroDiff[0] > 0.15f ||
//                gyroDiff[1] > 0.15f ||
//                gyroDiff[2] > 0.15f)
//                //            {
//                break;
//            //            }

            DWT_Delay_s(0.0005);
        }

        mpu6050_data->g_norm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            mpu6050_data->gyro_offset[i] /= (float)CaliTimes;

        mpu6050_cali_count++;

    } while (
             fabsf(mpu6050_data->g_norm - 9.8f) > 6.0f ||
             fabsf(mpu6050_data->gyro_offset[0]) > 1.0f ||
       
             fabsf(mpu6050_data->gyro_offset[2]) > 0.08f);
    // ��������while��˵��У׼�������ӣ���ʱ��ʹ��Ԥ��ֵУ׼
    mpu6050_data->accel_scale = 9.81f / mpu6050_data->g_norm;
}

/**
 * @brief   ��ȡ����������
 */

void MPU6050_Read(IMU_InitData_t *mpu6050_data)
{

    static uint8_t accBuf[6],a[2];
    static uint8_t gyroBuf[6],gyroBuf_[2];
    static int16_t mpu6050_raw_temp;
	MPU6050_ReadData(0x41, a, 2,mpu6050_data->i2cHandler);
	mpu6050_data->temperature=(int16_t)(a[0]<<8|a[1])/340.0+36.53;
//mpu6050_data->gyro_offset[2]=(mpu6050_data->temperature*1.938-75.644)* MPU6050_GYRO_SEN;
    MPU6050_ReadData(MPU6050_GYRO_OUT+4, gyroBuf_, 2,mpu6050_data->i2cHandler);
 mpu6050_data->gyro[0]=0;
mpu6050_data->gyro[1]=0;
    mpu6050_raw_temp      = (int16_t)(gyroBuf_[0] << 8) | gyroBuf_[1];
	  mpu6050_data->Raw_gyro[2]=mpu6050_raw_temp ;
    mpu6050_data->gyro[2] = mpu6050_raw_temp * MPU6050_GYRO_SEN - mpu6050_data->gyro_offset[2];
		if (fabs(mpu6050_data->gyro[2]/MPU6050_GYRO_SEN)<30) mpu6050_data->gyro[2]=0;

}

float q0_1 = 1.0f, q1_1 = 0.0f, q2_1 = 0.0f, q3_1 = 0.0f;
void IMUupdate_1(IMU_InitData_t *mpu6050_data)
{
	MPU6050_t* mpu6050  =(MPU6050_t*)mpu6050_data ;
    static int8_t YawRoundCount;
   float  halfT,norm,vx,vy,vz,ex,ey,ez,tempq0,tempq1,tempq2,tempq3,now,dt;
		static uint32_t DWT_Count = 0;
	static float YawAngleLast;
	  dt =DWT_GetDeltaT(&DWT_Count); 
	//mpu6050_data->gyro[2]*=1.58;
    tempq0 = q0_1 + ( - q3_1*mpu6050_data->gyro[2])*dt*0.5;
    tempq1 = q1_1 + ( q2_1*mpu6050_data->gyro[2] )*dt*0.5;
    tempq2 = q2_1 + ( - q1_1*mpu6050_data->gyro[2] )*dt*0.5;
    tempq3 = q3_1 + (q0_1*mpu6050_data->gyro[2] )*dt*0.5;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0_1= tempq0 * norm;
    q1_1 = tempq1 * norm;
    q2_1 = tempq2 * norm;
    q3_1= tempq3 * norm;

    

		mpu6050->Yaw=-atan2(2 * q1_1 * q2_1+ 2 * q0_1* q3_1, -2 * q2_1*q2_1- 2 * q3_1 * q3_1 + 1)*RtA; // yaw        -pi----pi
    mpu6050->Roll= -asin(-2 * q1_1 * q3_1+ 2 * q0_1 * q2_1)*RtA; // pitch    -pi/2    --- pi/2 
//		//>pitch��Ƕ�
    mpu6050->Pitch= -atan2(2 * q2_1* q3_1 + 2 * q0_1* q1_1, -2 * q1_1 * q1_1 - 2 * q2_1* q2_1+ 1)* RtA; 
		    if (mpu6050->Yaw -YawAngleLast > 180.0f)
    {
        YawRoundCount--;
    }
    else if (mpu6050->Yaw -YawAngleLast < -180.0f)
    {
        YawRoundCount++;
    }
		mpu6050->Yaw_total_angle = 360.0f * YawRoundCount + mpu6050->Yaw;
		YawAngleLast = mpu6050->Yaw;
		// roll       -pi-----pi        -pi-----pi 
}	


void MPU6050_Read_1(IMU_InitData_t *mpu6050_data)
{
	static uint8_t Buf[14];
	static int16_t mpu6050_raw_temp;
	MPU6050_t* mpu6050  =(MPU6050_t*)mpu6050_data ;
	MPU6050_ReadData_1(MPU6050_ACC_OUT, Buf, 14,mpu6050->mpu6050_Data.i2cHandler);
	mpu6050_raw_temp       = (int16_t)(Buf[0] << 8) | Buf[1];
	mpu6050_data->accel[0] = mpu6050_raw_temp * MPU6050_ACCEL_SEN * mpu6050_data->accel_scale;
	mpu6050_raw_temp       = (int16_t)(Buf[2] << 8) | Buf[3];
	mpu6050_data->accel[1] = mpu6050_raw_temp * MPU6050_ACCEL_SEN * mpu6050_data->accel_scale;
	mpu6050_raw_temp       = (int16_t)(Buf[4] << 8) | Buf[5];
	mpu6050_data->accel[2] = mpu6050_raw_temp * MPU6050_ACCEL_SEN * mpu6050_data->accel_scale;

	mpu6050_data->temperature=(int16_t)(Buf[6]<<8|Buf[7])/340.0+36.53;

	mpu6050_raw_temp      = (int16_t)(Buf[8] << 8) | Buf[9];
mpu6050_data->Raw_gyro[0]=mpu6050_raw_temp ;
	mpu6050_data->gyro[0] = mpu6050_raw_temp * MPU6050_GYRO_SEN - mpu6050_data->gyro_offset[0];

	mpu6050_raw_temp      = (int16_t)(Buf[10] << 8) | Buf[11];
mpu6050_data->Raw_gyro[1]=mpu6050_raw_temp ;
	mpu6050_data->gyro[1] = mpu6050_raw_temp * MPU6050_GYRO_SEN - mpu6050_data->gyro_offset[1];
	mpu6050_raw_temp      = (int16_t)(Buf[12] << 8) | Buf[13];
mpu6050_data->Raw_gyro[2]=mpu6050_raw_temp ;

	mpu6050_data->gyro[2] = mpu6050_raw_temp * MPU6050_GYRO_SEN - mpu6050_data->gyro_offset[2];
//    static uint8_t Buf[14];
//    int16_t mpu6050_raw_temp;
//	  MPU6050_t* mpu6050  =(MPU6050_t*)mpu6050_data ;
//    MPU6050_ReadData(MPU6050_ACC_OUT, Buf, 16,mpu6050_data->i2cHandler);

//	mpu6050_data->temperature=(int16_t)(Buf[6]<<8|Buf[7])/340.0+36.53;		
//	for (int i=0;i<3;i++)
//	  {
//			mpu6050_data->Raw_accel[i]= (int16_t)(Buf[2*i] << 8) | Buf[2*i+1];
//      mpu6050_data->accel[i] = mpu6050_data->Raw_accel[i] * MPU6050_ACCEL_SEN * mpu6050_data->accel_scale;
//		  mpu6050_data->Raw_gyro[i]= (int16_t)(Buf[2*i+8] << 8) | Buf[2*i+9];
//      mpu6050_data->gyro[i] = mpu6050_data->Raw_gyro[i] * MPU6050_GYRO_SEN- mpu6050_data->gyro_offset[i];
//    }
	if (fabs(mpu6050_data->gyro[2]/MPU6050_GYRO_SEN)<30.0) mpu6050_data->gyro[2]=0;
	 
	
}
float q0_0 = 1.0f, q1_0 = 0.0f, q2_0 = 0.0f, q3_0 = 0.0f;
float exInt_1 = 0, eyInt_1 = 0, ezInt_1 = 0;    // scaled integral error
void IMUupdate(IMU_InitData_t *mpu6050_data)
{
	MPU6050_t* mpu6050  =(MPU6050_t*)mpu6050_data ;
    static int8_t YawRoundCount;
   float  halfT,norm,vx,vy,vz,ex,ey,ez,tempq0,tempq1,tempq2,tempq3,now,dt;
		static uint32_t DWT_Count_1 = 0;
	static float YawAngleLast,a,b,c;
	  dt =DWT_GetDeltaT(&DWT_Count_1); 
	
	    vx = 2.0f*(q1_0*q3_0 - q0_0*q2_0);
    vy = 2.0f*(q0_0*q1_0 + q2_0*q3_0);
    vz = q0_0*q0_0 - q1_0*q1_0 - q2_0*q2_0 + q3_0*q3_0;
	    ex = (mpu6050_data->accel[1] * vz - mpu6050_data->accel[2] * vy);// + (my*wz - mz*wy);
    ey = (mpu6050_data->accel[2] * vx - mpu6050_data->accel[0] * vz);// + (mz*wx - mx*wz);
    ez = (mpu6050_data->accel[0] * vy - mpu6050_data->accel[1] * vx);// + (mx*wy - my*wx);
	 a=mpu6050_data->gyro[0];
	b=mpu6050_data->gyro[1];
	c=mpu6050_data->gyro[2];
	    if(fabs(mpu6050_data->Raw_gyro[1])<100&& fabs(mpu6050_data->Raw_gyro[2])<100&&fabs(mpu6050_data->Raw_gyro[0])<100)
    {
			exInt_1 = exInt_1 + ex * 0.1 *dt*0.5;
			eyInt_1 = eyInt_1 + ey * 0.1  *dt*0.5;	
			ezInt_1 = ezInt_1 + ez * 0.1  *dt*0.5;
			// �ò���������PI����������ƫ
			mpu6050_data->gyro[0]+=0.5*ex ;
			mpu6050_data->gyro[1]+=0.5*ey ;
			mpu6050_data->gyro[2]+=0.5*ez ;
		}
    tempq0 = q0_0 + (-q1_0*mpu6050_data->gyro[0] - q2_0*mpu6050_data->gyro[1] - q3_0*mpu6050_data->gyro[2])*dt*0.5;
    tempq1 = q1_0 + (q0_0*mpu6050_data->gyro[0] + q2_0*mpu6050_data->gyro[2] - q3_0*mpu6050_data->gyro[1])*dt*0.5;
    tempq2 = q2_0 + (q0_0*mpu6050_data->gyro[1] - q1_0*mpu6050_data->gyro[2] + q3_0*mpu6050_data->gyro[0])*dt*0.5;
    tempq3 = q3_0 + (q0_0*mpu6050_data->gyro[2] + q1_0*mpu6050_data->gyro[1] - q2_0*mpu6050_data->gyro[0])*dt*0.5;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0_0= tempq0 * norm;
    q1_0 = tempq1 * norm;
    q2_0 = tempq2 * norm;
    q3_0= tempq3 * norm;




//		mpu6050->Yaw=-atan2(2 * q1_1 * q2_1+ 2 * q0_1* q3_1, -2 * q2_1*q2_1- 2 * q3_1 * q3_1 + 1)*RtA; // yaw        -pi----pi
    mpu6050->Roll= -asin(-2 * q1_0 * q3_0+ 2 * q0_0 * q2_0)*RtA; // pitch    -pi/2    --- pi/2 
//		//>pitch��Ƕ�
    mpu6050->Pitch= -atan2(2 * q2_0* q3_0 + 2 * q0_0* q1_0, -2 * q1_0 * q1_0 - 2 * q2_0* q2_0+ 1)* RtA; 
			 mpu6050_data->gyro[0]=a;
	mpu6050_data->gyro[1]=b;
	mpu6050_data->gyro[2]=c;
//		    if (mpu6050->Yaw -YawAngleLast > 180.0f)
//    {
//        YawRoundCount--;
//    }
//    else if (mpu6050->Yaw -YawAngleLast < -180.0f)
//    {
//        YawRoundCount++;
//    }
//		mpu6050->Yaw_total_angle = 360.0f * YawRoundCount + mpu6050->Yaw;
//		YawAngleLast = mpu6050->Yaw;
		// roll       -pi-----pi        -pi-----pi 
}	