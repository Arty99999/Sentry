#ifndef SWERVECHASSIS_H
#define SWERVECHASSIS_H

#include "stm32h7xx.h"
#include "devices.h"
#include "pid.h"
#include "holder.h"
	
#define vx_Sence 0.04f
#define vy_Sence 0.06f

/**
  * @brief  ���ֵ��̽ṹ��
  */
typedef struct 
{
	struct
	{
		uint8_t Enable;    						//ʹ��״̬			
		uint8_t isRefereeUpdating; 				//����ϵͳ�Ƿ��ڸ�������
		uint8_t DriveMode;						//����ģʽ
		uint8_t ChassisState;					//����״̬
		uint8_t SinRollingFlag;					//�Ƿ��������
	}Flag;

	struct 
	{
		Motor motor[4];							//���̵���ṹ��
		BasePID_Object RunPID[4];				//�ٶȿ���PID����
		BasePID_Object FollowPID;			//���̸���PID����
	}Motors3508;

	struct 
	{
		Motor motor[4];							//���̵���ṹ��
		BasePID_Object TurnPID[4];				//ת��Ƕȿ��ƽṹ��
	}Motors6020;

	struct 
	{
		int32_t Vx;					//ǰ���˶����ٶ�
		int32_t Vy;		  			//�����˶����ٶ�
		int32_t Omega;				//��ת�Ľ��ٶ�
		int32_t RollingOmega;		//�����Ľ��ٶȣ�ֻģ����ʾ��С����λ����
		int32_t	DeltaVx;
		int32_t	DeltaVy;
		int16_t	DeltaOmega;
		uint8_t MoveFlag;
		struct
		{
			float Vx;
			float Vy;
			float Omega;
		}Sensitivity;
		
		float ModuleOfSpeed;		//�ٶ�������ģֵ
		float AngleOfSpeed;			//�ٶ������ĽǶ�
		float K_OmeToSpeed;
	}Movement;
		float brain_vx;
		float brain_vy;
	struct 
	{
		float Rc_Ctrl_ModuleOfSpeed;
		float Rc_Ctrl_AngleOfSpeed;		
		int32_t Vx[4];
		int32_t Vy[4];
		float Angle[4];	
		float BestAngle[4];
		int16_t TargetEcd[4];
		float FeedbackAngle[4];
		float SpeedNo[4];
		uint8_t SpeedChangeFlag[4];
	}Vectors;
	
	struct
	{
		BasePID_Object PowerPID_Vmax;
		uint8_t SuperCapIsOnline;
		float SpeedPowerLimit;
		float SpeedPowerLimit_initial;
		uint8_t PowerLimitChangeFlag;
		uint32_t PowerLimitLast;
		float K_PowerPlus;
		float K_OnlyRolling;//ֻ����ʱ����Ŀ���ٶȵı��������Ŷ��ƺ���׼
		float K_MovingRolling;//����ƽ��ʱ�ı���
		float K_Turning;//ת��ʱ�ı���
		float Power_Plus;//Ŀ�깦������
	}Power;
	
}SwerveChassis;


/**
  * @brief  �޸Ķ���PID�����Ľṹ��
  */
typedef struct 
{	
	float SpeedKp,SpeedKi,SpeedKd;
	float TurnKp,TurnKi,TurnKd;
}PIDParameters;

extern int randtime;


void ChangeChassisPID(SwerveChassis* chassis, PIDParameters pid);
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, BasePID_Object power_pid,BasePID_Object follow_pid,BasePID_Object vmax_pid,BasePID_Object omegamax_pid, CanNumber canx);
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Chassis_Attitude_Info* Swerve, float canAngle);
void SwerveChassisMotionControl(SwerveChassis* chassis , RC_Ctrl* rc_ctrl ,Holder_t* holder);
void VectorSolve(float vx, float vy, float* vectorAngle, float* vectorModule, uint8_t id);
float EcdtoAngle(int16_t offset, int16_t ecd);
float FindBestTargetAngle(float targetAngle, float feedbackAngle, uint8_t* flag);
void SwerveChassisPowerControl(SwerveChassis* chassis,Chassis_Attitude_Info* Swerve,RC_Ctrl* rc_ctrl);
float FindBestTargetAngle_special(float targetAngle, float feedbackAngle, uint8_t* flag,uint8_t MotorNum);
float FittedSpeed(float targetpower);

#endif
