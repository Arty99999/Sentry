#ifndef LOAD_H
#define LOAD_H
#include "stm32h7xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "devices.h"
#include "vision.h"

typedef struct  
{
	int16_t Target_Firing_Rate;   		//������ת��
	int32_t  Shoot_sum_cut;				//Ħ���ֻ���������
	uint8_t SuperHeatMode;				//������ģʽ��־λ 1Ϊ���볬����ģʽ
	uint16_t Angle_inversion;			
	volatile long int Shoot_Target_Sum;      //Ŀ�귢�䵯������  //Ӧ����ĵ���
	uint8_t Crowd_back_Flag;                 //��ת��־λ	Crowd_back_Flag=0������Crowd_back_Flag=1����
	uint8_t Stop_Back_Flag;									//�����жϱ�־λ 0Ϊ������
//	uint8_t BulletCap_Status_Flag;           //�������ոǱ�־λ��1Ϊ��
		struct 
	{
		Motor motor[4];						//< ���̵���ṹ��
		BasePID_Object RunPID[4];			//< �ٶȿ���PID����
	}Motors3508;
			struct 
	{
		Motor motor[1];						//< ���̵���ṹ��
		BasePID_Object RunPID[4];			//< �ٶȿ���PID����
	}Motors2006;
	
}Shoot_Info;

typedef struct            //Ħ���ֲ�������Ϣ
{
	float Target_Speed[3];       //0��1�ֱ�������Ħ���ֵ�ת�٣�2�ǲ����̵�ת��
	float Angle_Plate;			//����Ѿ�ת����ת��
	float Angle_Target;     //ʵ����û���������Ӧ��ת�ĽǶȣ������Ѿ�ת���ĽǶȺͼ���ת���ĽǶ�	
	float Oneshoot_Angle;     //����ڵ�������µ���Ҫת��ת��
	float ThreeAngle;				//�������������µ���Ҫת��ת��
	float Angle;						//��������������Ӧ��ת��ת�ǣ�������ȥ�Ѿ�ת����ת��
	uint16_t  Inversion_Cnt;  //�����̿�������
	float Current_1[4];				//���շ��͸����������
	float delta_angle;				//�ٶȻ��ֵ�����ÿʱÿ��ת���ĽǶ�
}Friction_Load_Info;

//����С����ģʽ�ṹ�壬��Ҳ��֪��Ϊɶ����
typedef struct
{
	uint8_t Roll_Flag;
	uint8_t Roll_Flag1;
	uint8_t Rock_Flag;
	uint8_t Remake_Flag;
	uint8_t Chassis_State;
	float Chassis_Power;
	float Chassis_Power_Buff;
	float Chassis_Out_Sum;
	float Chassis_Allowed_Out;
}Chassis_Attitude_Info;

struct Pid_Info
{	
	float Kp_Pid;//����ϵ��
	float Ki_Pid;//����ϵ��
	float Kd_Pid;//΢��ϵ��  
	float E0_Pid;//��ֵ
	float Last_E0_Pid;
	float Last_Last_E0_Pid;
	float Ep_Part_Pid;//�������ֿ�����
	float Ei_Part_Pid;//���ֲ��ֿ�����
	float Ed_Part_Pid;//΢�ֲ��ֿ�����
	float Out_Pid;//������ֵ
	float Target_Pid; //Ŀ��ֵ
	float Feedback_Pid;//����ֵ
};
extern Shoot_Info Shoot;
extern Friction_Load_Info Booster;
extern Friction_Load_Info Motor_Booster;
extern uint16_t Friction_Start_Cnt;
extern struct Pid_Info Pid_Load;
extern struct Pid_Info Pid_Friction;

void LoadInit(Shoot_Info* shoot, BasePID_Object friction_pid, BasePID_Object load_pid, CanNumber canx);
//void Friction_Load_Fire_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,Trace* Info_Vision);
void Shoot_Fire_Mode_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot);
float Friction_Load_Pid_Control(BasePID_Object* base_pid,float Feedback,int16_t target,uint8_t who);

#endif
