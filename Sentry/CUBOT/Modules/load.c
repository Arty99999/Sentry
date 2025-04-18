#include "load.h"
#include "init.h"
#include "dr16.h"
#include "fdcan.h"
#include "driver_timer.h"
#include "heat_control.h"
#include "referee.h"
#include "hardware_config.h"


uint16_t Friction_Start_Cnt;	//Ħ������������
uint16_t Friction_Stop_Cnt;		//������ң�����ϵ���Ħ���ֽ��л�ͣ
uint16_t Firing_Rate =5300;		//������ת��  5300
uint16_t Friction_Speed_Left  =7175;//7475    //Ħ����ת��  =
uint16_t Friction_Speed_Right =7165;//7530  
uint16_t heat_t=45;
float shoot_sense =0.171;  //0.2245;//0.1708f;//0.220517f;  //0.255f;//0.2245
float Finish_Angle=30.0f;
uint16_t ShootWaitTime = 0;
uint16_t Shoot_DelayTime = 800;


 /**
  * @brief ���������ʼ���������̵����������Ħ���ֵĵ��һ�����������
  */
void LoadInit(Shoot_Info* shoot, BasePID_Object friction_pid, BasePID_Object load_pid, CanNumber canx)        
{	
	MotorInit(&shoot->Motors3508.motor[0], 0 , Motor3508, canx, 0x201);
	MotorInit(&shoot->Motors3508.motor[1], 0 , Motor3508, canx, 0x202);
	MotorInit(&shoot->Motors2006.motor[0], 0 , Motor2006, canx, 0x203);
	BasePID_Init(&shoot->Motors3508.RunPID[0],friction_pid.Kp,friction_pid.Ki,friction_pid.Kd,friction_pid.KiPartDetachment);
	BasePID_Init(&shoot->Motors3508.RunPID[1],friction_pid.Kp,friction_pid.Ki,friction_pid.Kd,friction_pid.KiPartDetachment);
	BasePID_Init(&shoot->Motors2006.RunPID[0],load_pid.Kp,load_pid.Ki,load_pid.Kd,load_pid.KiPartDetachment);
	BasePID_Init(&shoot->Motors2006.RunPID[1],650,0,4.6,0);
	shoot->Target_Firing_Rate = 6000;  //8100
	shoot->Shoot_sum_cut = 0 ;
}	

	
	/**
  * @brief ��������ٶ�(ɵ�Ͽ��Ʒ�)
  */
void Load_Motor_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,RC_Ctrl* rc_ctrl)
{
		
	
/*------------------------�жϴ��Ƿ���ɡ��Ƿ񿨵�----------------------------------*/
	if ((booster->Angle_Target-booster->Angle_Plate) < Finish_Angle )
    {
		booster->Target_Speed[2] = 0;	
		}
	else
		{
			booster->Target_Speed[2]=shoot->Target_Firing_Rate;
			if(shoot->Motors2006.motor[0].Data.SpeedRPM<50) //��ת��������
				{
					booster->Inversion_Cnt++;
					
//					if(booster->Inversion_Cnt==100)
//						shoot->Crowd_back_Flag=1;
					
//					if(booster->Inversion_Cnt>100)
//					{
//						booster->Target_Speed[2]=-3000;  //ʹ2006������ٷ�ת��
//						if(booster->Inversion_Cnt>400)
//						{
//							//��ռ���������������ת
//							booster->Inversion_Cnt=0;
//							booster->Target_Speed[2]= shoot->Target_Firing_Rate;
//							shoot->Shoot_Target_Sum=booster->Angle_Plate/45;
//						}
//						if(shoot->Crowd_back_Flag==1)		//����
//						{
//							shoot->Angle_inversion++;
//							if(shoot->Angle_inversion>180)
//							{
//								shoot->Crowd_back_Flag=0;
//								shoot->Angle_inversion=0;
//								shoot->Shoot_Target_Sum=booster->Angle_Plate/45-rc_ctrl->OneShoot;
//							}
//						}
//					}
				}
			}
		/*------------------------�жϴ��Ƿ���ɵľ������----------------------------------*/
		booster->Oneshoot_Angle =rc_ctrl->OneShoot  *45;
		booster->ThreeAngle = rc_ctrl->ThreeShoot * 45;

		if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)
		{
			booster->Angle =shoot->Shoot_Target_Sum *45;
		}

		if(((shoot->Crowd_back_Flag==0)&&(shoot->Stop_Back_Flag==0))&&(rc_ctrl->rc.s1!=2))	//û��ת����������
		booster->Angle_Target = booster->Oneshoot_Angle + booster->Angle+booster->ThreeAngle;
		else if((shoot->Stop_Back_Flag==1)||((rc_ctrl->rc.s1==3)&&(rc_ctrl->mouse.press_l_flag==0)))
		{
			booster->Angle_Target = booster->Angle_Plate;
			shoot->Stop_Back_Flag=0;
		}
}	
	
/**
  * @brief  Ħ���ֿ���
  *         �����ıջ� �úõ� ���ٷ�����ʱ
  */

void Friction_Load_Fire_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,Trace* Info_Vision)
{
	 uint8_t i;
							/*���ģʽ*/
	
   Heat_Control();//����ȴ
	 
	//Q�� ���� ctrl + Q ������ 
	if(tim14.HolderTime > 500&&rc_Ctrl.isOnline == 1) 
	{
		Friction_Start_Cnt++;	
		Friction_Stop_Cnt=0;
		/*���ٴ���*/ //ʼ���Ե�ǰ������� ����������
        if(Friction_Start_Cnt>1000)		//1s������
	    {
	       	Friction_Start_Cnt=1000;
	    }
			//Ħ����1s������
			booster->Target_Speed[0]=Friction_Speed_Left*Friction_Start_Cnt/1000;
		  booster->Target_Speed[1]=-Friction_Speed_Right*Friction_Start_Cnt/1000;	
			
		//���������ת��Ϊ�Ƕ�Load_Info.speed_raw * 2�У����Ǹõ��Ϊ1/36���ٵ����������Ҫ/36��
		//��������������ֵ�����磺ʵ��תһȦ��348����˴˴���Ҫ*(360/348)���˴�������������Ե���
			booster->delta_angle =shoot->Motors2006.motor[0].Data.SpeedRPM  * 0.001f*shoot_sense ;//0.16890283083816092679906684856341 * 0.80f
		if(abs(booster->delta_angle)<0.01f) booster->delta_angle=0;//��ֹ�ܷ�
		/*���˲������ԽǶ�*/
		booster->Angle_Plate += booster->delta_angle;//������һ��ת���ĽǶ�
		
		
	  Load_Motor_Control(&Shoot,&Booster,&rc_Ctrl);
			

		/*Ħ���ֻ�����*/
		booster->Current_1[0]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+0),shoot->Motors3508.motor[0].Data.SpeedRPM,booster->Target_Speed[0],0);
		booster->Current_1[1]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+1),shoot->Motors3508.motor[1].Data.SpeedRPM,booster->Target_Speed[1],1);
		booster->Current_1[2]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors2006.RunPID+0),shoot->Motors2006.motor[0].Data.SpeedRPM,booster->Target_Speed[2],2);	
		if( booster->Current_1[2] >9000)  booster->Current_1[2] =9000;
		if( booster->Current_1[2] <-9000)  booster->Current_1[2] =-9000;
	}
else if(rc_Ctrl.isOnline == 0)
	{
	  	Friction_Start_Cnt=0;
	
		if((Shoot.Motors3508.motor[0].Data.SpeedRPM!=0)||(Shoot.Motors3508.motor[1].Data.SpeedRPM!=0))
		{
			Friction_Stop_Cnt += Friction_Speed_Left*0.0003;
			
			if(Friction_Stop_Cnt >= Friction_Speed_Left)
				Friction_Stop_Cnt = Friction_Speed_Left;
			
			booster->Target_Speed[0]=Friction_Speed_Left-Friction_Stop_Cnt;
			booster->Target_Speed[1]=booster->Target_Speed[0];
			booster->Current_1[0]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+0),shoot->Motors3508.motor[0].Data.SpeedRPM,booster->Target_Speed[0],0);
			booster->Current_1[1]=Friction_Load_Pid_Control((BasePID_Object*)(shoot->Motors3508.RunPID+1),shoot->Motors3508.motor[1].Data.SpeedRPM,booster->Target_Speed[1],1);
		}
		else
			for(i=0;i<3;i++)//��Ħ���� ������
			{
  	  	booster->Current_1[i]=0;
			}
	}
		 MotorFillData(&shoot->Motors3508.motor[0], booster->Current_1[0]);
		 MotorFillData(&shoot->Motors3508.motor[1], booster->Current_1[1]);
		 MotorFillData(&shoot->Motors2006.motor[0], booster->Current_1[2]);
}	
	
	/**
  * @brief �������
  */
void Shoot_Fire_Mode_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot)
{
	
	//�ж��Ƿ���볬����ģʽ�����жϵ�ǰǹ������
		if(shoot->SuperHeatMode ==0){
			if(Muzzle.on_time_heat>=referee2022.game_robot_status.shooter_id1_17mm_cooling_limit-heat_t)
			{
				shoot->Target_Firing_Rate=0;
				Muzzle.heat_status=0; //��������
//				shoot->Target_Firing_Rate=Firing_Rate;
//				Muzzle.heat_status=1;  //�޲���ϵͳʱ������
			}
			else
			{
				shoot->Target_Firing_Rate=Firing_Rate;
				Muzzle.heat_status=1; //��������
			}
		}	
		else if(shoot->SuperHeatMode ==1){
			shoot->Target_Firing_Rate=Firing_Rate;
		}
	
	if(Vision_Info.Hit_Mode==2)
	{
		if(Brain.BrainCore[1].CoreInstruction.IsFire)
		{
			if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)//���ٵ���||Vision_Info.Flag_Auto_Enable==1		
			{	
				if(rc_Ctrl.isOnline == 1)  //���䳬����ֵʱת��ģʽΪ����
				{ 
					shoot->Shoot_sum_cut ++;  //����Ƶ��1000֡
					if((Muzzle.heat_status==1)&&(shoot->Shoot_sum_cut%50==0)&&(shoot->Crowd_back_Flag==0))  //��Ƶ��һ��20��
						shoot->Shoot_Target_Sum++;
					else if(Muzzle.heat_status==0)
						shoot->Stop_Back_Flag=1;
				}
			} 
		}
	}
	else{
		if((Vision_Info.Hit_Mode == 0)||(Vision_Info.Fire_Flag_True == 1))
		{
		if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)//���ٵ���||Vision_Info.Flag_Auto_Enable==1		
		{
			if(rc_Ctrl.isOnline == 1)   //���䳬����ֵʱת��ģʽΪ����
			{ 
				shoot->Shoot_sum_cut ++;  //����Ƶ��1000֡
				if((Muzzle.heat_status==1)&&(shoot->Shoot_sum_cut%50==0)&&(shoot->Crowd_back_Flag==0))  //��Ƶ��һ��20��
					shoot->Shoot_Target_Sum++;
				else if(Muzzle.heat_status==0)
					shoot->Stop_Back_Flag=1;
			}
		}
	  }	
	}
  
	 if(dafu_shoot_flag==1)
	 { 
		 ShootWaitTime++;
		 if(ShootWaitTime>=Shoot_DelayTime)
			{
		  	rc_ctrl->OneShoot +=1;
			  ShootWaitTime = 0;
			  dafu_shoot_flag = 0;
			  dafu_shoot_cut = 0;
			}
		}
	}
	

/**
  * @brief 
  */
float Friction_Load_Pid_Control(BasePID_Object* base_pid,float Feedback,int16_t target,uint8_t who)
{
	
	static  float  E_Sum[3];

	if(who==2)
	{
	base_pid->Error = target - Feedback;//Wheel_v_h_speed[who];
	
  base_pid->KpPart = base_pid->Error * base_pid->Kp;

	E_Sum[who]+=base_pid->Error*base_pid->Ki;
	
	if(target==0) E_Sum[who]=0;	
	
	base_pid->KiPart=E_Sum[who];
	
	if(base_pid->KiPart>1200) base_pid->KiPart=1000;
	if(base_pid->KiPart<-1200) base_pid->KiPart=-1000;
		
	base_pid->Out=base_pid->KpPart+base_pid->KiPart; 
	
	if(base_pid->Out>10000) base_pid->Out=10000;//����ֵ����̫��
	if(base_pid->Out<-10000) base_pid->Out=-10000;
	return base_pid->Out;
  }
	if(who<2)
	{
	base_pid->Error = target - Feedback;
		
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
 
	if(base_pid->Error<50||base_pid->Error>-50)//�������� 
	E_Sum[who]+=base_pid->Error*base_pid->Ki;
	
	if(target==0) E_Sum[who]=0;	
	base_pid->KiPart=E_Sum[who];
	
	if(base_pid->KiPart>1000) base_pid->KiPart=1000;
	if(base_pid->KiPart<-1000) base_pid->KiPart=-1000;
	
	base_pid->Out=base_pid->KpPart+base_pid->KiPart;
	
	if(base_pid->Out>30000)base_pid->Out=30000;
	if(base_pid->Out<-30000)base_pid->Out=-30000;
  return  base_pid->Out;		
  }
	return 0;
}	
