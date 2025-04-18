#include "load.h"
#include "init.h"
#include "dr16.h"
#include "fdcan.h"
#include "driver_timer.h"
#include "heat_control.h"
#include "referee.h"
#include "hardware_config.h"


uint16_t Friction_Start_Cnt;	//摩擦轮启动计数
uint16_t Friction_Stop_Cnt;		//用于在遥控器断电后对摩擦轮进行缓停
uint16_t Firing_Rate =5300;		//拨弹盘转速  5300
uint16_t Friction_Speed_Left  =7175;//7475    //摩擦轮转速  =
uint16_t Friction_Speed_Right =7165;//7530  
uint16_t heat_t=45;
float shoot_sense =0.171;  //0.2245;//0.1708f;//0.220517f;  //0.255f;//0.2245
float Finish_Angle=30.0f;
uint16_t ShootWaitTime = 0;
uint16_t Shoot_DelayTime = 800;


 /**
  * @brief 发射机构初始化，拨弹盘的两个电机和摩擦轮的电机一共三个电机。
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
  * @brief 拨弹电机速度(傻瓜控制法)
  */
void Load_Motor_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,RC_Ctrl* rc_ctrl)
{
		
	
/*------------------------判断打弹是否完成、是否卡弹----------------------------------*/
	if ((booster->Angle_Target-booster->Angle_Plate) < Finish_Angle )
    {
		booster->Target_Speed[2] = 0;	
		}
	else
		{
			booster->Target_Speed[2]=shoot->Target_Firing_Rate;
			if(shoot->Motors2006.motor[0].Data.SpeedRPM<50) //堵转，卡弹了
				{
					booster->Inversion_Cnt++;
					
//					if(booster->Inversion_Cnt==100)
//						shoot->Crowd_back_Flag=1;
					
//					if(booster->Inversion_Cnt>100)
//					{
//						booster->Target_Speed[2]=-3000;  //使2006电机快速反转，
//						if(booster->Inversion_Cnt>400)
//						{
//							//清空计数，重新正向旋转
//							booster->Inversion_Cnt=0;
//							booster->Target_Speed[2]= shoot->Target_Firing_Rate;
//							shoot->Shoot_Target_Sum=booster->Angle_Plate/45;
//						}
//						if(shoot->Crowd_back_Flag==1)		//卡弹
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
		/*------------------------判断打弹是否完成的具体过程----------------------------------*/
		booster->Oneshoot_Angle =rc_ctrl->OneShoot  *45;
		booster->ThreeAngle = rc_ctrl->ThreeShoot * 45;

		if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)
		{
			booster->Angle =shoot->Shoot_Target_Sum *45;
		}

		if(((shoot->Crowd_back_Flag==0)&&(shoot->Stop_Back_Flag==0))&&(rc_ctrl->rc.s1!=2))	//没堵转，且热量足
		booster->Angle_Target = booster->Oneshoot_Angle + booster->Angle+booster->ThreeAngle;
		else if((shoot->Stop_Back_Flag==1)||((rc_ctrl->rc.s1==3)&&(rc_ctrl->mouse.press_l_flag==0)))
		{
			booster->Angle_Target = booster->Angle_Plate;
			shoot->Stop_Back_Flag=0;
		}
}	
	
/**
  * @brief  摩擦轮控制
  *         拨弹的闭环 好好调 减少发弹延时
  */

void Friction_Load_Fire_Control(Shoot_Info* shoot ,Friction_Load_Info* booster,Trace* Info_Vision)
{
	 uint8_t i;
							/*射击模式*/
	
   Heat_Control();//先冷却
	 
	//Q键 单发 ctrl + Q 三连发 
	if(tim14.HolderTime > 500&&rc_Ctrl.isOnline == 1) 
	{
		Friction_Start_Cnt++;	
		Friction_Stop_Cnt=0;
		/*射速处理*/ //始终以当前最大射速 除缓启动外
        if(Friction_Start_Cnt>1000)		//1s缓启动
	    {
	       	Friction_Start_Cnt=1000;
	    }
			//摩擦轮1s缓启动
			booster->Target_Speed[0]=Friction_Speed_Left*Friction_Start_Cnt/1000;
		  booster->Target_Speed[1]=-Friction_Speed_Right*Friction_Start_Cnt/1000;	
			
		//正常情况下转换为角度Load_Info.speed_raw * 2π，但是该电机为1/36减速电机，所以需要/36，
		//并根据误差调整该值，比如：实测转一圈是348，因此此处需要*(360/348)，此处具体电机具体测试调整
			booster->delta_angle =shoot->Motors2006.motor[0].Data.SpeedRPM  * 0.001f*shoot_sense ;//0.16890283083816092679906684856341 * 0.80f
		if(abs(booster->delta_angle)<0.01f) booster->delta_angle=0;//防止跑飞
		/*加滤波函数对角度*/
		booster->Angle_Plate += booster->delta_angle;//拨弹盘一共转过的角度
		
		
	  Load_Motor_Control(&Shoot,&Booster,&rc_Ctrl);
			

		/*摩擦轮缓启动*/
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
			for(i=0;i<3;i++)//关摩擦轮 拨弹盘
			{
  	  	booster->Current_1[i]=0;
			}
	}
		 MotorFillData(&shoot->Motors3508.motor[0], booster->Current_1[0]);
		 MotorFillData(&shoot->Motors3508.motor[1], booster->Current_1[1]);
		 MotorFillData(&shoot->Motors2006.motor[0], booster->Current_1[2]);
}	
	
	/**
  * @brief 发射控制
  */
void Shoot_Fire_Mode_Control(RC_Ctrl* rc_ctrl,Shoot_Info* shoot)
{
	
	//判断是否进入超热量模式，并判断当前枪口热量
		if(shoot->SuperHeatMode ==0){
			if(Muzzle.on_time_heat>=referee2022.game_robot_status.shooter_id1_17mm_cooling_limit-heat_t)
			{
				shoot->Target_Firing_Rate=0;
				Muzzle.heat_status=0; //热量不足
//				shoot->Target_Firing_Rate=Firing_Rate;
//				Muzzle.heat_status=1;  //无裁判系统时测试用
			}
			else
			{
				shoot->Target_Firing_Rate=Firing_Rate;
				Muzzle.heat_status=1; //热量充足
			}
		}	
		else if(shoot->SuperHeatMode ==1){
			shoot->Target_Firing_Rate=Firing_Rate;
		}
	
	if(Vision_Info.Hit_Mode==2)
	{
		if(Brain.BrainCore[1].CoreInstruction.IsFire)
		{
			if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)//快速单发||Vision_Info.Flag_Auto_Enable==1		
			{	
				if(rc_Ctrl.isOnline == 1)  //连射超过限值时转换模式为单发
				{ 
					shoot->Shoot_sum_cut ++;  //控制频率1000帧
					if((Muzzle.heat_status==1)&&(shoot->Shoot_sum_cut%50==0)&&(shoot->Crowd_back_Flag==0))  //降频，一秒20发
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
		if(rc_ctrl->rc.s1==1||rc_ctrl->mouse.press_l_flag==1)//快速单发||Vision_Info.Flag_Auto_Enable==1		
		{
			if(rc_Ctrl.isOnline == 1)   //连射超过限值时转换模式为单发
			{ 
				shoot->Shoot_sum_cut ++;  //控制频率1000帧
				if((Muzzle.heat_status==1)&&(shoot->Shoot_sum_cut%50==0)&&(shoot->Crowd_back_Flag==0))  //降频，一秒20发
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
	
	if(base_pid->Out>10000) base_pid->Out=10000;//电流值给得太大
	if(base_pid->Out<-10000) base_pid->Out=-10000;
	return base_pid->Out;
  }
	if(who<2)
	{
	base_pid->Error = target - Feedback;
		
	base_pid->KpPart = base_pid->Error * base_pid->Kp;
 
	if(base_pid->Error<50||base_pid->Error>-50)//积分作用 
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
