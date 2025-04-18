#include "shoot.h"
#include "stm32h7xx_hal.h"
#include "motor.h"
#include "fdcan.h"
#include "driver_timer.h"
#include "dr16.h"
#include "pid.h"
#include "heat_control.h"
#include "brain.h"
#include "referee.h"
#include "brain.h"
#include "Holder.h"
#include "user_lib.h"


int16_t kadan = 0;
Ammo_Booster AmmoBooster;
void AmmoBoosterInit(Ammo_Booster *ammo_booster,BasePID_Object friction_pid, BasePID_Object friction_pid1,BasePID_Object load_pid)
{
	MotorInit(&ammo_booster->Friction_Wheel.motor3508[0], 0 , Motor3508, CAN1, 0x201);
	MotorInit(&ammo_booster->Friction_Wheel.motor3508[1], 0 , Motor3508, CAN1, 0x202);
	MotorInit(&ammo_booster->Shoot_Plate.motor2006, 0 , Motor2006, CAN1, 0x203);

	
	BasePID_Init(&ammo_booster->Shoot_Plate.RunPID, load_pid.Kp, load_pid.Ki, load_pid.Kd, load_pid.KiPartDetachment);
	BasePID_Init(&ammo_booster->Friction_Wheel.Friction_PID[0], friction_pid.Kp, friction_pid.Ki, friction_pid.Kd, friction_pid.KiPartDetachment);
	BasePID_Init(&ammo_booster->Friction_Wheel.Friction_PID[1], friction_pid1.Kp, friction_pid1.Ki, friction_pid1.Kd, friction_pid1.KiPartDetachment);

	
	ammo_booster->Shoot_Plate.Fire_Rate = 8000;//5250
	ammo_booster->Shoot_Plate.Fire_Margin = 40;
	ammo_booster->Shoot_Plate.Angle_Sense = 0.1689f;

	

	ammo_booster->Friction_Wheel.Friction_Start = 0;
	ammo_booster->Friction_Wheel.Friction_Speed[0] = -5700;
	ammo_booster->Friction_Wheel.Friction_Speed[1] =-5690;
//		ammo_booster->Friction_Wheel.Friction_Speed[0] = -100;
//	ammo_booster->Friction_Wheel.Friction_Speed[1] =- 100;
}

void ShootPlateControl(Ammo_Booster *ammo_booster,Brain_t* brain)
{
  ammo_booster->Shoot_Plate.heat_status = (referee2022.power_heat_data.shooter_id1_17mm_cooling_heat >= 
               referee2022.game_robot_status.shooter_id1_17mm_cooling_limit - 
               ammo_booster->Shoot_Plate.Fire_Margin) ? 0 : 1;
	/*
	 *通过2006转速积分得出拨弹盘当前角度
	 */
	ammo_booster->Shoot_Plate.Delta_Angle = ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM * 0.001 * ammo_booster->Shoot_Plate.Angle_Sense;
	if(ammo_booster->Shoot_Plate.Delta_Angle > 0.005)
	  ammo_booster->Shoot_Plate.Plate_Angle += ammo_booster->Shoot_Plate.Delta_Angle;
	

	if(rc_Ctrl.isOnline == 1&&ammo_booster->Shoot_Plate.heat_status==1)   ///
	{

    if(referee2022.power_heat_data.shooter_id1_17mm_cooling_heat >= referee2022.game_robot_status.shooter_id1_17mm_cooling_limit - ammo_booster->Shoot_Plate.Fire_Margin-70)
			ammo_booster->Shoot_Plate.Fire_Divider=125;else ammo_booster->Shoot_Plate.Fire_Divider=50;	
     	
		if (ammo_booster->Shoot_Plate.Shoot_rest_flag) ammo_booster->Shoot_Plate.Shoot_Cut++;
 		if (ammo_booster->Shoot_Plate.Shoot_Cut==ammo_booster->Shoot_Plate.Fire_Divider) ammo_booster->Shoot_Plate.Shoot_rest_flag=0;
		
				if((rc_Ctrl.rc.s1==1||referee2022.game_status.game_progress==4)&&rc_Ctrl.rc.s2 ==2&&brain->Autoaim.fire_flag==1&&ammo_booster->Shoot_Plate.Shoot_rest_flag==0 &&brain->Autoaim.IsFire==1)
				{
					ammo_booster->Shoot_Plate.Target_Angle  += 45;
					ammo_booster->Shoot_Plate.ShootNum++;
					ammo_booster->Shoot_Plate.Shoot_rest_flag=1;
					ammo_booster->Shoot_Plate.Shoot_Cut=0;
				}
				else if (rc_Ctrl.rc.s1 ==1&&rc_Ctrl.rc.s2 !=2&&ammo_booster->Shoot_Plate.Shoot_rest_flag==0)
				{
				ammo_booster->Shoot_Plate.Target_Angle  += 45;
					ammo_booster->Shoot_Plate.ShootNum++;
					ammo_booster->Shoot_Plate.Shoot_rest_flag=1;
					ammo_booster->Shoot_Plate.Shoot_Cut=0;
				}
			//	brain->Autoaim.fire_flag=0;

	}
	
	/*
	 *计算拨弹盘目标角度
	 */
	//if(rc_Ctrl.isOnline == 0) 	ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Plate_Angle;	
	
	/*
	 *拨弹盘控制（角度判断、速度控制）
	 */
	if(ammo_booster->Shoot_Plate.Target_Angle - ammo_booster->Shoot_Plate.Plate_Angle > 5)
	  ammo_booster->Shoot_Plate.Plate_Out = Pid_Control(&ammo_booster->Shoot_Plate.RunPID, ammo_booster->Shoot_Plate.Fire_Rate, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
	else ammo_booster->Shoot_Plate.Plate_Out = Pid_Control(&ammo_booster->Shoot_Plate.RunPID, 0, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
	
	if(ammo_booster->Shoot_Plate.Target_Angle - ammo_booster->Shoot_Plate.Plate_Angle > 5 && ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM < 1000)
	{
		kadan++;
		if(kadan >= 200)
		{
			ammo_booster->Shoot_Plate.Target_Angle = ammo_booster->Shoot_Plate.Plate_Angle+6;
			kadan++;
			ammo_booster->Shoot_Plate.Plate_Out = Pid_Control(&ammo_booster->Shoot_Plate.RunPID, -2000, ammo_booster->Shoot_Plate.motor2006.Data.SpeedRPM);
			if(kadan > 1200) kadan = 0;
				
		}	
	}
	else kadan=0;
	
	MotorFillData(&ammo_booster->Shoot_Plate.motor2006, ammo_booster->Shoot_Plate.Plate_Out);
}	


void FrictionWheelControl(Ammo_Booster *ammo_booster)
{ 
	if(tim14.ClockTime % 2 == 0)
	{if (rc_Ctrl.isOnline == 1) ammo_booster->Friction_Wheel.Friction_Start++;else ammo_booster->Friction_Wheel.Friction_Start--;}
	

	ammo_booster->Friction_Wheel.Friction_Start=int16_constrain(ammo_booster->Friction_Wheel.Friction_Start,0,1000);

	ammo_booster->Friction_Wheel.Friction_Target_Speed[0] = ammo_booster->Friction_Wheel.Friction_Speed[0] * ammo_booster->Friction_Wheel.Friction_Start * 0.001;
	ammo_booster->Friction_Wheel.Friction_Target_Speed[1] = ammo_booster->Friction_Wheel.Friction_Speed[1] * ammo_booster->Friction_Wheel.Friction_Start * 0.001*(-1);
	for (int i=0;i<2;i++)	
	{
  ammo_booster->Friction_Wheel.Friction_Out[i] = BasePID_SpeedControl((BasePID_Object*)(ammo_booster->Friction_Wheel.Friction_PID + i), ammo_booster->Friction_Wheel.Friction_Target_Speed[i], ammo_booster->Friction_Wheel.motor3508[i].Data.SpeedRPM);
	MotorFillData(&ammo_booster->Friction_Wheel.motor3508[i], ammo_booster->Friction_Wheel.Friction_Out[i]);
  }
}

float Pid_Control(BasePID_Object* base_pid, float target, int16_t Feedback)
{   
	base_pid->Error = target - Feedback;
	
  base_pid->KpPart = base_pid->Error * base_pid->Kp;	
	
	base_pid->KiPart += base_pid->Error* base_pid->Ki;
	
	if(base_pid->KiPart>1200) base_pid->KiPart=1000;
	if(base_pid->KiPart<-1200) base_pid->KiPart=-1000;
		
	base_pid->Out=base_pid->KpPart+base_pid->KiPart; 
	
	if(base_pid->Out>10000) base_pid->Out=10000;//电流值给得太大
	if(base_pid->Out<-10000) base_pid->Out=-10000;
	return base_pid->Out;
}