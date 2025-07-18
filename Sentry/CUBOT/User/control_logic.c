#include "ins.h"
#include "mpu6050.h"
#include "bmi088.h"
#include "ET08.h"
#include "holder.h"
#include "shoot.h"
#include "all_chassis.h"
#include "brain.h"
#include "referee.h"
#include "check.h"
#include "control_logic.h"
#include "hardware_config.h"
int n1;
int  flag_ppp,cntppp;
extern int flag00,flag01;
extern int FLAG_1,FLAG_Send;
uint8_t flag_change_last;
uint8_t referee_cnt,referee_Fps;
extern Shoot_state shoot;
int16_t ThisSecond =0;
	int cnt_heat,flag_fallover,cnt_fallover;
//extern	float init_quaternion[4];
	extern float angle;
extern float a222;
extern int flag000;
extern int change_position,cnt_shoot;
int cnt000;
extern int m00;
int cnt_change,flag_change,cnt_vxvy,flag_vxvy;

int cntll,flag_roll,cnt_refree;
int cnth;
void TIM14_Task(void)
{
	
		tim14.ClockTime++;
	
  if (flag000==1) cnth++;
	if (cnth>=1000) {cnth=1;flag000=0;}
	if(tim14.ClockTime%5000==0) 
	{referee_Fps=referee_cnt;
		referee_cnt=0;
	}
	
	if (Brain.Lidar.mode==3) cnt_change++;else cnt_change=0;
	if (cnt_change>7500)  {flag_change=1;cnt_change=0;}
//	if(flag_change==1&&change_position==0 )  {flag_change_last=change_position;change_position=2;flag_change=0;}
//	else if(flag_change==1&&change_position==2&&flag_change_last==0 )  {flag_change_last=change_position;change_position=1;flag_change=0;}
//	else if(flag_change==1&& change_position==2&&flag_change_last==1)  {flag_change_last=change_position;change_position=0;flag_change=0;}
//	else if(flag_change==1&&change_position==1)  {flag_change_last=change_position;change_position=2;flag_change=0;}
	if(flag_change==1&&change_position==0 )  {change_position=1;flag_change=0;}
	else if(flag_change==1&&change_position==1)  {change_position=2;flag_change=0;}
	else if(flag_change==1&&change_position==2 )  {change_position=3;flag_change=0;}
	else if(flag_change==1&&change_position==3)  {change_position=0;flag_change=0;}

//	if ((referee2022.game_status.stage_remain_time<=360||((referee2022.game_robot_status.robot_id>10 && referee2022.game_robot_hp.red_outpost_HP==0) ||(referee2022.game_robot_status.robot_id<10 && referee2022.game_robot_hp.blue_outpost_HP==0)))&&referee2022.game_status.game_progress==4&&change_position!=2&&change_position!=3) change_position=2;
	//if (referee2022.game_status.game_progress!=4)change_position=0;
	

	
		if(tim14.ClockTime%1000==0) FPS_Check(&tim14_FPS);
	if (Brain.Lidar.mode!=4) {flag000=0;a222=0;cnth=0;}
	if(tim14.ClockTime%2000==0)
	{
		for (int i=0;i<8;i++)
		{
			if (referee2022.game_robot_hp.blue_robot_HP[i]<referee2022.game_robot_hp.blue_robot_HP_last[i])referee2022.game_robot_hp.blue_robot_Hurt[i]=1;else referee2022.game_robot_hp.blue_robot_Hurt[i]=0;
		if (referee2022.game_robot_hp.red_robot_HP[i]<referee2022.game_robot_hp.red_robot_HP_last[i])referee2022.game_robot_hp.red_robot_Hurt[i]=1;else referee2022.game_robot_hp.red_robot_Hurt[i]=0;
		}
	
	}
 // Brain.Autoaim.Mode=2;
		
	if (rc_Ctrl_et.rc.s2==1) cnt_vxvy++;else cnt_vxvy=0;
		if (cnt_vxvy>=7000) {flag_vxvy=!flag_vxvy;cnt_vxvy=0;}
	
				if (Brain.Autoaim.mode==Change&&fabs(Holder.Yaw1.Can_Angle-Holder.Yaw1.Target_Angle)<1) Brain.Autoaim.mode_cnt[Change]+=100;else if (Brain.Autoaim.mode==Change)    Brain.Autoaim.mode_cnt[Change]++;  else  Brain.Autoaim.mode_cnt[Change]=0;
		if (Brain.Autoaim.mode_cnt[Change]>=1000) {Brain.Autoaim.mode=Cruise;Brain.Autoaim.mode_cnt[Change]=0;}
	
    if(tim14.ClockTime%10==0 &&Brain.Autoaim.mode!=Change) Brain.Autoaim.mode_cnt[Cruise]++;
	if (Brain.All_See.mode==Wait) Brain.All_See.mode_cnt[Wait]++;
	if ((Brain.Autoaim.mode_cnt[Cruise]>40&&Brain.Autoaim.Mode==Autoaim)|| (Brain.Autoaim.mode_cnt[Cruise]>100&&Brain.Autoaim.Mode==Outpost) || (Brain.Autoaim.mode_cnt[Cruise]>150&&Brain.Autoaim.Mode==Single) ) {Brain.Autoaim.mode=Cruise;Brain.Autoaim.mode_cnt[Cruise]=10;}
	
		if (Brain.All_See.mode_cnt[Wait]>2000) {Brain.All_See.mode=None;Brain.All_See.mode_cnt[Wait]=0;}
//		UsartDmaPrintf("%d,%d,%d\r\n",Brain.All_See.mode,Brain.Autoaim.mode,Brain.All_See.mode_cnt[1]);
		if (Brain.All_See.mode_cnt[Found]>=2){Brain.All_See.mode=Found;Brain.All_See.mode_cnt[Found]=0;}
	// Brain.Autoaim.Mode=2;
	if (rc_Ctrl_et.isOnline == 1 && Behind_camera.reset_Flag==0)
	{
		Behind_camera_Reset();
	}
		if (rc_Ctrl_et.isOnline == 1 && Behind_camera.reset_Flag==1)
	{
	Behind_camera_control();
	}
//	UsartDmaPrintf("%d,%.2f,%.2f\r\n",Brain.Autoaim.fire_flag,Holder.Yaw1.Target_Angle,Holder.Yaw1.Can_Angle);
	if (rc_Ctrl_et.isOnline == 1 ) 
		{
		  ShootPlateControl(&AmmoBooster,&Brain);

				HolderGetRemoteData(&Holder, &rc_Ctrl_et,&Brain);

	
			Lidar_Allchassis_control(&allchassis,&check_robot_state,&Brain, &rc_Ctrl_et);
		}
		   if (tim14.ClockTime>500) FrictionWheelControl(&AmmoBooster);
		if(rc_Ctrl_et.isOnline == 0) 	AmmoBooster.Shoot_Plate.Target_Angle = AmmoBooster.Shoot_Plate.Plate_Angle;	

  RobotOnlineState(&check_robot_state, &rc_Ctrl_et);
		 if(tim14.ClockTime%200==0)  sentry_decision_control();
	Change_BrainMode(&Brain);
	//Brain.Autoaim.Mode=2;
	RobotToBrain(&Brain);
		
		
		
	if (referee_Fps>=0 && referee2022.game_status.game_progress!=4)
	{
			referee2022.map_command_t.cmd_keyboard=0;
				referee2022.map_command_t.target_position_y=0;
				referee2022.map_command_t.target_position_x=0;
	}
		if(rc_Ctrl_et.isOnline == 1 && flag_fallover==0) {;}
		else 
		{	
			Behind_camera.reset_Flag=0;
			Behind_camera.reset_index=0;
			Behind_camera.reset_Target=-200;
			MotorFillData(&Behind_camera.Behind_camera_motor,0 );
			ET08Init(&rc_Ctrl_et);
			MotorFillData(&Holder.Motors6020.motor[0],0);
			MotorFillData(&Holder.Motors6020.motor[1],0);
			MotorFillData(&Holder.Motors6020.motor[2],0);
			for(int i=0;i<4;i++) MotorFillData(&allchassis.Motors.motor[i],0);
			MotorFillData(&AmmoBooster.Shoot_Plate.motor2006,0);
		}
		if ( referee2022.map_command_t.cmd_keyboard=='A'&& fabs(referee2022.map_command_t.target_position_y-8)<=2  && fabs(referee2022.map_command_t.target_position_x-14.18)<=13 && fabs(referee2022.map_command_t.target_position_x-14.18)>=10)
		{
					ET08Init(&rc_Ctrl_et);
			MotorFillData(&Holder.Motors6020.motor[0],0);
			MotorFillData(&Holder.Motors6020.motor[1],0);
			MotorFillData(&Holder.Motors6020.motor[2],0);
			for(int i=0;i<4;i++) MotorFillData(&allchassis.Motors.motor[i],0);
			MotorFillData(&AmmoBooster.Shoot_Plate.motor2006,0);
		}
//		if (tim14.ClockTime%200==0)
//		  	UsarttoWifi("%d,%.2f,%.2f,%.2f,%.2f,%d,%d\r\n",1,1.0,INS_attitude->roll,INS_attitude->pitch,Holder.Pitch.Can_Angle,tim14_FPS.Lidar_FPS,referee2022.game_status.stage_remain_time);

		



		if (referee2022.game_status.game_progress!=4) bullet_num_17mm=0; 
 Judege_reverge();
Brain.Autoaim.Last_mode=Brain.Autoaim.mode;

		if (fabs(mpu6050.Roll)>=40&&rc_Ctrl_et.isOnline == 1) cnt_fallover++;else cnt_fallover--;
 cnt_fallover=int16_constrain(cnt_fallover ,0, 4000);
		if (cnt_fallover>=3000) flag_fallover=1; else if (cnt_fallover<=1500) flag_fallover=0;
		
		
if (referee2022.game_robot_status.mains_power_gimbal_output==0) MotorFillData(&Holder.Motors6020.motor[0],0);
if ((allchassis.Motors.motor[0].Data.Online_check.FPS<500||allchassis.Motors.motor[1].Data.Online_check.FPS<500||allchassis.Motors.motor[2].Data.Online_check.FPS<500||allchassis.Motors.motor[3].Data.Online_check.FPS<500) &&flag_ppp==0)
{
	for (int i=0;i<4;i++)
	MotorFillData(&allchassis.Motors.motor[i],0);
	if (referee2022.game_robot_status.mains_power_chassis_output==1)cntppp++;else cntppp=0;
}
else cntppp=0;
if (cntppp>10000){flag_ppp=1;cntppp=0;}

 MotorCanOutput(can1, 0x1ff);
 MotorCanOutput(can1, 0x200);
		if (tim14.ClockTime%4==0)
 MotorCanOutput(can2, 0x1ff);
 MotorCanOutput(can2, 0x200);

	
		
//		if (tim14.ClockTime%200==0)
		
//		UsartDmaPrintf("%.2f,%d\r\n",-(Behind_camera.Behind_camera_motor.Data.TotalAngle-Behind_camera.deltaAngle-90),Behind_camera.Behind_camera_motor.Data.SpeedRPM);
	//UsartDmaPrintf("%d,%d\r\n",Brain.Autoaim.IsFire,Brain.Autoaim.fire_flag);
//  UsartDmaPrintf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",a1,a2,a3,a4,abs1,abs2,abs3,abs4,Holder.Motors6020.motor[0].Data.Angle);
//	UsartDmaPrintf("%.2f,%.2f,%.2f,%.2f\r\n",Holder.Yaw1.Target_Angle,Holder.Yaw1.Can_Angle,Holder.Pitch.GYRO_Angle,Holder.Pitch.Target_Angle);
	//	UsartDmaPrintf("%d\r\n",Holder.Motors6020.motor[0].Data.Output);
	//	UsartDmaPrintf("%d,%.2f,%.2f,%d\r\n",AmmoBooster.Friction_Wheel.motor3508[0].Data.SpeedRPM,AmmoBooster.Shoot_Plate.Plate_Angle,AmmoBooster.Shoot_Plate.Target_Angle,AmmoBooster.Shoot_Plate.motor2006.Data.Output);
	//UsartDmaPrintf("%d\r\n",Brain.Autoaim.mode);
}

void TIM13_Task(void)
{
	tim14_FPS.Gyro_Out_cnt++;
	MPU6050_Read_1(&mpu6050.mpu6050_Data);
	IMUupdate_1(&mpu6050.mpu6050_Data);
	//IMUupdate(&mpu6050.mpu6050_Data);
	INS_attitude = INS_GetAttitude(IMU_data);
//	BMI088_IMUupdate(&bmi088.bmi088_Data);
}
//start = DWT->CYCCNT;          // 记录开始周期
////INS_attitude = INS_GetAttitude(IMU_data);
//end = DWT->CYCCNT;            // 记录结束周期
//cycles = end - start;         // 计算周期数
//time_us = (float)cycles / SystemCoreClock * 1e6; // 转换为微秒/