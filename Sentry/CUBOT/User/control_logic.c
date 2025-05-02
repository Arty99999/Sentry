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




extern int flag00,flag01;
extern int FLAG_1,FLAG_Send;

uint8_t referee_cnt,referee_Fps;

int16_t ThisSecond =0;
	int cnt_heat;
extern	float init_quaternion[4];
	extern float angle;
extern float a222;
extern int flag000;
extern int change_position;
int cnt000;
void TIM14_Task(void)
{
	
	static int remain_num;
		tim14.ClockTime++;
	

	if(tim14.ClockTime%5000==0) 
	{referee_Fps=referee_cnt;
		referee_cnt=0;
	}
	if(tim14.ClockTime%2500==0&& Brain.Lidar.mode==3&&Brain.Lidar.Arrive==1&&change_position==0)  change_position=1;
	else if(tim14.ClockTime%2500==0&& Brain.Lidar.mode==3&&Brain.Lidar.Arrive==1&&change_position==1)  change_position=2;
	else if(tim14.ClockTime%2500==0&& Brain.Lidar.mode==3&&Brain.Lidar.Arrive==1&&change_position==2)  change_position=0;
	
		if(tim14.ClockTime%1000==0) FPS_Check(&tim14_FPS);
	if (Brain.Lidar.mode== 4&& Brain.Lidar.movemode==0) {flag000=0;a222=0;}
    if(tim14.ClockTime%10==0 &&Brain.Autoaim.mode!=Change) Brain.Autoaim.mode_cnt[Cruise]++;
	if (Brain.All_See.mode==Wait) Brain.All_See.mode_cnt[Wait]++;
	if (Brain.Autoaim.mode_cnt[Cruise]>30) {Brain.Autoaim.mode=Cruise;Brain.Autoaim.mode_cnt[Cruise]=10;}
		if (Brain.All_See.mode_cnt[Wait]>1200) {Brain.All_See.mode=None;Brain.All_See.mode_cnt[Wait]=0;if (Brain.Autoaim.mode==Change)  Brain.Autoaim.mode=Cruise;}
		if (Brain.All_See.mode_cnt[Found]>2){Brain.All_See.mode=Found;Brain.All_See.mode_cnt[Found]=0;}
	if (rc_Ctrl_et.isOnline == 1 ) 
		{
		ShootPlateControl(&AmmoBooster,&Brain);

		 HolderGetRemoteData(&Holder, &rc_Ctrl_et,&Brain);

	
			Lidar_Allchassis_control(&allchassis,&check_robot_state,&Brain, &rc_Ctrl_et);
		}
if (tim14.ClockTime>500) FrictionWheelControl(&AmmoBooster);
		if(rc_Ctrl_et.isOnline == 0) 	AmmoBooster.Shoot_Plate.Target_Angle = AmmoBooster.Shoot_Plate.Plate_Angle;	
//	
  RobotOnlineState(&check_robot_state, &rc_Ctrl_et);
		Brain.Autoaim.Mode=Outpost;
		 if(tim14.ClockTime%200==0)  sentry_decision_control();
		Change_BrainMode(&Brain);

	RobotToBrain(&Brain);
		if(rc_Ctrl_et.isOnline == 1) {;}
		else 
		{	
			ET08Init(&rc_Ctrl_et);
			MotorFillData(&Holder.Motors6020.motor[0],0);
			MotorFillData(&Holder.Motors6020.motor[1],0);
			MotorFillData(&Holder.Motors6020.motor[2],0);
			for(int i=0;i<4;i++) MotorFillData(&allchassis.Motors.motor[i],0);
			MotorFillData(&AmmoBooster.Shoot_Plate.motor2006,0);
		}
//		if (tim14.ClockTime%500==0 )
//		{
//		UsarttoWifi("AT+CIPSEND=%d\r\n",1);
//			FLAG_Send=0;
//		}
		if (tim14.ClockTime%200==0)
			UsarttoWifi("%d\r\n",Brain.Lidar.vx);
		
		if (referee2022.game_status.game_progress!=4) bullet_num_17mm=0; 
		
		
		
		
//			if (FLAG_1==1)
//	{
//		UsarttoWifi("1");
//			FLAG_1=0;
//	}
//if (Flag_not!=0) cnt25++;
//if (cnt25>2000) {Flag_not=0;cnt25=0;}

if (referee2022.game_robot_status.mains_power_gimbal_output==0) MotorFillData(&Holder.Motors6020.motor[0],0);

 Judege_reverge();
Brain.Autoaim.Last_mode=Brain.Autoaim.mode;
		if (flag01==0)
		{MotorCanOutput(can1, 0x1ff);
 MotorCanOutput(can1, 0x200);
 MotorCanOutput(can2, 0x1ff);
 MotorCanOutput(can2, 0x200);}
  	//UsartDmaPrintf("%d,%d,%d,%d,%d,%d\r\n",bmi088.bmi088_Data.Raw_accel[0],bmi088.bmi088_Data.Raw_accel[1],bmi088.bmi088_Data.Raw_accel[2],bmi088.bmi088_Data.Raw_gyro[0],bmi088.bmi088_Data.Raw_gyro[1],bmi088.bmi088_Data.Raw_gyro[2]);
//	if (tim14.ClockTime%50==0)
//	UsartDmaPrintf("%d,%d\r\n",Brain.Autoaim.IsFire,Brain.Autoaim.fire_flag);
//  UsartDmaPrintf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",a1,a2,a3,a4,abs1,abs2,abs3,abs4,Holder.Motors6020.motor[0].Data.Angle);
	//UsartDmaPrintf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Holder.Yaw1.Target_Angle,Holder.Yaw1.Can_Angle,Holder.Yaw.Target_Angle,Holder.Yaw.GYRO_Angle);
		UsartDmaPrintf("%d,%.2f,%.2f,%.2f,%.2f\r\n",Holder.Motors6020.motor[2].Data.Output,Holder.Yaw1.GYRO_Angle_speed,Holder.Yaw1.Can_Angle,Holder.Yaw1.Target_Angle,Holder.Yaw1.PID.ShellPID->Out);
	//	UsartDmaPrintf("%d,%.2f,%.2f,%d\r\n",AmmoBooster.Friction_Wheel.motor3508[0].Data.SpeedRPM,AmmoBooster.Shoot_Plate.Plate_Angle,AmmoBooster.Shoot_Plate.Target_Angle,AmmoBooster.Shoot_Plate.motor2006.Data.Output);
	//UsartDmaPrintf("%d\r\n",Brain.Autoaim.mode);
}

int cnt1111;
void TIM13_Task(void)
{
	tim14_FPS.Gyro_Out_cnt++;
	MPU6050_Read(&mpu6050.mpu6050_Data);	
	 IMUupdate_1(&mpu6050.mpu6050_Data);
	if (flag00==0 ) INS_attitude = INS_GetAttitude(IMU_data);
	
}
