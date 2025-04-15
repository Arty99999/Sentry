#include "Supercap.h"
#include "brain.h"
#include "hardware_config.h"
#include "shoot.h"

#include "gyro.h"
#include "all_chassis.h"
#include "dr16.h"
#include "et08.h"
#include "pid.h"
#include "holder.h"
#include "referee.h"
#include "brain.h"

#include "shoot.h"
#include "check.h"
#include "ladrc.h"
#include "control_logic.h"
#include "mpu6050.h"
#include "bmi088.h"
#include "brain.h"
#include "ins.h"
extern float a1,a2,a3,a4,abs1,abs2,abs3,abs4;
 int Flag_streng;
int16_t ThisSecond =0;
extern int flag00,flag01;
uint16_t Brain_Cnt=0;
extern int FLAG_1,FLAG_Send;
extern float Target_Angle,yaw1_Angle;
//< TIM14的触发频率在CubeMX中被配置为1000Hz
/**
  * @brief  定时器中断回调
  */
	extern int blue_3,Flag_not,Flag_Follow;
	int cnt_heat;
extern	float init_quaternion[4];
	extern float angle;
	extern	float nmm;
void TIM14_Task(void)
{
		tim14.ClockTime++;
	

	
		if(tim14.ClockTime%1000==0)  FPS_Check(&tim14_FPS);

    if(tim14.ClockTime%10==0 &&Brain.Autoaim.mode!=Change) Brain.Autoaim.mode_cnt[Cruise]++;
	if (Brain.All_See.mode==Wait) Brain.All_See.mode_cnt[Wait]++;
	if (Brain.Autoaim.mode_cnt[Cruise]>20) {Brain.Autoaim.mode=Cruise;Brain.Autoaim.mode_cnt[Cruise]=10;}
		if (Brain.All_See.mode_cnt[Wait]>1200) {Brain.All_See.mode=None;Brain.All_See.mode_cnt[Wait]=0;if (Brain.Autoaim.mode==Change)  Brain.Autoaim.mode=Cruise;}
		if (Brain.All_See.mode_cnt[Found]>2){Brain.All_See.mode=Found;Brain.All_See.mode_cnt[Found]=0;}
		
	if (rc_Ctrl.isOnline == 1 ) 
		{
//			ShootPlateControl(&AmmoBooster,&Brain);

		 HolderGetRemoteData(&Holder, &rc_Ctrl,&Brain);

	
			Lidar_Allchassis_control(&allchassis,&check_robot_state,&Brain, &rc_Ctrl);
		}
//	if (tim14.ClockTime>500)
//	FrictionWheelControl(&AmmoBooster);
		if(rc_Ctrl.isOnline == 0) 	AmmoBooster.Shoot_Plate.Target_Angle = AmmoBooster.Shoot_Plate.Plate_Angle;	
//	
  RobotOnlineState(&check_robot_state, &rc_Ctrl);
	RobotToBrain(&Brain);
		if(rc_Ctrl.isOnline == 1) {;}
		else 
		{	
			DR16Init(&rc_Ctrl);
			MotorFillData(&Holder.Motors6020.motor[0],0);
			MotorFillData(&Holder.Motors6020.motor[1],0);
			MotorFillData(&Holder.Motors6020.motor[2],0);
			for(int i=0;i<4;i++) MotorFillData(&allchassis.Motors.motor[i],0);
			MotorFillData(&AmmoBooster.Shoot_Plate.motor2006,0);
		}
		
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

	UsartDmaPrintf("%d,%d\r\n",Brain.Lidar.vx,Brain.Lidar.vy);
//	if (tim14.ClockTime%50==0)
//	UsartDmaPrintf("%d,%d\r\n",Brain.Autoaim.IsFire,Brain.Autoaim.fire_flag);
 // UsartDmaPrintf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",a1,a2,a3,a4,abs1,abs2,abs3,abs4,Holder.Motors6020.motor[0].Data.Angle);
	//UsartDmaPrintf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Holder.Yaw1.Target_Angle,Holder.Yaw1.Can_Angle,);
	//	UsartDmaPrintf("%d,%.2f,%.2f,%.2f,%.2f\r\n",Holder.Motors6020.motor[1].Data.Output,Holder.Pitch.GYRO_Angle_speed,Holder.Pitch.GYRO_Angle,Holder.Pitch.Target_Angle,Holder.Motors6020.turnPID[2].Out);
	//	UsartDmaPrintf("%d,%.2f,%.2f,%d\r\n",AmmoBooster.Friction_Wheel.motor3508[0].Data.SpeedRPM,AmmoBooster.Shoot_Plate.Plate_Angle,AmmoBooster.Shoot_Plate.Target_Angle,AmmoBooster.Shoot_Plate.motor2006.Data.Output);
	//UsartDmaPrintf("%d\r\n",Brain.Autoaim.mode);
}

void TIM13_Task(void)
{
	tim14_FPS.Gyro_Out_cnt++;
	MPU6050_Read(&mpu6050.mpu6050_Data);	
	 IMUupdate_1(&mpu6050.mpu6050_Data);
	if (flag00==0) INS_attitude = INS_GetAttitude(IMU_data);
	
}
