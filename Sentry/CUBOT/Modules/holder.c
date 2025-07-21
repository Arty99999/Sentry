#include "holder.h"
#include "control_logic.h"

#include "brain.h"
#include "filter.h"
#include <math.h>
#include <ins.h>
#include <mpu6050.h>
#include "hardware_config.h"
#include "all_chassis.h"

#define abs(x) ((x)>0? (x):(-(x)))

Behind_camera_t Behind_camera={.index_camera=0,.camera_turnFlag=0,.reset_Flag=0,.reset_index=0};
Hurt_state hurt=HURT_IDLE;
Shoot_state shoot=SHOOT_IDLE;
Holder_t Holder;
uint8_t Choose_Target(Brain_t* brain);
int a111=0;
int cnt_35081;
int cnt111=0;
float a12;
void Camare_control(Brain_t* brain,Holder_t* holder);
/**
  * @brief 云台初始化
  */
void HolderInit(Holder_t* holder,DualPID_Object* pitch_pid ,DualPID_Object* yaw_pid,DualPID_Object* yaw1_pid,CanNumber canx)
{
	MotorInit(&holder->Motors6020.motor[0],9400 ,Motor6020,CAN2,0x205);   
	MotorInit(&holder->Motors6020.motor[1], 8167 ,Motor6020,canx,0x206);
	MotorInit(&holder->Motors6020.motor[2], 5626 ,Motor6020,canx,0x205);   

	DualPID_Init(&holder->Pitch.PID,pitch_pid->ShellPID,pitch_pid->CorePID);
  DualPID_Init(&holder->Yaw.PID,yaw_pid->ShellPID,yaw_pid->CorePID);
  DualPID_Init(&holder->Yaw1.PID,yaw1_pid->ShellPID,yaw1_pid->CorePID);
	
	holder->Pitch.Sensitivity = -0.00085f;  //-0.0015f
	holder->Yaw.Sensitivity = 0.0010f;     //0.003f /0.0015
	holder->Pitch.MouseSensitivity=-0.006f;
	holder->Yaw.MouseSensitivity=0.006f;
	holder->Cruise_Mode.yaw1_sense=0.00325;
	holder->Cruise_Mode.pitch_sense=0.0065;
	holder->up_litmit=40;
	holder->down_litmit=-35;//-35
	
	holder->right_litmit=-85;
	holder->left_litmit=75;
}
uint8_t flag000;
uint32_t m=0;
int cntmm;
extern float a222;
extern uint8_t referee_Fps;
float y3,k3;
 extern int hurt_flag,cnth;
int cntxx;

 int hurt_flag,cnt_hurt_reset;
void HolderGetRemoteData(Holder_t* holder, RC_Ctrl_ET* rc_ctrl,Brain_t* brain) 
{
	static int16_t yaw_time,pitch_time;
	holder->Pitch.Target_Angle -= ((rc_ctrl->rc.ch3 -1024) * holder->Pitch.Sensitivity);
	if(rc_ctrl->rc.s2!=1) holder->Yaw.Target_Angle += ((rc_ctrl->rc.ch2 -1024)* holder->Yaw.Sensitivity);
	else if(rc_ctrl->rc.s2==1)holder->Yaw1.Target_Angle += ((rc_ctrl->rc.ch2 -1024)* holder->Yaw.Sensitivity);

	if(brain->Autoaim.mode==Cruise&&(rc_Ctrl_et.rc.s2==2||referee2022.game_status.game_progress==4 )&&brain->All_See.mode!=Wait&&tim14_FPS.Vision_FPS>0)
			{
//				if (brain->Autoaim.Last_mode!=Cruise)
//				{
//					holder->Cruise_Mode.yaw1_time=asin(holder->Yaw.Can_Angle/85)/holder->Cruise_Mode.yaw1_sense;
//				  holder->Cruise_Mode.pitch_time=(holder->Pitch.GYRO_Angle-5)/25/holder->Cruise_Mode.pitch_sense;
//				}
				brain->Autoaim.Attack_state.shoot_num=0;
		m++;
				if (m<=14000) 
				{
					holder->Cruise_Mode.pitch_time++;
				holder->Cruise_Mode.yaw1_time++;
					
				}
				else if (m>15000) {m=0;}
				else {holder->Cruise_Mode.pitch_time=0;holder->Cruise_Mode.yaw1_time=0;}
				if (brain->Autoaim.Mode==Autoaim)
				{Holder.Yaw1.Target_Angle = 70.0f*sin(holder->Cruise_Mode.yaw1_time*holder->Cruise_Mode.yaw1_sense);
			Holder.Pitch.Target_Angle = 5-abs(sin(holder->Cruise_Mode.pitch_time*holder->Cruise_Mode.pitch_sense))*25.0f;}
				else 
				{Holder.Yaw1.Target_Angle = 75.0f*sin(holder->Cruise_Mode.yaw1_time*holder->Cruise_Mode.yaw1_sense);
			Holder.Pitch.Target_Angle = 5+abs(sin(holder->Cruise_Mode.pitch_time*holder->Cruise_Mode.pitch_sense))*25.0f;}
				
			
			}
			else m=0;
			if (a222!=0 && flag000==0&&Brain.Lidar.mode==4&&cnth==0) {  if (a222>180&&a222<=270) Holder.Yaw.Target_Angle-=(a222-360)*57.3;else if (a222>=-90&&a222<0) Holder.Yaw.Target_Angle-=a222*57.3;flag000=1;}

			if (brain->Autoaim.mode==Cruise && brain->Autoaim.Mode==Outpost&&referee2022.game_status.game_progress==4) cntmm++;else cntmm=0;
				
				if (cntmm>3000) {Holder.Yaw.Target_Angle-=180;cntmm=0;}
//				if (brain->Autoaim.mode==Cruise && hurt_flag==1) cntxx++;else cntxx=0;
//				if (cntxx>1500) {Holder.Yaw.Target_Angle-=180;cntxx=0;}
				//		if (brain->All_See.mode==Found && brain->Autoaim.mode==Cruise&&brain->Autoaim.Mode==Autoaim && referee2022.game_status.game_progress==4&&referee2022.game_status.stage_remain_time<=400)
		if (brain->All_See.mode==Found && brain->Autoaim.mode==Cruise&&brain->Autoaim.Mode==Autoaim )
		{
			brain->Autoaim.mode=Change;
//
		Camare_control(brain,holder);
		brain->All_See.mode=Wait;
		}	
		else if (brain->All_See.mode==Found) Brain.All_See.mode=None;
//		else brain->All_See.mode=None;
			
		if (holder->Yaw1.Can_Angle>61) holder->down_litmit=-19;else holder->down_litmit=-35;
	
	holder->Yaw.Can_Angle = holder->Motors6020.motor[0].Data.Angle;
	holder->Pitch.Can_Angle = holder->Motors6020.motor[1].Data.Angle;
	holder->Yaw1.Can_Angle = holder->Motors6020.motor[2].Data.Angle
			;
	holder->Yaw.Can_Angle_speed  =  holder->Motors6020.motor[0].Data.SpeedRPM/2;
	holder->Yaw1.Can_Angle_speed  =  holder->Motors6020.motor[2].Data.SpeedRPM;
	holder->Pitch.Can_Angle_speed  =  holder->Motors6020.motor[1].Data.SpeedRPM;
			
	 holder->Yaw.GYRO_Angle=mpu6050.Yaw_total_angle;
	 holder->Yaw1.GYRO_Angle=INS_attitude->yaw;
	 holder->Yaw.GYRO_Angle_speed=-150*mpu6050.mpu6050_Data.gyro[2]*0.001*50*3;
	 holder->Pitch.GYRO_Angle_speed=-(INS_attitude->gyro[0]*0.001)*150*57.32;
	 holder->Yaw1.GYRO_Angle_speed=((INS_attitude->gyro[2]-mpu6050.mpu6050_Data.gyro[2])*0.001)*150*50*2;
	 holder->Pitch.GYRO_Angle=-INS_attitude->pitch;

	if(tim14.ClockTime%100==0&& holder->Yaw_Fllow_Mode.Flag_Fllow==0&&brain->Autoaim.mode!=Cruise)//大云台跟随
	{
	if(holder->Yaw1.Target_Angle>(holder->left_litmit-10)) {holder->Yaw.Target_Angle+=60.0f;holder->Yaw_Fllow_Mode.Flag_Fllow=1;}			
	else if(holder->Yaw1.Target_Angle<(holder->right_litmit+10))	{holder->Yaw.Target_Angle-=60.0f;holder->Yaw_Fllow_Mode.Flag_Fllow=1;}
	}
	if (holder->Yaw_Fllow_Mode.Flag_Fllow==1) holder->Yaw_Fllow_Mode.Lock_cnt++;
	if (holder->Yaw_Fllow_Mode.Lock_cnt>=500)  {holder->Yaw_Fllow_Mode.Lock_cnt=0;holder->Yaw_Fllow_Mode.Flag_Fllow=0;}


 holder->Yaw1.Target_Angle =float_constrain(holder->Yaw1.Target_Angle,holder->right_litmit,holder->left_litmit);	
 holder->Pitch.Target_Angle =float_constrain(holder->Pitch.Target_Angle,holder->down_litmit,holder->up_litmit);		
thinchicken_feedback_control();

	holder->Motors6020.motor[0].Data.Output =
	BasePID_SpeedControl( holder->Yaw.PID.CorePID, 
	BasePID_AngleControl(holder->Yaw.PID.ShellPID, holder->Yaw.Target_Angle , holder->Yaw.GYRO_Angle)  ,holder->Yaw.GYRO_Angle_speed);

	holder->Motors6020.motor[1].Data.Output =
	BasePID_SpeedControl(holder->Pitch.PID.CorePID , 
	BasePID_AngleControl(holder->Pitch.PID.ShellPID , holder->Pitch.Target_Angle , holder->Pitch.GYRO_Angle)  ,holder->Pitch.GYRO_Angle_speed);

//	if (Brain.Autoaim.vison_mode==1)  holder->Yaw1.PID.ShellPID->Kp=13;
//		else holder->Yaw1.PID.ShellPID->Kp=8.5;
//	if (Brain.Autoaim.vison_mode==1)  holder->Yaw1.PID.ShellPID->Kp=13;
//		else
			holder->Yaw1.PID.ShellPID->Kp=2;
	holder->Motors6020.motor[2].Data.Output =
	BasePID_SpeedControl(holder->Yaw1.PID.CorePID ,
	BasePID_AngleControl(holder->Yaw1.PID.ShellPID , holder->Yaw1.Target_Angle , holder->Yaw1.Can_Angle)  ,-holder->Yaw1.GYRO_Angle_speed);
	
for (int i=0;i<3;i++)
if (i!=2)
  MotorFillData(&holder->Motors6020.motor[i], holder->Motors6020.motor[i].Data.Output);

}
 void thinchicken_feedback_control()//瘦鸡反馈
 {
static int cnt_hurt_doubt,blood;
	static uint16_t bullet,cnt_shoot_reset,cnt_shoot;
	switch (hurt) {
        case HURT_IDLE:
				{
			     if ((blood-referee2022.game_robot_status.remain_HP>=90) && referee2022.robot_hurt.hurt_type==0) hurt=HURT_ATTACKED;
           else if ((blood-referee2022.game_robot_status.remain_HP)<=58&&(blood-referee2022.game_robot_status.remain_HP)>=8&&referee2022.robot_hurt.hurt_type==0) hurt=HURT_ATTACKED;
				}
       break;

        case HURT_ATTACKED:
				{
             cnt_hurt_reset++;
				if ((blood-referee2022.game_robot_status.remain_HP>=8) && referee2022.robot_hurt.hurt_type==0) cnt_hurt_reset=0;
            if (cnt_hurt_reset>2000) {hurt=HURT_IDLE;cnt_hurt_reset=0;}
					}
          break;

				 case HURT_DOUBT:
         {
            cnt_hurt_doubt++;
     if (cnt_hurt_doubt>10&cnt_hurt_doubt<2000 && (blood-referee2022.game_robot_status.remain_HP>=8&&referee2022.robot_hurt.hurt_type==0)) hurt=HURT_ATTACKED;
	 else if (cnt_hurt_doubt>2000) {hurt=HURT_IDLE;cnt_hurt_doubt=0;}
	 
         }
          break;		
								
    }
	
		
    	switch (shoot) {
								 case SHOOT_DOUBT:
         {

            cnt_shoot_reset++;
					  if (bullet!=bullet_num_17mm) {cnt_shoot++;}
         if (cnt_shoot_reset==500) {if (cnt_shoot>=1) shoot=SHOOT_ING;else shoot=SHOOT_IDLE; cnt_shoot_reset=0;cnt_shoot=0;}
			 }
          break;	
        case SHOOT_IDLE:
				{

              if (bullet!=bullet_num_17mm){shoot=SHOOT_DOUBT;}
				}
       break;

        case SHOOT_ING:
					{
             cnt_shoot_reset++;
			 if (bullet!=bullet_num_17mm) {cnt_shoot++;}
            if (cnt_shoot_reset==1000) {if (cnt_shoot<=2) shoot=SHOOT_IDLE; cnt_shoot=0;cnt_shoot_reset=0;}
					}
          break;
								
    }

	 blood=referee2022.game_robot_status.remain_HP;
   bullet=bullet_num_17mm;
}
#define h1 186
#define h2 279
#define c1 100
#define c2 182.5
float Target_Angle,yaw1_Angle;
//int b;
void Camare_control(Brain_t* brain,Holder_t* holder)
{
	uint8_t Target=Choose_Target(brain);
//	b=brain->All_See.armorNumber[Target];
	float a=brain->All_See.Yaw_add[Target];
//if (a-(Behind_camera.Behind_camera_motor.Data.TotalAngle-Behind_camera.deltaAngle-90)&&a-(Behind_camera.Behind_camera_motor.Data.TotalAngle-Behind_camera.deltaAngle-90)>=-90)
//{
//	UsartDmaPrintf("%.2f\r\n",Target_Angle);
	Target_Angle=90+a+(Behind_camera.Behind_camera_motor.Data.TotalAngle-Behind_camera.deltaAngle-90);
	UsartDmaPrintf("%.2f\r\n",Target_Angle);
  yaw1_Angle=90-holder->Yaw1.Can_Angle;
	if ((Target_Angle+yaw1_Angle)<180)
	{if 		((5*Target_Angle)<2*yaw1_Angle-140) {Holder.Yaw.Target_Angle+=(yaw1_Angle+Target_Angle)/3.5;Holder.Yaw1.Target_Angle+=(yaw1_Angle+Target_Angle)/3.5*2.5;}
	else {Holder.Yaw.Target_Angle+=Target_Angle+(115-holder->left_litmit);Holder.Yaw1.Target_Angle=holder->left_litmit-25;}}
	else 
		{if 		(0>(2*yaw1_Angle-5*Target_Angle+645)) {Holder.Yaw.Target_Angle-=(360-yaw1_Angle-Target_Angle)/3.5;Holder.Yaw1.Target_Angle-=(360-yaw1_Angle-Target_Angle)/3.5*2.5;}
	else {Holder.Yaw.Target_Angle-=(270+holder->right_litmit+25)-Target_Angle;Holder.Yaw1.Target_Angle=holder->right_litmit+25;}}
			//Holder.Pitch.Target_Angle= - atan((brain->All_See.Distance[0]/1000.0*sin(-1*brain->All_See.Pitch_add[0]/57.3)-0.0725)/(0.06+brain->All_See.Distance[0]/1000.0*cos(-1*brain->All_See.Pitch_add[0]/57.3)))*57.3;
	Holder.Pitch.Target_Angle=atan((c1+brain->All_See.Distance[0]*sin(brain->All_See.Pitch_add[0]/57.3)-h1)/(h2-(c2-brain->All_See.Distance[0]*cos(brain->All_See.Pitch_add[0]/57.3))))*57.3;
//}
//	Holder.Pitch.Target_Angle= -15;
}

uint8_t Choose_Target(Brain_t* brain)
{
	 uint8_t best_index=0;
	float best_priority=10000; 
	if (brain->All_See.Find_size==1)   return (0);
	for (int i=0;i<brain->All_See.Find_size;i++)
	{
		float priority = 0;
        if (brain->All_See.Distance[i] > 5000) priority+=2000;
		
        

        // (1) 血量优先级（血量越低越好）
		if (referee2022.game_robot_status.robot_id>10)
		{

		priority -= (brain->All_See.armorNumber[i] == 0 ? 
             referee2022.game_robot_hp.red_robot_HP[7] : 
             referee2022.game_robot_hp.red_robot_HP[brain->All_See.armorNumber[i]]) < 100 ? 1000 : 0;
			
			
		}
     else 
	   {
		priority -= (brain->All_See.armorNumber[i] == 0 ? 
             referee2022.game_robot_hp.blue_robot_HP[7] : 
             referee2022.game_robot_hp.blue_robot_HP[brain->All_See.armorNumber[i]]) < 100 ? 1000 : 0;
		}
        // (2) 装甲板 ID 优先级（2 > 1 > 0 > 3 = 4）
        switch (brain->All_See.armorNumber[i]) {
            case 2: priority -= 500; break;  // 最高优先级
            case 1: priority -= 200;  break;
            case 0: priority -= 100;  break;
            case 3:
            case 4: priority += 0;    break;  // 最低优先级
            default: priority += 100; break;  // 未知 ID 降低优先级
        }
//        priority += targets[i].Distance * 0.001; 

        // 3. 更新最优目标
        if (priority < best_priority) {
            best_priority = priority;
            best_index = i;
        }
    }

		
		
    return best_index;
		
}
void Behind_camera_Init(void)
{
	MotorInit(&Behind_camera.Behind_camera_motor,0,Motor3508,CAN1,0x204);
	BasePID_Init(&Behind_camera.Bcamera_motor_pid, 20, 0, 0,  10);
	BasePID_Init(&Behind_camera.Bcamera_motor_Inpid, 5, 0, -30,  10);
	BasePID_Init(&Behind_camera.Reset_pid, 50, 0, 0,  10);
	Behind_camera.Behind_camera_motor.Data.Target=0;
	Behind_camera.reset_Target=-200;
}
void Behind_camera_Reset(void)
{
	Behind_camera.reset_index++;
	Behind_camera.Behind_camera_motor.Data.Output=BasePID_SpeedControl(&Behind_camera.Reset_pid,
			Behind_camera.reset_Target,Behind_camera.Behind_camera_motor.Data.SpeedRPM);
	if(Behind_camera.Behind_camera_motor.Data.SpeedRPM==0 && Behind_camera.reset_index>50)
	{
		Behind_camera.reset_Target=0;
		Behind_camera.reset_Flag=1;
		Behind_camera.deltaAngle=Behind_camera.Behind_camera_motor.Data.TotalAngle;
		
	}
	MotorFillData(&Behind_camera.Behind_camera_motor,Behind_camera.Behind_camera_motor.Data.Output );
}

int i=0;

void Behind_camera_control(void)
{
	
	Behind_camera.index_camera++;
	if(Behind_camera.index_camera%500==0 && Behind_camera.camera_turnFlag==0)
	{
		Behind_camera.Behind_camera_motor.Data.Target=0;
		Behind_camera.camera_turnFlag=1;
	}else if(Behind_camera.index_camera%500==0 && Behind_camera.camera_turnFlag==1)
	{
		Behind_camera.Behind_camera_motor.Data.Target=60;
		Behind_camera.camera_turnFlag=2;
	}else if(Behind_camera.index_camera%500==0 && Behind_camera.camera_turnFlag==2)
	{
		Behind_camera.Behind_camera_motor.Data.Target=120;
		Behind_camera.camera_turnFlag=3;
	}else if(Behind_camera.index_camera%500==0 && Behind_camera.camera_turnFlag==3)
	{
		Behind_camera.Behind_camera_motor.Data.Target=180;
		Behind_camera.camera_turnFlag=4;
	}else if(Behind_camera.index_camera%500==0 && Behind_camera.camera_turnFlag==4)
	{
		Behind_camera.Behind_camera_motor.Data.Target=120;
		Behind_camera.camera_turnFlag=5;
	}else if(Behind_camera.index_camera%500==0 && Behind_camera.camera_turnFlag==5)
	{
		Behind_camera.Behind_camera_motor.Data.Target=60;
		Behind_camera.camera_turnFlag=0;
	}
	Behind_camera.Behind_camera_motor.Data.Output=BasePID_SpeedControl(&Behind_camera.Bcamera_motor_pid,
		BasePID_AngleControl(&Behind_camera.Bcamera_motor_Inpid,Behind_camera.Behind_camera_motor.Data.Target,
		Behind_camera.Behind_camera_motor.Data.TotalAngle-Behind_camera.deltaAngle),Behind_camera.Behind_camera_motor.Data.SpeedRPM);
	MotorFillData(&Behind_camera.Behind_camera_motor,Behind_camera.Behind_camera_motor.Data.Output );
}