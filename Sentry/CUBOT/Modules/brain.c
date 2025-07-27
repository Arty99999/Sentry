#include "brain.h"
#include "hardware_config.h"

#include "control_logic.h"
#include "check.h"
#include "filter.h"

#include "string.h"
#include "ins.h"
#include "all_chassis.h"

void Armor_Ignore(Brain_t* brain);




void Single_Mode(Brain_t* Brain);


float angle_to_lidar;  //雷达建图坐标系到世界坐标系的角度，用于转换坐标系



int M111;




uint8_t RobotToBrainQuestBuffer[5] ;
uint8_t RobotToBrainCMDBuffer[3];                   //命令数据
uint8_t RobotToBrainLogBuffer[20];                  //日志数据
uint8_t RobotToBrainTimeBuffer[50];
uint8_t RobotToBrainChassisTimeBuffer[22];
uint8_t RobotToBrainQuestBuffer_XinTiaoBao[3] ;     //请求数据
uint8_t RobotToBrainQuestBuffer_WorkingModel[5] ;   //请求数据
uint8_t RobotToBrainQuestBuffer_Velocity[5] ; 



Brain_t Brain={.Lidar.mode=Lidar_Outpost};
Singe_state single;
void Brain_DataUnpack(Brain_t* brain, uint8_t* recBuffer);

/**
  * @brief   串口2视觉回调函数 
  * @param[in]  
  */
uint16_t aff;
uint8_t Brain_Autoaim_callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_vision_cnt=0;
//	tim14_FPS.Vision_cnt++;
	UsartDmaPrintf("%d\r\n",len);
  Brain_Autoaim_DataUnpack(&Brain,recBuffer);
	return 0;
}

uint8_t Brain_Camera_callback(uint8_t * recBuffer, uint16_t len)
{
	tim14_FPS.Camera_cnt++;
  Brain_Camera_DataUnpack_New(&Brain,recBuffer);
	return 0;
}

uint8_t Brain_Lidar_callback(uint8_t * recBuffer, uint16_t len)
{
	
	check_robot_state.Check_Usart.Check_lidar_cnt=0;
	tim14_FPS.Lidar_cnt++;
	Brain_Lidar_DataUnpack(&Brain,recBuffer);
	return 0;
}

/**
  * @brief  注册内核的回调函数
  */
//void Brain_Init(CubotBrain_t* brain, uint8_t index, Brain_CoreCallback callback)
//{
//	brain->BrainCore[index].CoreCallback = callback;
//}
void Brain_Camera_DataUnpack(Brain_t* Brain ,uint8_t * recBuffer)//解包雷达数据
{
	uint8_t k=0;
	if(recBuffer[0]==0xAA  &&tim14_FPS.Camera_FPS>10&&recBuffer[1]<5)
	{
		Brain->All_See.Find_size=recBuffer[1];
		if ( Brain->All_See.Find_size==0) 
		{
			memset(&Brain->All_See.Camera_Index, 0, 
       sizeof(Brain->All_See.Camera_Index) + 
       sizeof(Brain->All_See.armorNumber) + 
       sizeof(Brain->All_See.Pitch_add) + 
       sizeof(Brain->All_See.Yaw_add) + 
       sizeof(Brain->All_See.Distance));
		}
		
		
		if	(Brain->All_See.mode!=Wait) Brain->All_See.mode=None;
		
for (int i=0;i< Brain->All_See.Find_size;i++)
			{
				if	(Brain->All_See.mode!=Wait) Brain->All_See.mode_cnt[Found]++;else Brain->All_See.mode_cnt[Found]=0;
				
		  Brain->All_See.armorNumber[i] = recBuffer[i*7+2] % 9;
      Brain->All_See.Camera_Index[i]=recBuffer[i*7+3];
			Brain->All_See.Distance[i]=10*recBuffer[i*7+8];
					if((recBuffer[i*7+4] >> 6) == 0) 
				Brain->All_See.Yaw_add[i] = ((float)((recBuffer[i*7+4]&0x3f)*100 + recBuffer[i*7+5])/100);
			else if((recBuffer[i*7+4] >> 6) == 1) 
				Brain->All_See.Yaw_add[i] = (-1) * ((float)((recBuffer[i*7+4]&0x3f)*100 + recBuffer[i*7+5])/100);
			
			if((recBuffer[i*7+6] >> 6) == 0) 
				Brain->All_See.Pitch_add[i] = ((float)((recBuffer[i*7+6]&0x3f)*100 + recBuffer[i*7+7])/100);
			else if((recBuffer[i*7+6] >> 6) == 1) 
				Brain->All_See.Pitch_add[i] = (-1) * ((float)((recBuffer[i*7+6]&0x3f)*100 + recBuffer[i*7+7])/100);
       
		}
		
	}
}
void Brain_Camera_DataUnpack_New(Brain_t* Brain ,uint8_t * recBuffer)//解包
{
	uint8_t k=0;
	if(recBuffer[0]==0xAA  &&(recBuffer[recBuffer[1]*6+2])==0xDD&&tim14_FPS.Camera_FPS>5&&tim14_FPS.Camera_FPS<50)
	{
		Brain->All_See.Find_size=recBuffer[1];
		
		
for (int i=0;i< Brain->All_See.Find_size;i++)
			{     
				if	(Brain->All_See.mode!=Wait) Brain->All_See.mode_cnt[Found]++;else Brain->All_See.mode_cnt[Found]=0;
				
				if (recBuffer[i*6+2]>11) break;
		  Brain->All_See.armorNumber[i] = recBuffer[i*6+2];

			Brain->All_See.Distance[i]=10*recBuffer[i*6+7];
					if((recBuffer[i*6+3] >> 7) == 0) 
				Brain->All_See.Yaw_add[i] = ((float)((recBuffer[i*6+3]&0x7f)*100 + recBuffer[i*6+4])/100);
			else if((recBuffer[i*6+3] >> 7) == 1) 
				Brain->All_See.Yaw_add[i] = (-1) * ((float)((recBuffer[i*6+3]&0x7f)*100 + recBuffer[i*6+4])/100);
			
			if((recBuffer[i*6+5] >> 7) == 0) 
				Brain->All_See.Pitch_add[i] = ((float)((recBuffer[i*6+5]&0x7f)*100 + recBuffer[i*6+6])/100);
			else if((recBuffer[i*6+5] >> 7) == 1) 
				Brain->All_See.Pitch_add[i] = (-1) * ((float)((recBuffer[i*6+5]&0x7f)*100 + recBuffer[i*6+6])/100);
       
		
		
	    }
   }
}
float a222;
void Brain_Lidar_DataUnpack(Brain_t* Brain ,uint8_t * recBuffer)//解包雷达数据
{
	if(recBuffer[0]==0xAA)
	{
		Brain->Lidar.Brain_Data.FrameType= recBuffer[1];
			
		Brain->Lidar.movemode=recBuffer[2];
     
		if(Brain->Lidar.Brain_Data.FrameType == BRAIN_TO_ROBOT_CMD)  //< 解算偏转角
		{	
				Brain->Lidar.vy=1*((recBuffer[3] >> 6) == 0 ? 1 : -1)*((float)((recBuffer[3]&0x3f)*100 + recBuffer[4])/100)*1146*3;
				Brain->Lidar.vx=-1*((recBuffer[5] >> 6) == 0 ? 1 : -1)*((float)((recBuffer[5]&0x3f)*100 + recBuffer[6])/100)*1146*3;				
				//Brain->Lidar.angle_to_lidar=((recBuffer[7] >> 6) == 0 ? 1 : -1)*((float)((recBuffer[7]&0x3f)*100 + recBuffer[8])/100);
			  Brain->Lidar.Arrive = recBuffer[9];
				a222=((recBuffer[10] >> 6) == 0 ? 1 : -1)*((float)((recBuffer[10]&0x3f)*100 + recBuffer[11])/100);
		}
	}
}

void  Brain_Autoaim_DataUnpack(Brain_t* Brain ,uint8_t * recBuffer)//解包自瞄数据
{
	
	if(recBuffer[0]==0xAB  &&(recBuffer[recBuffer[1]*6+2])==0xDD)
	{
		Brain->All_See.Find_size=recBuffer[1];
		tim14_FPS.Camera_cnt++;
		
for (int i=0;i< Brain->All_See.Find_size;i++)
			{  
				if	(Brain->All_See.mode!=Wait&&abs(Behind_camera.Behind_camera_motor.Data.SpeedRPM)<5) Brain->All_See.mode_cnt[Found]++;else Brain->All_See.mode_cnt[Found]=0;
				
				if (recBuffer[i*6+2]>11) break;
		  Brain->All_See.armorNumber[i] = recBuffer[i*6+2];

			Brain->All_See.Distance[i]=100*recBuffer[i*6+7];
					if((recBuffer[i*6+3] >> 7) == 0) 
				Brain->All_See.Yaw_add[i] = ((float)((recBuffer[i*6+3]&0x7f)*100 + recBuffer[i*6+4])/100);
			else if((recBuffer[i*6+3] >> 7) == 1) 
				Brain->All_See.Yaw_add[i] = (-1) * ((float)((recBuffer[i*6+3]&0x7f)*100 + recBuffer[i*6+4])/100);
			
			if((recBuffer[i*6+5] >> 7) == 0) 
				Brain->All_See.Pitch_add[i] = ((float)((recBuffer[i*6+5]&0x7f)*100 + recBuffer[i*6+6])/100);
			else if((recBuffer[i*6+5] >> 7) == 1) 
				Brain->All_See.Pitch_add[i] = (-1) * ((float)((recBuffer[i*6+5]&0x7f)*100 + recBuffer[i*6+6])/100);
       
		
		
	    }
   }
	
	if(recBuffer[0] == 0xAA && recBuffer[1]<40&&recBuffer[recBuffer[1]-1]==0xDD )
	{
      tim14_FPS.Vision_cnt++;
		if( recBuffer[1] == 3)  Brain->Autoaim.fire_flag=0;
		else 
		{
			
			Brain->Autoaim.mode_cnt[Cruise] = 0;
			
			Brain->Autoaim.mode=Lock;
			
      Brain->Autoaim.Yaw_add = ((recBuffer[2] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[2] & 0x3f) * 100 + recBuffer[3]) / 100);
      Brain->Autoaim.Pitch_add =((recBuffer[4] >> 6) == 0 ? 1 : -1)* ((float)((recBuffer[4] & 0x3f) * 100 + recBuffer[5]) / 100);
			

			Brain->Autoaim.Distance = (float)(recBuffer[6])*100;
			
      Brain->Autoaim.IsFire = recBuffer[7];
			Brain->Autoaim.IsFire =1;
			Brain->Autoaim.vison_mode=recBuffer[8];
			
		//	Brain->Autoaim.Attack_state.camara_num=recBuffer[11];
		//	if (Brain->Autoaim.Attack_state.camara_num>=9) Brain->Autoaim.Attack_state.camara_num-=9;

//			Brain->All_See.armorNumber[0] = Brain->Autoaim.camara_num;
//			Brain->All_See.Distance[0]=Brain->Autoaim.Distance*100;
			
			
			
      Brain->Autoaim.fire_flag=0;
			if (rc_Ctrl_et.rc.s2==2 ||rc_Ctrl_et.rc.s2==1)
			{ 
				
				if (fabs(Holder.Yaw1.Target_Angle-Holder.Yaw1.Can_Angle)<0.8 && Brain->Autoaim.Mode==Outpost) Brain->Autoaim.fire_flag=1;
				else if (fabs(Holder.Yaw1.Target_Angle-Holder.Yaw1.Can_Angle)<0.8 && Brain->Autoaim.Mode==Autoaim) Brain->Autoaim.fire_flag=1;
				else if (fabs(Holder.Yaw1.Target_Angle-Holder.Yaw1.Can_Angle)<0.2 && Brain->Autoaim.Mode==2&&fabs(Holder.Pitch.GYRO_Angle-Holder.Pitch.Target_Angle)<0.2) Brain->Autoaim.fire_flag=1;
				else Brain->Autoaim.fire_flag=0;	
					Holder.Yaw1.Target_Angle=Holder.Yaw1.Can_Angle+Brain->Autoaim.Yaw_add;
					Holder.Pitch.Target_Angle= Holder.Pitch.GYRO_Angle+Brain->Autoaim.Pitch_add ;
			}
	

			

			}				
		
		}
}
/**
  * @brief  下位机向上位机发送时间戳以及四元数
  */
int cnt___;
int n=0;
int change_position; 
void Single_Mode(Brain_t* Brain)
{
	static int Arrive_cnt,Find_cnt;
		      switch (single) {
         case Single_IDLE: 
					 
				 if (Brain->Lidar.mode==Lidar_Patrol && Brain->Lidar.change_position==4&& Brain->Lidar.Arrive==1) {single=Single_Arrive;}

					 break;
				 case Single_Arrive:
					Arrive_cnt++;
				 if (Arrive_cnt>=7000) {single=Single_Exit;Arrive_cnt=0;}
				 else if (Brain->Autoaim.mode==Lock){single=Single_Find;Arrive_cnt=0;}
					break;
         case Single_Find: 
					 
				 Find_cnt++;
				  if (Find_cnt>=7000) {single=Single_Exit;Find_cnt=0;}
					else if (Brain->Autoaim.mode==Cruise) Find_cnt+=2;
				 
          break;
				 
         case Single_Exit: 
					 break;

      }
	
	
	
}
void RobotToBrain_Autoaim(float yaw,Brain_t* brain)//发给自瞄
{
	int16_t tmp0,tmp1,tmp2,tmp3,tmp4,cnt;
	
	tmp0 = (int16_t)(INS_attitude->q[0] *  30000);
	tmp1 = -(int16_t)(INS_attitude->q[1] *  30000);
	tmp2 = -(int16_t)(INS_attitude->q[2] *  30000);
	tmp3 = (int16_t)(INS_attitude->q[3] *  30000);

							
	RobotToBrainTimeBuffer[0]  = 0xAA;

	
	RobotToBrainTimeBuffer[1]  = ( tim14.ClockTime >>24);    //定时器时间，int32_t型
	RobotToBrainTimeBuffer[2]  = ((tim14.ClockTime >>16)&0xff);
	RobotToBrainTimeBuffer[3]  = ((tim14.ClockTime >>8)&0xff);
	RobotToBrainTimeBuffer[4]  = ((tim14.ClockTime &0xff));
	
  RobotToBrainTimeBuffer[5] = (referee2022.game_robot_status.robot_id > 10) ? 0 : 1;
            
	RobotToBrainTimeBuffer[6] = tmp0 & 0xFF;                   //四元数q0，float型
	RobotToBrainTimeBuffer[7] = tmp0 >> 8;
	RobotToBrainTimeBuffer[8] = tmp1 & 0xFF;
	RobotToBrainTimeBuffer[9] = tmp1 >> 8;  
	RobotToBrainTimeBuffer[10] = tmp2 & 0xFF;                   //四元数q1，float型
	RobotToBrainTimeBuffer[11] = tmp2 >> 8;
	RobotToBrainTimeBuffer[12] = tmp3 & 0xFF;
	RobotToBrainTimeBuffer[13] = tmp3 >> 8;  
//brain->Autoaim.Mode=n;
  Armor_Ignore(brain); 
	RobotToBrainTimeBuffer[14] = brain->Autoaim.Mode;//1 是前哨站 0是普通 2是打符
  RobotToBrainTimeBuffer[15] = brain->Autoaim.Ignore_armorNumber;//


	RobotToBrainTimeBuffer[16] = 0xDD;
	RobotToBrainTimeBuffer[17] = 0xDD;
	
	HAL_UART_Transmit_DMA(&huart2 , RobotToBrainTimeBuffer, 18);
}



uint8_t amm;
int kkk,kk1;
extern uint8_t referee_Fps;
uint8_t lidar_station_flag;//2梯高，1环高，0基地前面
void RobotToBrain_Lidar(Brain_t* Brain)//发给雷达
{
//  x = referee2022.map_command_t.target_position_x * 100;
//	y = referee2022.map_command_t.target_position_y * 100;
	RobotToBrainChassisTimeBuffer[0]  = 0xAA;

	if(referee_Fps==0)
  {RobotToBrainChassisTimeBuffer[1]  = 1;        
	  RobotToBrainChassisTimeBuffer[2]  = 1;}
	else if(referee2022.game_status.game_progress == 4) 
	{
		RobotToBrainChassisTimeBuffer[1]  = referee2022.game_status.stage_remain_time & 0xff;        //referee2022.game_status.stage_remain_time
	 RobotToBrainChassisTimeBuffer[2]  = referee2022.game_status.stage_remain_time >> 8;
  }
	else 
	{
		 RobotToBrainChassisTimeBuffer[1]  = 0;        
	  RobotToBrainChassisTimeBuffer[2]  = 0;
	}
		if (rc_Ctrl_et.rc.s2==1)
		{	RobotToBrainChassisTimeBuffer[1]  = 1;        //referee2022.game_status.stage_remain_time
	 RobotToBrainChassisTimeBuffer[2]  = 1;}
  //if(referee2022.game_robot_status.robot_id==0x07)
 // {
  //  outpost_self=referee2022.game_robot_hp.red_outpost_HP;
	//	outpost_enemy=referee2022.game_robot_hp.blue_outpost_HP;
  //}
//	else if(referee2022.game_robot_status.robot_id==0x6b)
//{
 //   outpost_self=referee2022.game_robot_hp.blue_outpost_HP;
//		outpost_enemy=referee2022.game_robot_hp.red_outpost_HP;
 // }
//	if(referee2022.bullet_remaining.bullet_remaining_num>0&&referee2022.game_robot_status.mains_power_shooter_output==0)
//		shoot_flag=0;
//	else shoot_flag=1;
	//if(referee2022.buff.defence_buff>0)defense_flag=1;
//	else defense_flag=0;
	
//change_position=kkk;
	RobotToBrainChassisTimeBuffer[3]  = Brain->Lidar.change_position;

//Brain->Lidar.mode=kk1;
	//=Lidar_Fortress;
	//Brain->Lidar.mode=kk1;
	RobotToBrainChassisTimeBuffer[4]  = Brain->Lidar.mode ;
//	RobotToBrainChassisTimeBuffer[5]  = y&0xff;    
//	RobotToBrainChassisTimeBuffer[6]  = y>>8;	
//	RobotToBrainChassisTimeBuffer[7]  = outpost_self&0xff;    
//	RobotToBrainChassisTimeBuffer[8]  = outpost_self>>8;	
//	RobotToBrainChassisTimeBuffer[9]  = outpost_enemy&0xff;    
//	RobotToBrainChassisTimeBuffer[10]  = outpost_enemy>>8;
	RobotToBrainChassisTimeBuffer[11]  = referee2022.game_robot_status.remain_HP&0xff;    
	RobotToBrainChassisTimeBuffer[12]  = referee2022.game_robot_status.remain_HP>>8;
//	if (RobotToBrainChassisTimeBuffer[11]==0xDD) RobotToBrainChassisTimeBuffer[11]=0xDE;
	
	
//	RobotToBrainChassisTimeBuffer[13]  = referee2022.bullet_remaining.bullet_remaining_num&0xff;    
//	RobotToBrainChassisTimeBuffer[14]  = referee2022.bullet_remaining.bullet_remaining_num >> 8;	
//	RobotToBrainChassisTimeBuffer[15]  = shoot_flag;//referee2022.game_robot_status.mains_power_shooter_output;    
//	RobotToBrainChassisTimeBuffer[16]  = referee2022.bullet_remaining.money&0xff;	
//	RobotToBrainChassisTimeBuffer[17]  = referee2022.bullet_remaining.money >> 8;  
  //RobotToBrainChassisTimeBuffer[18]  = defense_flag;
//	RobotToBrainChassisTimeBuffer[19]  = lidar_station_flag;
//	RobotToBrainChassisTimeBuffer[20]  = lidar_mode;
	RobotToBrainChassisTimeBuffer[21]  = 0xDD;
	HAL_UART_Transmit_DMA(&huart5, RobotToBrainChassisTimeBuffer, 22);
}
uint8_t RobotToAllSee[10];
void RobotToBrain_All_See()
{
	RobotToAllSee[0]=0xAA;
  RobotToAllSee[1] = (referee2022.game_robot_status.robot_id > 10);
	RobotToAllSee[2]=0XDD;
		HAL_UART_Transmit_DMA(&huart4,RobotToAllSee, 3); 
}
/*下位机对上位机数据发送主任务*/

void RobotToBrain(Brain_t* Brain)
{
  RobotToBrain_All_See();
	
if(tim14.ClockTime%2== 0) {RobotToBrain_Lidar(Brain);}
	if(tim14.ClockTime%1== 0) {RobotToBrain_Autoaim(Holder.Yaw.GYRO_Angle,Brain);} 
	
}

extern int hurt_flag;

void Change_BrainMode(Brain_t* Brain)
{
	static int cnt_change,flag_Outpose;
if (referee_Fps==0&&referee2022.game_status.game_progress!=4)
{	
	if (referee2022.game_robot_status.remain_HP<150) Brain->Lidar.mode=Lidar_home;
	else if (rc_Ctrl_et.rc.s2==1) {Single_Mode(Brain);Brain->Lidar.mode=Lidar_Fortress;}
	else if (rc_Ctrl_et.rc.s2==3) Brain->Lidar.mode=Lidar_Patrol;
	else Brain->Lidar.mode=Lidar_Outpost;
	
if (Brain->Lidar.mode==Lidar_Outpost && Brain->Lidar.Arrive==1) {Brain->Autoaim.Mode=Outpost;}
	 else if (single!=Single_IDLE&&single!=Single_Exit)  Brain->Autoaim.Mode=Single;
	else 	 Brain->Autoaim.Mode=Autoaim;
//	if (referee2022.game_robot_status.remain_HP<150) Brain->Lidar.mode=Lidar_home;
//	else if (rc_Ctrl_et.rc.s2==1) {Brain->Lidar.mode=Lidar_Fortress;}
//	else if (rc_Ctrl_et.rc.s2==3) Brain->Lidar.mode=Lidar_Patrol;
//	else Brain->Lidar.mode=Lidar_Outpost;
	
//if (Brain->Lidar.mode==Lidar_Outpost && Brain->Lidar.Arrive==1) {Brain->Autoaim.Mode=Outpost;}
//	else 	 Brain->Autoaim.Mode=Autoaim;
}
 else if (referee2022.game_status.game_progress==4)
 {
//	if (((referee2022.game_robot_status.robot_id>10 && referee2022.game_robot_hp.red_outpost_HP>=800) ||(referee2022.game_robot_status.robot_id<10 && referee2022.game_robot_hp.blue_outpost_HP>=800))&&referee2022.game_status.stage_remain_time<=360) flag_Outpose=1;
	 
			if (referee2022.game_status.stage_remain_time<=380 &&referee2022.game_status.stage_remain_time>=378)single=Single_IDLE;//重置
	else if (referee2022.game_status.stage_remain_time<=300 &&referee2022.game_status.stage_remain_time>=298)single=Single_Exit;
	 if (referee2022.game_robot_status.remain_HP<=150) Brain->Lidar.mode=Lidar_home;
	else if (referee2022.buff.recovery_buff>=10&&referee2022.game_robot_status.remain_HP!=400) Brain->Lidar.mode=Lidar_home;
	else 
	{
		Single_Mode(Brain);
		if (single!=Single_Exit)  {Brain->Lidar.mode=Lidar_Patrol;Brain->Lidar.change_position=4;}
	else if (((referee2022.game_robot_status.robot_id>10 && referee2022.game_robot_hp.red_outpost_HP>0) ||(referee2022.game_robot_status.robot_id<10 && referee2022.game_robot_hp.blue_outpost_HP>0))) Brain->Lidar.mode=Lidar_Outpost;
	else {Brain->Lidar.mode=Lidar_Patrol;if (Brain->Lidar.change_position==4) Brain->Lidar.change_position=5;}


	}

if (Brain->Lidar.mode==Lidar_Outpost && Brain->Lidar.Arrive==1) {Brain->Autoaim.Mode=Outpost;}
if (single!=Single_IDLE&&single!=Single_Exit) {Brain->Autoaim.Mode=Single;}	 
else Brain->Autoaim.Mode=Autoaim;
 }
 if (referee2022.game_status.game_progress!=4 ||referee2022.game_status.stage_remain_time<=240) flag_Outpose=0;
 
}
void Armor_Ignore(Brain_t* brain)
{
	brain->Autoaim.Ignore_armorNumber=0;
	
	if (brain->Autoaim.Mode==Autoaim)brain->Autoaim.Ignore_armorNumber|=0x20;//忽略前哨站
	else brain->Autoaim.Ignore_armorNumber=0xDF;
	brain->Autoaim.Ignore_armorNumber|=0x80;//忽略基地
//	if (brain->Autoaim.Attack_state.shoot_num>=50 && referee2022.game_robot_hp.blue_robot_Hurt[brain->Autoaim.Attack_state.]=0;)
	if (referee2022.game_status.stage_remain_time>=360) brain->Autoaim.Ignore_armorNumber|=0x02;
	
	if (referee2022.game_robot_status.robot_id>10)
	{
		if (referee2022.game_robot_hp.red_robot_revge[1]==2) brain->Autoaim.Ignore_armorNumber|=0x01;
	 if (referee2022.game_robot_hp.red_robot_revge[2]==2) brain->Autoaim.Ignore_armorNumber|=0x02;
	 if (referee2022.game_robot_hp.red_robot_revge[3]==2) brain->Autoaim.Ignore_armorNumber|=0x04;
	 if (referee2022.game_robot_hp.red_robot_revge[4]==2) brain->Autoaim.Ignore_armorNumber|=0x08;
	 if (referee2022.game_robot_hp.red_robot_revge[7]==2) brain->Autoaim.Ignore_armorNumber|=0x10;
	}
else 
	{
		if (referee2022.game_robot_hp.blue_robot_revge[1]==2) brain->Autoaim.Ignore_armorNumber|=0x01;
	 if (referee2022.game_robot_hp.blue_robot_revge[2]==2) brain->Autoaim.Ignore_armorNumber|=0x02;
	 if (referee2022.game_robot_hp.blue_robot_revge[3]==2) brain->Autoaim.Ignore_armorNumber|=0x04;
	 if (referee2022.game_robot_hp.blue_robot_revge[4]==2) brain->Autoaim.Ignore_armorNumber|=0x08;
	 if (referee2022.game_robot_hp.blue_robot_revge[7]==2) brain->Autoaim.Ignore_armorNumber|=0x10;
	}
}	


