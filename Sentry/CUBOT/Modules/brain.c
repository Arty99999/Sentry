#include "brain.h"
#include "hardware_config.h"

#include "control_logic.h"
#include "check.h"
#include "filter.h"

#include "string.h"
#include "ins.h"
#include "all_chassis.h"


uint16_t time;
uint8_t time_flag;
uint16_t time2;
uint8_t sentry_attack;//攻击烧饼标志位
uint8_t lidar_mode;
uint8_t sentry_attack1;//攻击烧饼标志（用于调试）
float YawDeflectionAngle_test; //未使用


float pitch_comp_angle;  //未使用
float brain_sense_pitch=1.0;  //pitch视觉灵敏度，用于帧率匹配
float brain_sense_yaw=1.0;  //yaw...
float angle_to_lidar;  //雷达建图坐标系到世界坐标系的角度，用于转换坐标系
uint16_t shoot_number;  //暂时没用
uint8_t brain_camara_num; //识别到装甲板的相机的ID号
uint8_t brain_flag; //视觉标志位，区分巡航与击打
int M111;

float brain_sense_add=0; //没用
uint8_t fire_flag; //电控开火标志位，误差角小于某值时开火
uint32_t brain_time; //大yaw轴旋转分频（没用）

uint8_t Brain_recData[Brain_rxBufferLengh];
uint8_t Lidar_recData[Lidar_rxBufferLengh];


uint8_t RobotToBrainQuestBuffer[5] ;
uint8_t RobotToBrainCMDBuffer[3];                   //命令数据
uint8_t RobotToBrainLogBuffer[20];                  //日志数据
uint8_t RobotToBrainTimeBuffer[50];
uint8_t RobotToBrainChassisTimeBuffer[22];
uint8_t RobotToBrainQuestBuffer_XinTiaoBao[3] ;     //请求数据
uint8_t RobotToBrainQuestBuffer_WorkingModel[5] ;   //请求数据
uint8_t RobotToBrainQuestBuffer_Velocity[5] ; 



Brain_t Brain={.Lidar.mode=Lidar_Outpost};

void Brain_DataUnpack(Brain_t* brain, uint8_t* recBuffer);

/**
  * @brief   串口2视觉回调函数 
  * @param[in]  
  */
uint16_t aff;
uint8_t Brain_Autoaim_callback(uint8_t * recBuffer, uint16_t len)
{
	check_robot_state.Check_Usart.Check_vision_cnt=0;
	tim14_FPS.Vision_cnt++;
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
void Brain_Camera_DataUnpack_New(Brain_t* Brain ,uint8_t * recBuffer)//解包雷达数据
{
	uint8_t k=0;
	if(recBuffer[0]==0xAA  &&recBuffer[1]<5)
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
	if(recBuffer[0] == 0xAB)
	{
		Brain->Autoaim.Brain_Data.FrameType=recBuffer[1];
		Brain->Autoaim.Brain_Data.FrameCoreID = recBuffer[2];
		
		if((Brain->Autoaim.Brain_Data.FrameType == BRAIN_TO_ROBOT_CMD) && recBuffer[13] == 0xDD)  //< 解算偏转角
		{
			Brain->Autoaim.mode_cnt[Cruise] = 0;
			Brain->Autoaim.Use_Can_angle=Brain->Autoaim.Send_Can_angle[recBuffer[12]];
			Brain->Autoaim.Use_Gyro_angle=Brain->Autoaim.Send_Gyro_angle[recBuffer[12]];
			
			Brain->Autoaim.mode=Lock;
			
      Brain->Autoaim.Yaw_add = ((recBuffer[3] >> 6) == 0 ? 1 : -1) * ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
      Brain->Autoaim.Pitch_add =((recBuffer[5] >> 6) == 0 ? 1 : -1)* ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100);
			

			Brain->Autoaim.Distance = (float)(recBuffer[7])/10;
      Brain->Autoaim.IsFire = ((float)(recBuffer[8]));
			Brain->Autoaim.camara_num=recBuffer[11];
			
//			Brain->All_See.armorNumber[0] = Brain->Autoaim.camara_num;
//			Brain->All_See.Distance[0]=Brain->Autoaim.Distance*100;
			
			
			
      Brain->Autoaim.fire_flag=0;
			if (rc_Ctrl_et.rc.s2==2)
			{ if (fabs(Holder.Yaw1.Target_Angle-Holder.Yaw1.Can_Angle)<0.4) Brain->Autoaim.fire_flag=1;else Brain->Autoaim.fire_flag=0;	
		   	  Holder.Yaw1.Target_Angle=Brain->Autoaim.Use_Can_angle+Brain->Autoaim.Yaw_add;
					Holder.Pitch.Target_Angle= Brain->Autoaim.Use_Gyro_angle+Brain->Autoaim.Pitch_add ;}

			

			}				
		}
		else if((Brain->Autoaim.Brain_Data.FrameType == BRAIN_TO_ROBOT_HINT) && recBuffer[6] == 0xDD)  //< 解算brain状态
		{
		
			
		}
	}
/**
  * @brief  下位机向上位机发送时间戳以及四元数
  */
int MLL;
void RobotToBrain_Autoaim(float yaw,Brain_t* brain)//发给自瞄
{
	int16_t tmp0,tmp1,tmp2,tmp3,tmp4,cnt;
	
	ThisSecond++;
	tmp0 = (int16_t)(INS_attitude->q[0] *  30000);
	tmp1 = -(int16_t)(INS_attitude->q[1] *  30000);
	tmp2 = -(int16_t)(INS_attitude->q[2] *  30000);
	tmp3 = (int16_t)(INS_attitude->q[3] *  30000);
	tmp4 = (int16_t)(yaw *3.1701f* 100) % 36000;
	cnt = (yaw*3.1701f* 100)/360;
	
	if(tmp4 > 18000)tmp4 = tmp4 - 36000;else if(tmp4 < -18000) tmp4 = 36000 + tmp4;
									
	RobotToBrainTimeBuffer[0]  = 0xAB;
	RobotToBrainTimeBuffer[1]  = 0x07;                      //Type ;  //固定为0x07
	RobotToBrainTimeBuffer[2]  = 0x01;                      //coreID;  //目前固定为0x01
	RobotToBrainTimeBuffer[3]  = (ThisSecond >> 8) ;        //索引，int16_t型
	RobotToBrainTimeBuffer[4]  = (ThisSecond &0xff);
	
	RobotToBrainTimeBuffer[5]  = ( tim14.ClockTime >>24);    //定时器时间，int32_t型
	RobotToBrainTimeBuffer[6]  = ((tim14.ClockTime >>16)&0xff);
	RobotToBrainTimeBuffer[7]  = ((tim14.ClockTime >>8)&0xff);
	RobotToBrainTimeBuffer[8]  = ((tim14.ClockTime &0xff));
			if (referee2022.game_robot_status.robot_id>10) RobotToBrainTimeBuffer[9]  =0;
else RobotToBrainTimeBuffer[9]  =1;//0 识别红方  1识别蓝方
            
	RobotToBrainTimeBuffer[10] = tmp0 & 0xFF;                   //四元数q0，float型
	RobotToBrainTimeBuffer[11] = tmp0 >> 8;
	RobotToBrainTimeBuffer[12] = tmp1 & 0xFF;
	RobotToBrainTimeBuffer[13] = tmp1 >> 8;  
	RobotToBrainTimeBuffer[14] = tmp2 & 0xFF;                   //四元数q1，float型
	RobotToBrainTimeBuffer[15] = tmp2 >> 8;
	RobotToBrainTimeBuffer[16] = tmp3 & 0xFF;
	RobotToBrainTimeBuffer[17] = tmp3 >> 8;  
	RobotToBrainTimeBuffer[18] = tmp4 & 0xFF;
	RobotToBrainTimeBuffer[19] = tmp4 >> 8; 

//brain->Autoaim.Ignore_armorNumber=0;

	RobotToBrainTimeBuffer[20] = brain->Autoaim.Mode;//1 是前哨站 0是普通
//	brain->Autoaim.Ignore_armorNumber|=0x20;
	//brain->Autoaim.Ignore_armorNumber|=0x04;
	if (referee2022.game_robot_hp.blue_robot_revge[1]==2) brain->Autoaim.Ignore_armorNumber|=0x02;
	 if (referee2022.game_robot_hp.blue_robot_revge[2]==2) brain->Autoaim.Ignore_armorNumber|=0x04;
	 if (referee2022.game_robot_hp.blue_robot_revge[3]==2) brain->Autoaim.Ignore_armorNumber|=0x08;
	 if (referee2022.game_robot_hp.blue_robot_revge[4]==2) brain->Autoaim.Ignore_armorNumber|=0x10;
RobotToBrainTimeBuffer[21] = brain->Autoaim.Ignore_armorNumber;//忽略装甲板


RobotToBrainTimeBuffer[22] = 0xDD;
RobotToBrainTimeBuffer[23] = 0xDD;

brain->Autoaim.Send_Can_angle[RobotToBrainTimeBuffer[8]]=Holder.Yaw1.Can_Angle;
brain->Autoaim.Send_Gyro_angle[RobotToBrainTimeBuffer[8]]=Holder.Pitch.GYRO_Angle;
	HAL_UART_Transmit_DMA(&huart2 , RobotToBrainTimeBuffer, 24);
	
}



int change_position; 
uint8_t amm;
uint8_t lidar_station_flag;//2梯高，1环高，0基地前面
void RobotToBrain_Lidar(Brain_t* Brain)//发给雷达
{
//  x = referee2022.map_command_t.target_position_x * 100;
//	y = referee2022.map_command_t.target_position_y * 100;
	RobotToBrainChassisTimeBuffer[0]  = 0xAA;

	if (referee2022.game_status.game_progress==0&&referee2022.game_status.game_type==0)
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
	

	RobotToBrainChassisTimeBuffer[3]  = change_position;


	//=Lidar_Fortress;
	
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
	RobotToBrainChassisTimeBuffer[19]  = lidar_station_flag;
	RobotToBrainChassisTimeBuffer[20]  = lidar_mode;
	RobotToBrainChassisTimeBuffer[21]  = 0xDD;
	HAL_UART_Transmit_DMA(&huart5, RobotToBrainChassisTimeBuffer, 22);
}
uint8_t A[3]={0xAA,0x00,0xDD};
void RobotToBrain_All_See()
{
		if (referee2022.game_robot_status.robot_id>10) A[1]=1;
else A[1]=0;
		HAL_UART_Transmit_DMA(&huart4,A, 3); 
}
/*下位机对上位机数据发送主任务*/

void RobotToBrain(Brain_t* Brain)
{
  RobotToBrain_All_See();
	
if(tim14.ClockTime%2== 0) {RobotToBrain_Lidar(Brain);}
	if(tim14.ClockTime%1== 0) {RobotToBrain_Autoaim(Holder.Yaw.GYRO_Angle,Brain);} 
	
}
extern uint8_t referee_Fps;
extern int hurt_flag;
void Change_BrainMode(Brain_t* Brain)
{
		if (Brain->Lidar.mode==Lidar_Outpost && Brain->Lidar.Arrive==1&&hurt_flag==0 &&bullet_num_17mm<=300&&(referee2022.game_status.game_progress==4||referee_Fps==0)) Brain->Autoaim.Mode=Outpost;
else Brain->Autoaim.Mode=Autoaim;
		
	if (referee2022.game_robot_status.remain_HP>=400) {
		Brain->Lidar.mode=Lidar_Outpost;
	if (referee2022.game_status.game_progress==4&&referee2022.game_status.game_type==1)
	{if (referee2022.game_robot_status.robot_id>10 && referee2022.game_robot_hp.red_outpost_HP==0) {Brain->Autoaim.Mode=Autoaim;Brain->Lidar.mode=Lidar_Patrol;}
	else if (referee2022.game_robot_status.robot_id<10 && referee2022.game_robot_hp.blue_outpost_HP==0) {Brain->Autoaim.Mode=Autoaim;Brain->Lidar.mode=Lidar_Patrol;}
 }}
	if (referee2022.game_robot_status.remain_HP<150||(referee2022.bullet_remaining.bullet_remaining_num<=50&&referee2022.game_status.game_progress==4)) Brain->Lidar.mode=Lidar_home;
	if (rc_Ctrl_et.rc.s2==1)Brain->Lidar.mode=Lidar_Fortress;
	//Brain->Autoaim.Mode=Outpost;
//	if (Brain->Autoaim.Mode==c) cnt_Outpost++;
	//if (brain->Autoaim.Mode==Autoaim)

}