//#include "vision.h"
//#include "referee.h"
//#include "kalman.h"
////#include "remote.h"
////#include "referee_2021.h"
////#include "usart.h"
//////#include "shoot.h"
//#include "init_bmi088.h"
//#include "hardware_config.h"
////#include "kalman.h"

////#include "cubot_Communication.h"
////#include "cubot_motors.h"
////#include "cubot_algorithm.h"
////#include "cubot_applications.h"
//#define myABS(x) ( (x)>0?(x):-(x) )
////uint8_t Vision_to_NUC[6]__attribute__((at(0x24016000)));
////uint8_t Vision_meta[6]__attribute__((at(0x24018000)));//nuc缓存数组
//float K_YAW;
//uint8_t Hit_B_Gap,Hit_B_Gap_Limit;
///*
//视觉协议
//7位 第0位 0xaa 帧头
//		第5位 自瞄，打符使能位
//    第6位 0xdd 帧尾
//*/

///**
//  * @brief  接收NUC计算结果，云台跟随，自瞄控制
//  */
//uint8_t Rune_AutoHit = 0;
//float Yaw_K = 1.0f,Pitch_K = 0.9f;
//void Run_User_Vision_Task(RC_Ctrl* rc_ctrl,Holder_t* holder ,Trace* Info_Vision)
//{
////	for(int j = 0;j<12;j++)
//  {
//		int j = 0;
//    if(Vision_meta[j]==0xaa&&Vision_meta[j+5]==0xdd)           
////		if((Vision_meta[0]==0xaa)&&(Vision_meta[5]==0xdd))
//		{			 
//		  Info_Vision->X_Error=0;
//			Info_Vision->Y_Error=0;
////			Vision_Info.Vision_Eable=0;
//			Info_Vision->Position_X_raw=(int16_t)(Vision_meta[j+1]<<8|Vision_meta[j+2]);      //接收NUC计算结果
//			Info_Vision->Position_Y_raw=(int16_t)(Vision_meta[j+3]<<8|Vision_meta[j+4]);						
//                
//			Info_Vision->X_Error=Info_Vision->Position_X_raw/100;                             //计算误差值
//			Info_Vision->Y_Error=Info_Vision->Position_Y_raw/100;		
//			Info_Vision->Kalman_X_Error=KalmanFilter_yaw(Info_Vision->X_Error,VISION_KALMAN_Q,VISION_KALMAN_R);       //卡尔曼滤波
//			Info_Vision->Kalman_Y_Error=KalmanFilter_pitch(Info_Vision->Y_Error,VISION_KALMAN_Q,VISION_KALMAN_R);
////		  Vision_Info.
//			Info_Vision->Vision_Eable=0;
//			 if(rc_ctrl->key_B_flag==1||rc_ctrl->rc.s2==1) //key_B_flag                //按键控制自瞄模式选择
//			 { 
//				 Info_Vision->Hit_Mode=1;
//			 }
//			 else if(rc_ctrl->mouse.press_r_flag==1||rc_ctrl->rc.s2==2) //.mouse.press_r_flag
//			 {
//				 Info_Vision->Hit_Mode=2;
//			 }
//			 
//			  if(rc_ctrl->mouse.press_r_flag!=1&&rc_ctrl->rc.s2!=2)//||RC_Ctl.mouse.press_r_flag!=1&&RC_Ctl.rc.s2!=1
//			 {	
//        if(Info_Vision->Hit_Mode!=1)		
//				{					
//          Info_Vision->Hit_Mode=0;				 
////			  RC_Holder_Angel_Deal(); 
//				}		
//			}
//			  if(rc_ctrl->key_B_flag!=1&&rc_ctrl->rc.s2!=1)//||RC_Ctl.mouse.press_r_flag!=1&&RC_Ctl.rc.s2!=1
//			 {	
//        if(Info_Vision->Hit_Mode!=2)		
//				{					
//          Info_Vision->Hit_Mode=0;				 
////			  RC_Holder_Angel_Deal(); 
//				}						
//			 }
//		 if(myABS(Info_Vision->X_Error)<20&&myABS(Info_Vision->Y_Error)<20)//过滤可能错误的数据 &&(RC_Ctl.mouse.press_r_flag==1||RC_Ctl.rc.s2==1)
//		 { 		
//			 if(Info_Vision->Hit_Mode==2)            //自瞄模式  装甲板自瞄 			 
////			 if(Vision_Info.Hit_Mode==2)           //自瞄模式  装甲板自瞄 
//			 { 
//				 holder->Yaw.Target_Angle=holder->Yaw.Bmi088_Angle+Info_Vision->X_Error*Yaw_K;		 
//				 holder->Pitch.Target_Angle=holder->Motors6020.motor[1].Data.Angle - Info_Vision->Y_Error*Pitch_K;

//			if(holder->Pitch.Target_Angle <= -45.0f)            //限幅
//				 holder->Pitch.Target_Angle = -45.0f;        
//		  else if(Holder.Pitch.Target_Angle >= 28.0f)
//				 Holder.Pitch.Target_Angle = 28.0f;
//				 
//				 //自瞄辅助发弹 			
//				 if(myABS(Info_Vision->X_Error)<1.0f&&myABS(Info_Vision->Y_Error)<1.0f&&myABS(Info_Vision->X_Error)>0&&myABS(Info_Vision->Y_Error)>0)
//				{
//				 Info_Vision->Flag_Auto_Enable=1;	
//				}
//				else
//				{
//		  	 Info_Vision->Flag_Auto_Enable=0;
//				}
//			 }	
////		   else if(Vision_Info.Hit_Mode==2)   //自动发弹 是打符模式
//			 else if(Info_Vision->Hit_Mode==1)   //自动发弹 是打符模式
//			 {

////				 Holder.Yaw.Target_Angle=Holder.Yaw.Bmi088_Angle + Vision_Info.X_Error+Holder.Yaw.Bmi088_Angle_speed*K_YAW;00
//				holder->Yaw.Target_Angle=holder->Yaw.Bmi088_Angle + Info_Vision->X_Error;				 
//				holder->Pitch.Target_Angle=holder->Motors6020.motor[1].Data.Angle + Info_Vision->Y_Error;
//			 
//				 //自瞄辅助发弹 
//				if(myABS(Info_Vision->X_Error)<2.0f&&myABS(Info_Vision->Y_Error)<2.0f)//瞄准才允许发射  条件可适当放宽
//			  { 		
//					Hit_B_Gap++;
//					
////					if(game_robot_state.shooter_id1_17mm_speed_limit==15) Hit_B_Gap_Limit=20;//与射速有关
////					else if(game_robot_state.shooter_id1_17mm_speed_limit==18) Hit_B_Gap_Limit=20;
////						else if(game_robot_state.shooter_id1_17mm_speed_limit==22) Hit_B_Gap_Limit=20;
////							else if(game_robot_state.shooter_id1_17mm_speed_limit==30) Hit_B_Gap_Limit=20;
//					Hit_B_Gap_Limit = 28;
//				  if(Hit_B_Gap>Hit_B_Gap_Limit)	
//				  {
//					 Rune_AutoHit = 1;
//				   Info_Vision->Flag_Shoot_B_Enable=1;
//				   Hit_B_Gap=0;
//				  }					
//				  else				
//				  {
//						Rune_AutoHit = 0;
//				   Info_Vision->Flag_Shoot_B_Enable=0;	
//				  }
//			  } 
//			  else				
//				{
//					 Rune_AutoHit = 0;
//				  Info_Vision->Flag_Shoot_B_Enable=0;	
//				}
//			  
//			 }
//			 else
//			 {
//				 Rune_AutoHit = 0;
//				 Info_Vision->Flag_Auto_Enable=0;
//				 Info_Vision->Flag_Shoot_B_Enable=0;	
//			 }
//			 
//		 }
//  	 else
//		 {
//					Info_Vision->X_Error=0;
//					Info_Vision->Y_Error=0;		
//	   }
//	}
//   else
//	 {
//					Info_Vision->X_Error=0;
//					Info_Vision->Y_Error=0;		
//	 }
////		break;
// }	 
//			 
//}


///**
//  * @brief  发送指令给NUC
//  */
//void Send_Command_NUC(uint8_t Vision_mode,uint8_t data,Trace* Info_Vision)
//{
////	Vision_Info.Predict_Flag = 1;
//	int16_t temp=0;
//	temp=Holder.Yaw.Bmi088_Angle_speed*Gyro_Gr*57.3;
//	if(temp>127)temp=127;
//	else if(temp<-127) temp=-127;
//	Vision_to_NUC[0]=0xaa;	
//	Vision_to_NUC[1]=Vision_mode;//1 自瞄 2 打符
//	Vision_to_NUC[2]=(int8_t)temp;
//	Vision_to_NUC[3]=Info_Vision->Predict_Flag;  //
//	Vision_to_NUC[4]=(uint8_t)referee2022.game_robot_status.shooter_id1_17mm_speed_limit;
////	Vision_to_NUC[4]=0x01;
//	Vision_to_NUC[5]=0xdd;
////	DMA1_Stream3->M0AR =(uint32_t)(Vision_to_NUC);
////	DMA1_Stream3->NDTR=5;
//	HAL_UART_Transmit_DMA(&huart2,Vision_to_NUC,6);
//}



