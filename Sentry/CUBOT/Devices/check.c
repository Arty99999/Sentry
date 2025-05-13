#include "check.h"
#include "referee.h"
#include "dr16.h"
#include "hardware_config.h"
#include <stdlib.h>
#include "stb_ds.h"
Check_Robot_State check_robot_state={
.Check_Usart.Check_receiver_cnt=300
};
FPS tim14_FPS={
.Receiver_cnt= 0,
.Referee_cnt = 0,
.Vision_cnt = 0,
.Gyro_In_FPS= 0,
.Gyro_Out_FPS=0,
.Lidar_cnt=0
};

/**
  *读取机器人状态，各部分是否在线，
	*/
	
void RobotOnlineState(Check_Robot_State *CheckRobotState,RC_Ctrl_ET *rc_ctrl)
{
	Motor* motor = NULL;
	list_t *node = NULL;	
	
	CheckRobotState->Check_Usart.Check_receiver_cnt++;
	CheckRobotState->Check_Usart.Check_vision_cnt++;
	CheckRobotState->Check_Usart.Check_referee_cnt++;
	CheckRobotState->Check_Usart.Check_lidar_cnt++;
		
	for (node = can2.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (can2.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
			motor->Data.Online_check.StatusCnt++;
		if (motor->Data.Online_check.StatusCnt>=30) {motor->Data.Online_check.StatusCnt=30;motor->Data.Online_check.Status=0;}
			else motor->Data.Online_check.Status=1;
	}
	for (node = can1.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (can1.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
			motor->Data.Online_check.StatusCnt++;
		if (motor->Data.Online_check.StatusCnt>=30) {motor->Data.Online_check.StatusCnt=30;motor->Data.Online_check.Status=0;}
			else motor->Data.Online_check.Status=1;
	}

//	Motor_Check(&CheckRobotState->Check_Can1,can1) ;
//  Motor_Check(&CheckRobotState->Check_Can2,can2) ;
	
  CheckRobotState->Check_Usart.Check_vision_cnt=int16_constrain(CheckRobotState->Check_Usart.Check_vision_cnt,0,200);
  CheckRobotState->Check_Usart.Check_vision = (CheckRobotState->Check_Usart.Check_vision_cnt > 100) ? 0 : 1;
	
  CheckRobotState->Check_Usart.Check_lidar_cnt=int16_constrain(CheckRobotState->Check_Usart.Check_lidar_cnt,0,200);
  CheckRobotState->Check_Usart.Check_lidar = (CheckRobotState->Check_Usart.Check_lidar_cnt > 100) ? 0 : 1;
	
	CheckRobotState->Check_Usart.Check_referee_cnt= int16_constrain(CheckRobotState->Check_Usart.Check_referee_cnt,0,200);
  CheckRobotState->Check_Usart.Check_referee = (CheckRobotState->Check_Usart.Check_referee_cnt > 100) ? 0 : 1;
	
//	  
	CheckRobotState->Check_Usart.Check_receiver_cnt=int16_constrain(CheckRobotState->Check_Usart.Check_receiver_cnt,0,200);
	if(CheckRobotState->Check_Usart.Check_receiver_cnt>100)  {rc_ctrl->isOnline = 0 ;CheckRobotState->Check_Usart.Check_receiver=0;} else { if (rc_Ctrl_et.rc.sA==1&&rc_Ctrl_et.rc.ch2!=1)rc_ctrl->isOnline = 1;else rc_ctrl->isOnline = 0;CheckRobotState->Check_Usart.Check_receiver=1;}
		
}

/*各模块帧率检测*/
void FPS_Check(FPS * fps)
{
		fps->Receiver_FPS=fps->Receiver_cnt;
		fps->Referee_FPS=fps->Referee_cnt;
		fps->Vision_FPS=fps->Vision_cnt;
		fps->Lidar_FPS= fps->Lidar_cnt;
	  fps->Gyro_Out_FPS=fps->Gyro_Out_cnt;
	  fps->Camera_FPS=fps->Camera_cnt;
	  fps->Lidar_cnt=0;
		fps->Receiver_cnt=0;
		fps->Referee_cnt=0;
		fps->Vision_cnt=0;
		fps->Gyro_Out_cnt= 0;
	  fps->Camera_cnt=0;
	Motor_CheckFPS();
}

void  Motor_Check(Check_Motor* check,CAN_Object canx) 
{
	Motor* motor = NULL;
	list_t *node = NULL;		
  uint8_t j,i,count,size;

	if (check->Offline!= NULL) {
    arrfree(check->Offline);
	check->Offline=NULL;
	}

	if (check->Online != NULL) {
    arrfree(check->Online);
	check->Online=NULL;
	}

	for (node = canx.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (canx.DevicesList.prev->next);
			 node = node->next)
	{
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
		if (motor->Data.Online_check.Status== 1)
    arrput(check->Online,motor->Param.CanId);
		else arrput(check->Offline,motor->Param.CanId);

	}
		check->size_Offline=arrlen(check->Offline) ;
		check->size_Online=arrlen(check->Online) ;

}
void Motor_CheckFPS()
{
	Motor* motor = NULL;
	list_t *node = NULL;		
for (node = can1.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (can1.DevicesList.prev->next);
			 node = node->next)
	{	
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
    motor->Data.Online_check.FPS=motor->Data.Online_check.Cnt;
		motor->Data.Online_check.Cnt=0;
	}
for (node = can2.DevicesList.next;    		//< 对循环链表遍历一圈
			 node != (can2.DevicesList.prev->next);
			 node = node->next)
	{	
		motor = list_entry(node, Motor, list);  //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
    motor->Data.Online_check.FPS=motor->Data.Online_check.Cnt;
		motor->Data.Online_check.Cnt=0;
	}
	
}