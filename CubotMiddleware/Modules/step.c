#include "step.h"
#include "hardware_config.h"

/*下面执行上台阶步骤的if判断估计有不少冗余，再优化吧*/


void stepinit(Step_t * step,BasePID_Object step_start_pid,BasePID_Object step_push_pid,BasePID_Object step_reset_pid,CanNumber canx)
{
	MotorInit(&step->motor[0], 1856 , Motor3508, canx, 0x207);
	MotorInit(&step->motor[1], 718 , Motor3508, canx, 0x204);
	BasePID_Init(&step->StartPID[0],step_start_pid.Kp,step_start_pid.Ki,step_start_pid.Kd,step_start_pid.KiPartDetachment);
	BasePID_Init(&step->PushPID[0],step_push_pid.Kp,step_push_pid.Ki,step_push_pid.Kd,step_push_pid.KiPartDetachment);
	BasePID_Init(&step->ResetPID[0],step_reset_pid.Kp,step_reset_pid.Ki,step_reset_pid.Kd,step_reset_pid.KiPartDetachment);
	step->StepFlag = 0;
	step->Start_speed = 1000;

}

void StepGetData_Single(Step_t * step,RC_Ctrl* rc_ctrl)
{
	if(step->StepFlag == 0)
	{
		step->Step1FinishFlag = 0;
		step->Step2FinishFlag = 0;
	}
	else if(step->Step1FinishFlag == 0)
	{
		step->Step2FinishFlag = 0;
	}
	
	if(tim14.Step_Time == 500)	//读取初始角度
	{
		step->first_angle[0]=step->motor[0].Data.Angle;
	}
	
	StepAnglePlus(&step->motor[0],&step->angle[0],&step->last_angle[0]);
	
	if((step->Step1FinishFlag == 1 && tim14.Step_Time > 500) || step->Step2_cnt !=0)
	{
//		if(tim14.Step_Time > 1500 && (step->motor[0].Data.SpeedRPM < 100 || step->Step1_StartFinishFlag == 1))
//		{
//			step->Step1_StartFinishFlag = 1;
//			step->target_angle[0] = 180*71 + step->first_angle[0] / 3500;
//			step->angle_out[0] = Step_Pid_Push(&step->PushPID[0],step->angle[0],step->target_angle[0]);
//		}
//		else if(step->Step1_StartFinishFlag == 0)
//		{
//			step->angle_out[0] = Step_Pid_Start(&step->StartPID[0],step->motor[0].Data.SpeedRPM,step->Start_speed);
//		}
		step->Step2_cnt++;
		
		if(step->Step2_cnt > 400 && (step->motor[0].Data.SpeedRPM < 100 || step->Step1_StartFinishFlag == 1))
		{
			step->Step1_StartFinishFlag = 1;
			step->target_angle[0] = 180*71 + step->first_angle[0];
			step->angle_out[0] = Step_Pid_Push(&step->PushPID[0],step->angle[0],step->target_angle[0]);
		}
		else if(step->Step1_StartFinishFlag == 0)
		{
			step->angle_out[0] = Step_Pid_Start(&step->StartPID[0],step->motor[0].Data.SpeedRPM,step->Start_speed);
		}
		
		if(step->Step2_cnt == 1500)
		{
			step->Step2_cnt = 0;
			tim14.Step_Time = 501;
			step->Step1_StartFinishFlag = 0;
		}
		
	}
	else if(step->Step1FinishFlag == 0 && tim14.Step_Time > 500)		//时间轴停止
	{
		tim14.Step_Time = 501;
		if(rc_Ctrl.isOnline == 1)
		{
			step->angle_out[0] = Step_Reset_Pid(&step->ResetPID[0],step->angle[0],step->first_angle[0]+10*71);
		}
	}

	
	if(step->Step2FinishFlag == 1 && tim14.Step_Time >= 3000)
	{
		step->Step1_StartFinishFlag = 0;
		step->angle_out[0] = Step_Reset_Pid(&step->ResetPID[0],step->angle[0],step->first_angle[0]+10*71);
		step->StepFlag = 0;
	}

	if(rc_Ctrl.isOnline == 1)
	{
		MotorFillData(&step->motor[1],step->angle_out[1]);
		MotorFillData(&step->motor[0],step->angle_out[0]);
//			MotorFillData(&step->motor[1],0);
//			MotorFillData(&step->motor[0],0);
	}
	else
	{
		MotorFillData(&step->motor[1],0);
		MotorFillData(&step->motor[0],0);
	}
}


//void StepGetData_Double(Step_t * step,RC_Ctrl* rc_ctrl)
//{
//	if(tim14.Step_Time == 500)	//读取初始角度
//	{
//		step->first_angle[0]=step->motor[0].Data.Angle;
//		step->first_angle[1]=step->motor[1].Data.Angle;
//	}

//	for(int i=0;i<2;i++)		//减速箱角度累加
//	{
//		StepAnglePlus(&step->motor[i],&step->angle[i],&step->last_angle[i]);
//	}
//	
//	if(step->StepFlag == 0 && tim14.Step_Time > 500)		//时间轴停止
//	{
//		tim14.Step_Time = 501;
//		if(rc_Ctrl.isOnline == 1)
//		{
//			step->angle_out[0] = Step_Reset_Pid(&step->ResetPID[0],step->angle[0],step->first_angle[0]+10*19);
//			step->angle_out[1] = Step_Reset_Pid(&step->ResetPID[1],step->angle[1],step->first_angle[1]+10*27);
//		}
//	}
//	else if(step->StepFlag == 1 && tim14.Step_Time > 500)
//	{
//		if(tim14.Step_Time>501&&tim14.Step_Time<3000)//缓启动
//		{
//			step->target_angle[1] = ((float)tim14.Step_Time-500)*(60*27+step->first_angle[1])/2500;
//			step->angle_out[1] = Step_Pid(&step->RunPID[1],step->angle[1],step->target_angle[1]);
//			step->angle_out[0] = Step_Reset_Pid(&step->ResetPID[0],step->angle[0],step->first_angle[0]+10*19);

//		}
//		else
//		{
//			step->target_angle[1] = 160*27+step->first_angle[1];
//			step->angle_out[1] = Step_Pid(&step->RunPID[1],step->angle[1],step->target_angle[1]);
//			step->angle_out[0] = Step_Reset_Pid(&step->ResetPID[0],step->angle[0],step->first_angle[0]+10*19);

//		}
//		
//		
//		if((step->RunPID[1].Error <= 100 && step->RunPID[1].Error >= -100) && tim14.Step_Time >= 3000 && step->Step1FinishFlag == 1 )
//		{
//			//前撑起动作完成,车尾结构开始撑起
//			if(tim14.Step_Time < 5500)//缓启动
//			{
//				step->target_angle[0] = ((float)tim14.Step_Time-3000)*(220*19+step->first_angle[0])/2500;
//				step->angle_out[0] = Step_Pid(&step->RunPID[0],step->angle[0],step->target_angle[0]);
//				step->angle_out[1] = Step_Pid(&step->RunPID[1],step->angle[1],step->target_angle[1]);
//			}
//			else
//			{
//				step->target_angle[0] = 160*19+step->first_angle[0];
//				step->angle_out[0] = Step_Pid(&step->RunPID[0],step->angle[0],step->target_angle[0]);
//				step->angle_out[1] = Step_Pid(&step->RunPID[1],step->angle[1],step->target_angle[1]);

//			}
//		}
//		else if(tim14.Step_Time >= 3000)
//		{
//			//前撑起动作未完成，其判定还应该加上底盘陀螺仪角度
//			tim14.Step_Time = 3000;
//		}
//		
//		//后撑起动作未完成
//		if((step->RunPID[0].Error >= 100 || step->RunPID[0].Error <= -100) && tim14.Step_Time >= 5500)
//		{
//			tim14.Step_Time = 5500;
//		}
//		else if(tim14.Step_Time > 5501 && step->Step2FinishFlag == 1)
//		{
//			step->angle_out[0] = Step_Reset_Pid(&step->ResetPID[0],step->angle[0],step->first_angle[0]+10*19);
//			step->angle_out[1] = Step_Reset_Pid(&step->ResetPID[1],step->angle[1],step->first_angle[1]+10*27);
//		}
//		
//	}
//		if(rc_Ctrl.isOnline == 1)
//		{
//			MotorFillData(&step->motor[1],step->angle_out[1]);
//			MotorFillData(&step->motor[0],step->angle_out[0]);
////			MotorFillData(&step->motor[1],0);
////			MotorFillData(&step->motor[0],0);
//		}
//		else
//		{
//			MotorFillData(&step->motor[1],0);
//			MotorFillData(&step->motor[0],0);
//		}
//}


void StepAnglePlus(Motor* motor,float* angle,float* last_angle)
{
	if((motor->Data.Angle<-100)&&(*last_angle>100))    //换算为累加角度
	{
		*angle +=  360 + motor->Data.Angle - *last_angle;
	}
	else if((motor->Data.Angle>100)&&(*last_angle<-100))
	{
		*angle += -360 + motor->Data.Angle - *last_angle;
	}
	else
	{
		*angle += motor->Data.Angle - *last_angle;
	}
	
	*last_angle = motor->Data.Angle;
}


float Step_Pid_Start(BasePID_Object* base_pid,float Feedback,float target)
{   
	base_pid->Error = target - Feedback;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;

	if(base_pid->Error <= 300 && base_pid->Error >= -300)
	{
		base_pid->KiPart += base_pid->Error * base_pid->Ki;
	}
	else
	{
		base_pid->KiPart = 0;
	}
	
	if(base_pid->KiPart > base_pid->KiPartDetachment) base_pid->KiPart = base_pid->KiPartDetachment;
	if(base_pid->KiPart < -base_pid->KiPartDetachment) base_pid->KiPart = -base_pid->KiPartDetachment;
		
	base_pid->Out=base_pid->KpPart+base_pid->KiPart; 
	
	if(base_pid->Out > 8000) base_pid->Out = 8000;//电流值给得太大
	if(base_pid->Out < -8000) base_pid->Out = -8000;
	return base_pid->Out;
}

float Step_Reset_Pid(BasePID_Object* base_pid,float Feedback,float target)
{   
	
	base_pid->Error = target - Feedback;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;

	if(base_pid->Error < base_pid->KiPartDetachment || base_pid->Error > -base_pid->KiPartDetachment)
	{
		base_pid->KiPart += base_pid->Error * base_pid->Ki;
	}
	else
	{
		base_pid->KiPart=0;
	}

	if(base_pid->KiPart>=1000)
	{
		base_pid->KiPart=1000;
	}
	else if(base_pid->KiPart<=-1000)
	{
		base_pid->KiPart=-1000;		
	}
	
	base_pid->Out=base_pid->KpPart+base_pid->KiPart; 
	
	if(base_pid->Out>6000) base_pid->Out=6000;//电流值给得太大
	if(base_pid->Out<-6000) base_pid->Out=-6000;
	
	return base_pid->Out;
}


float Step_Pid_Push(BasePID_Object* base_pid,float Feedback,float target)
{   
	base_pid->Error = target - Feedback;
	
	base_pid->KpPart = base_pid->Error * base_pid->Kp;

	if(base_pid->Error<=8000&&base_pid->Error>=-8000)
	{
	base_pid->KiPart+=base_pid->Error*base_pid->Ki;
	}
	else
	{
		base_pid->KiPart=0;
	}
	
	if(base_pid->KiPart>base_pid->KiPartDetachment) base_pid->KiPart=base_pid->KiPartDetachment;
	if(base_pid->KiPart<-base_pid->KiPartDetachment) base_pid->KiPart=-base_pid->KiPartDetachment;
		
	base_pid->Out=base_pid->KpPart+base_pid->KiPart; 
	
	if(base_pid->Out>16000) base_pid->Out=16000;//电流值给得太大
	if(base_pid->Out<-16000) base_pid->Out=-16000;
	return base_pid->Out;
}

