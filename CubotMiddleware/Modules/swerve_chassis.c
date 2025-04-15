#include "swerve_chassis.h"
#include "math.h"
#include "hardware_config.h"
#include "referee.h"
#include "math.h"
#include "hardware_config.h"
#include "control_logic.h"

#define myABS(x) ( (x)>0?(x):-(x) )
#define AtR 0.01745328f//3.14159/180 �Ƕ���ת��Ϊ������

static int8_t SinRollingCnt=1;
float test_power;
int randnumber[20]={27,53,29,55,20,54,3,26,18,65,32,8,44,11,73,29,41,7,33,40};
int randtime;

/**
  * @brief �����˶�ģʽ����
  */
void Chassis_Mode_Control(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Holder_t* holder,Chassis_Attitude_Info* Swerve,Supercap* Cap)
{
	
	
	SwerveChassisPowerControl(&swerveChassis,&ChassisSwerve,&rc_Ctrl);	
			if(Vision_Info.Hit_Mode == 1||Vision_Info.Hit_Mode == 2)                         
			{
				if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 0)//ֹͣ
				{
					chassis->Movement.Omega = 0;
					
					SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve, -holder->Motors6020.motor[0].Data.Angle);
					SwerveChassisMotionControl(&swerveChassis,&rc_Ctrl , &Holder);
				}
				else if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 1)
				{
					
					if(chassis->Movement.Vx>3000){
					chassis->Movement.Vx=3000;
					}
					if(chassis->Movement.Vx<-3000){
					chassis->Movement.Vx=-3000;
					}				
					if(chassis->Movement.Vy>3000){
					chassis->Movement.Vy=3000;
					}
					if(chassis->Movement.Vy<-3000){
					chassis->Movement.Vy=-3000;
					}
					
					SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve, 0);
					SwerveChassisMotionControl(&swerveChassis,&rc_Ctrl , &Holder);
			  }
				else if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 2)
				{
//					chassis->Movement.Vx = vx_test;
//		      chassis->Movement.Vy = vy_test;
					
					chassis->Movement.Omega =0;
				  if(chassis->Movement.Vx>3000){
		      chassis->Movement.Vx=3000;
		      }
		      if(chassis->Movement.Vx<-3000){
		      chassis->Movement.Vx=-3000;
		      }				
		      if(chassis->Movement.Vy>3000){
		      chassis->Movement.Vy=3000;
		      }
		      if(chassis->Movement.Vy<-3000){
		      chassis->Movement.Vy=-3000;
			    }
					
					SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve,-holder->Motors6020.motor[0].Data.Angle);		
		      SwerveChassisMotionControl(&swerveChassis ,&rc_Ctrl , &Holder); 
				}
				else if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 3)
				{
					chassis->Movement.Omega = 2700 + 300*sin(roll_time *0.07)+randnumber[randtime]*6;
					
					
					SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve, -holder->Motors6020.motor[0].Data.Angle);
					SwerveChassisMotionControl(&swerveChassis,&rc_Ctrl , &Holder);
				}
	    }
			else if(Vision_Info.Hit_Mode != 1)
			{
//				vx_test = 0;
//				vy_test = 0;
//				chassis->brain_vx = 0;
//				chassis->brain_vy = 0;
				
				if(chassis->Movement.Vx>3000){
		    chassis->Movement.Vx=3000;
		    }
		    if(chassis->Movement.Vx<-3000){
		    chassis->Movement.Vx=-3000;
		    }				
		    if(chassis->Movement.Vy>3000){
		    chassis->Movement.Vy=3000;
		    }
		    if(chassis->Movement.Vy<-3000){
		    chassis->Movement.Vy=-3000;
			  }
				
				if(Swerve->Roll_Flag==1)
					chassis->Movement.Omega = 2500;
				else
					chassis->Movement.Omega =  BasePID_AngleControlFollow(&chassis->Motors3508.FollowPID,0,holder->Motors6020.motor[0].Data.Angle, holder->Motors6020.motor[0].Data.SpeedRPM);
				
				SwerveChassisGetRemoteData(&swerveChassis, &rc_Ctrl,&ChassisSwerve,-holder->Motors6020.motor[0].Data.Angle);
				SwerveChassisMotionControl(&swerveChassis,&rc_Ctrl , &Holder);
			}
}


void SwerveChassisPowerControl(SwerveChassis* chassis,Chassis_Attitude_Info* Swerve,RC_Ctrl* rc_ctrl)
{
	uint16_t CurrentPowerLimit;
	uint16_t TargetPower;
	
	CurrentPowerLimit = referee2022.game_robot_status.chassis_power_limit;//referee2022.game_robot_status.chassis_power_limit

	TargetPower = CurrentPowerLimit + chassis->Power.Power_Plus;
	
	//���㡰���ٶȡ��仯��
	test_power=BasePID_PowerControl_Vmax(&chassis->Power.PowerPID_Vmax,TargetPower);

	//���������ޱ仯ʱ���޸����ٶȳ�ʼֵ
	if(chassis->Power.PowerLimitLast != TargetPower)
	{
		chassis->Power.SpeedPowerLimit_initial = FittedSpeed(TargetPower);
		chassis->Power.SpeedPowerLimit = chassis->Power.SpeedPowerLimit_initial;
		chassis->Power.PowerLimitLast = TargetPower;
	}

	//ȫ���˶�״̬�£��ԡ����ٶȡ��޸Ĳ��޷�
	if(chassis->Vectors.Rc_Ctrl_ModuleOfSpeed>=1||Swerve->Roll_Flag==1||Swerve->Roll_Flag1==1)
	{
		chassis->Power.SpeedPowerLimit += test_power;

		if(chassis->Power.SpeedPowerLimit > chassis->Power.SpeedPowerLimit_initial+1500)
			chassis->Power.SpeedPowerLimit = chassis->Power.SpeedPowerLimit_initial+1500;
		if(chassis->Power.SpeedPowerLimit < chassis->Power.SpeedPowerLimit_initial-1500)
			chassis->Power.SpeedPowerLimit = chassis->Power.SpeedPowerLimit_initial-1500;
	}
}

/**
  * @brief  ���ֵ��̳�ʼ��
  */
void SwerveChassisInit(SwerveChassis* chassis, BasePID_Object run_pid, BasePID_Object turn_pid, BasePID_Object power_pid,BasePID_Object follow_pid,BasePID_Object vmax_pid,BasePID_Object omegamax_pid, CanNumber canx)
{
	
	MotorInit(&chassis->Motors3508.motor[0], 0 , Motor3508, canx, 0x201);
	MotorInit(&chassis->Motors3508.motor[1], 0 , Motor3508, canx, 0x202);
	MotorInit(&chassis->Motors3508.motor[2], 0 , Motor3508, canx, 0x203);
	MotorInit(&chassis->Motors3508.motor[3], 0 , Motor3508, canx, 0x204);
	
	 
	
	MotorInit(&chassis->Motors6020.motor[0], 561 , Motor6020, canx, 0x205);
	MotorInit(&chassis->Motors6020.motor[1], 3516 , Motor6020, canx, 0x206);
	MotorInit(&chassis->Motors6020.motor[2], 7615 , Motor6020, canx, 0x207);
	MotorInit(&chassis->Motors6020.motor[3], 3393 , Motor6020, canx, 0x208);

	BasePID_Init(&chassis->Motors3508.RunPID[0], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[1], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[2], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors3508.RunPID[3], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);

	BasePID_Init(&chassis->Motors6020.TurnPID[0], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors6020.TurnPID[1], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors6020.TurnPID[2], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	BasePID_Init(&chassis->Motors6020.TurnPID[3], turn_pid.Kp, turn_pid.Ki, turn_pid.Kd, turn_pid.KiPartDetachment);
	
	BasePID_Init(&chassis->Motors3508.FollowPID, follow_pid.Kp, follow_pid.Ki, follow_pid.Kd, follow_pid.KiPartDetachment);
	BasePID_Init(&chassis->Power.PowerPID_Vmax, vmax_pid.Kp, vmax_pid.Ki, vmax_pid.Kd, vmax_pid.KiPartDetachment);

	chassis->Power.PowerLimitLast = referee2022.game_robot_status.chassis_power_limit;
	chassis->Power.K_PowerPlus = 140;
	chassis->Power.K_OnlyRolling = 0.7;
	chassis->Power.K_MovingRolling = 1.3;
	chassis->Power.K_Turning = 0.3;
	chassis->Power.Power_Plus = 0;

	chassis->Movement.Sensitivity.Vx 	= 5;
	chassis->Movement.Sensitivity.Vy    = 5;
	chassis->Movement.Sensitivity.Omega = 4;
	chassis->Movement.ModuleOfSpeed  	= 0;  
	chassis->Movement.AngleOfSpeed 	 	= 0;
	chassis->Movement.K_OmeToSpeed		= 0.5;
	chassis->Flag.SinRollingFlag		= 0;

}

/**
  * @brief  ��ȡ���ջ��������ݣ����ݶ����˶��߼�����
  */
void SwerveChassisGetRemoteData(SwerveChassis* chassis, RC_Ctrl* rc_ctrl,Chassis_Attitude_Info* Swerve, float canAngle)
{


	float angle;
	int32_t rawVx;
	int32_t rawVy;
	int32_t rawVx1;
	int32_t rawVy1;

//	if(step.Step1FinishFlag == 1)
//	{
//		angle = 45;
//	}
//	else
	{
		angle = AtR * (-canAngle);
	}
	
	//�������ٶȡ��ַ�Ϊ����ת�ٶȡ���ƽ���ٶ�
  	if((Swerve->Roll_Flag==1||Swerve->Roll_Flag1==1) && step.StepFlag==0 ) //С����ģʽ
	{
		if(chassis->Flag.SinRollingFlag==1)			//����С����ģʽ
		{
			SinRollingCnt++;
		}
		else
		{
			if(rc_ctrl->Chassis_Y_Integ==0&&rc_ctrl->Chassis_X_Integ==0)
			{
				chassis->Movement.RollingOmega = chassis->Power.SpeedPowerLimit * chassis->Power.K_OnlyRolling;
				chassis->Movement.ModuleOfSpeed = 0;
			}
			else
			{
				//�����߼�Ӧ���ǳ��Ը��Ŷ��Ŷԣ�����ʵ������ñ��ϵ��������
				chassis->Movement.RollingOmega = chassis->Movement.K_OmeToSpeed * chassis->Power.SpeedPowerLimit * chassis->Power.K_MovingRolling;

				chassis->Movement.ModuleOfSpeed = (1 - chassis->Movement.K_OmeToSpeed) * chassis->Power.SpeedPowerLimit;
			}
		}
	}
	else
	{
		if(rc_Ctrl.rc.s1==2)//ȥ�����̸���
		{
//			chassis->Movement.RollingOmega = BasePID_AngleControlFollow(&chassis->Motors3508.FollowPID,45,holder->Motors6020.motor[0].Data.Angle, holder->Motors6020.motor[0].Data.SpeedRPM);		
//			if(step.Step1FinishFlag == 1)
				chassis->Movement.RollingOmega = 0;
		}
		else
		{
			chassis->Movement.RollingOmega = BasePID_AngleControlFollow(&chassis->Motors3508.FollowPID,0,Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);
			if(chassis->Movement.RollingOmega >= chassis->Movement.K_OmeToSpeed * chassis->Power.SpeedPowerLimit * chassis->Power.K_Turning)
			{
				chassis->Movement.RollingOmega = (1 - chassis->Movement.K_OmeToSpeed) * chassis->Power.SpeedPowerLimit;
			}
			else if(chassis->Movement.RollingOmega <= -chassis->Movement.K_OmeToSpeed * chassis->Power.SpeedPowerLimit * chassis->Power.K_Turning)
			{
				chassis->Movement.RollingOmega = -(1 - chassis->Movement.K_OmeToSpeed) * chassis->Power.SpeedPowerLimit;
			}
		}
		
		chassis->Movement.ModuleOfSpeed = chassis->Power.SpeedPowerLimit - abs(chassis->Movement.RollingOmega*3);
	}

	VectorSolve(rc_ctrl->Chassis_X_Integ/3,rc_ctrl->Chassis_Y_Integ/3,&chassis->Vectors.Rc_Ctrl_AngleOfSpeed,&chassis->Vectors.Rc_Ctrl_ModuleOfSpeed,4);

	//��ң�����������ٶ�����������Բ��
	if(chassis->Vectors.Rc_Ctrl_ModuleOfSpeed>=1)
		chassis->Vectors.Rc_Ctrl_ModuleOfSpeed=1;
	if(chassis->Vectors.Rc_Ctrl_ModuleOfSpeed<=-1)
		chassis->Vectors.Rc_Ctrl_ModuleOfSpeed=-1;
	
	if((rc_Ctrl.rc.s1==3||rc_Ctrl.rc.s1==2)&&rc_Ctrl.rc.s2==3)
	{
	rawVx = chassis->Movement.ModuleOfSpeed * cos(0.01745329f*chassis->Vectors.Rc_Ctrl_AngleOfSpeed) * chassis->Vectors.Rc_Ctrl_ModuleOfSpeed;
	rawVy = chassis->Movement.ModuleOfSpeed * sin(0.01745329f*chassis->Vectors.Rc_Ctrl_AngleOfSpeed) * chassis->Vectors.Rc_Ctrl_ModuleOfSpeed;
	}
	else if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 1&&rc_Ctrl.rc.s1==2&&rc_Ctrl.rc.s2==2)
	{
		if(chassis->brain_vx>1000)chassis->brain_vx=1000;
		if(chassis->brain_vx<-1000)chassis->brain_vx=-1000;
		if(chassis->brain_vy>1000)chassis->brain_vy=1000;
		if(chassis->brain_vy<-1000)chassis->brain_vy=-1000;
		chassis->Movement.RollingOmega = 2400;
		rawVx1=chassis->brain_vx;
		rawVy1=chassis->brain_vy;

//		if(lidar_offline==0)
//		{
//			chassis->brain_vx=0;
//			chassis->brain_vy=0;
//			chassis->Movement.RollingOmega= 2700 + 500*sin(roll_time *0.09)+randnumber[randtime]*5;
//		}
	}

		else if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 2&&rc_Ctrl.rc.s1==2&&rc_Ctrl.rc.s2==2)
	{
		if(chassis->brain_vx>1000)chassis->brain_vx=1000;
		if(chassis->brain_vx<-1000)chassis->brain_vx=-1000;
		if(chassis->brain_vy>1000)chassis->brain_vy=1000;
		if(chassis->brain_vy<-1000)chassis->brain_vy=-1000;
		chassis->Movement.RollingOmega =2400;
		rawVx1=chassis->brain_vx;
		rawVy1=chassis->brain_vy;
//				if(lidar_offline==0)
//		{
//			chassis->brain_vx=0;
//			chassis->brain_vy=0;
//			chassis->Movement.RollingOmega= 2700 + 500*sin(roll_time *0.09)+randnumber[randtime]*5;
//		}
	}

	else if(Brain_chassis.BrainCore[1].CoreInstruction.movemode == 3&&rc_Ctrl.rc.s1==2&&(rc_Ctrl.rc.s2==2||rc_Ctrl.rc.s2==1))
	{
		if(chassis->brain_vx>1000)chassis->brain_vx=1000;
		if(chassis->brain_vx<-1000)chassis->brain_vx=-1000;
		if(chassis->brain_vy>1000)chassis->brain_vy=1000;
		if(chassis->brain_vy<-1000)chassis->brain_vy=-1000;
		rawVx=0;
		rawVy=0;
		chassis->Movement.RollingOmega = 2700 + 500*sin(roll_time *0.09)+randnumber[randtime]*5;
		if(chassis->Movement.RollingOmega>3300)chassis->Movement.RollingOmega=3300;
//				if(lidar_offline==0)
//		{
//			chassis->brain_vx=0;
//			chassis->brain_vy=0;
//			chassis->Movement.RollingOmega= 2700 + 500*sin(roll_time *0.09)+randnumber[randtime]*5;
//		}
	}
	else
  {
		rawVx=0;
		rawVy=0;
  }
	chassis->Movement.Vx = (rawVx * cos(angle) - rawVy * sin(angle));
	chassis->Movement.Vy = (rawVx * sin(angle) + rawVy * cos(angle));

	//������ջ���������ת������ƽ���ٶȺ��������ĸ��������Ϸֽ�ĽǶ�
	chassis->Vectors.Vx[0] =  chassis->Movement.Vx + chassis->Movement.RollingOmega;
	chassis->Vectors.Vy[0] =  chassis->Movement.Vy + chassis->Movement.RollingOmega;
	chassis->Vectors.Vx[1] =  chassis->Movement.Vx + chassis->Movement.RollingOmega;
	chassis->Vectors.Vy[1] =  chassis->Movement.Vy - chassis->Movement.RollingOmega;
	chassis->Vectors.Vx[2] =  chassis->Movement.Vx - chassis->Movement.RollingOmega;
	chassis->Vectors.Vy[2] =  chassis->Movement.Vy - chassis->Movement.RollingOmega;
	chassis->Vectors.Vx[3] =  chassis->Movement.Vx - chassis->Movement.RollingOmega;
	chassis->Vectors.Vy[3] =  chassis->Movement.Vy + chassis->Movement.RollingOmega;

	for(int i=0;i<4;i++)
	{
		//���ݽ��ջ�ˮƽ�ʹ�ֱ����ת���ٶȷ�������������ģ�ͼн�
		VectorSolve(chassis->Vectors.Vx[i], chassis->Vectors.Vy[i], &chassis->Vectors.Angle[i] ,&chassis->Vectors.SpeedNo[i],i);
		if(step.StepFlag==0)
		{	//���°���û�а�����
			//��ң���������Ŀ��Ƕ��뵱ǰ�����ĵ���ǶȶԱȣ�ѡ����뷴���Ƕ������Ŀ��Ƕȣ������жϵ������ת�Ƿ���Ҫ�ı�
			chassis->Vectors.BestAngle[i] = FindBestTargetAngle(chassis->Vectors.Angle[i], chassis->Motors6020.motor[i].Data.Angle, &chassis->Vectors.SpeedChangeFlag[i]);
//			chassis->Vectors.BestAngle[i] = FindBestTargetAngle_special(chassis->Vectors.Angle[i], chassis->Motors6020.motor[i].Data.Angle, &chassis->Vectors.SpeedChangeFlag[i],i);

		}
		else
		{//�ӽ��ջ��ӵ�Ҫ���µ�ָ��ʱ�����°�������Ѱ��Ŀ��Ƕȵ�ʱ��ȡ��һ���ӽ��ջ����ܵĽǶȶ���Ԥ�����úõĽǶ�
			chassis->Vectors.BestAngle[i] = FindBestTargetAngle_special(chassis->Vectors.Angle[i], chassis->Motors6020.motor[i].Data.Angle, &chassis->Vectors.SpeedChangeFlag[i],i);
		}

		//Ŀ�������ֵ
		chassis->Vectors.TargetEcd[i] = chassis->Motors6020.motor[i].Param.EcdOffset + 8192.0f*((float)chassis->Vectors.BestAngle[i]/(float)360.0f);

		//��Ŀ��Ƕȵı�����ֵΪ��㣬��������ǰ�����Ƕȱ�����ֵ��Ӧ�ĽǶ� (-180 <- 0 -> 180)
		chassis->Vectors.FeedbackAngle[i] = EcdtoAngle(chassis->Vectors.TargetEcd[i], chassis->Motors6020.motor[i].Data.RawEcd);
	}

}

/**
  * @brief  �ڶ������˶�ѧ����󱻵��ã�����3508��6020���������ֵ��������CAN����֡��0x200��0x1ff��������֡
  */
void SwerveChassisMotionControl(SwerveChassis* chassis ,RC_Ctrl* rc_ctrl ,Holder_t* holder)
{
	for(int j =0; j<4; j++)
	{
		if(chassis->Vectors.SpeedChangeFlag[j] == 1)
			chassis->Vectors.SpeedNo[j] = - chassis->Vectors.SpeedNo[j];
		else
			chassis->Vectors.SpeedNo[j] = chassis->Vectors.SpeedNo[j];

		chassis->Motors3508.motor[j].Data.Target = BasePID_SpeedControl((BasePID_Object*)(chassis->Motors3508.RunPID + j), chassis->Vectors.SpeedNo[j], chassis->Motors3508.motor[j].Data.SpeedRPM);

	    chassis->Motors6020.motor[j].Data.Target = BasePID_AngleControl((BasePID_Object*)(chassis->Motors6020.TurnPID + j), -chassis->Vectors.FeedbackAngle[j], 0, 0.1047f*chassis->Motors6020.motor[j].Data.SpeedRPM); //< RPMת��Ϊrad/s ��λת��ϵ��

	}

	for(int i=0;i<4;i++)
	{
		MotorFillData(&chassis->Motors3508.motor[i],chassis->Motors3508.motor[i].Data.Target);
		MotorFillData(&chassis->Motors6020.motor[i],chassis->Motors6020.motor[i].Data.Target);
	}
}


/**
* @brief ���ݽ��ջ��Ƿ����ߣ��ж��Ƿ����������̿����źš����̵�CAN����ȫ������0x1ff��0x200����CAN����֡
  */
void SwerveChassisOutputControl(SwerveChassis* chassis, RC_Ctrl rcCtrl)
{
	if(rcCtrl.isOnline == 1) 
	{
		MotorCanOutput(can2, 0x1ff);  
		MotorCanOutput(can2, 0x200);		
	}
	else //���ջ����ߣ�������Ϊ0
	{
		MotorFillData(&chassis->Motors6020.motor[0],0);
		MotorFillData(&chassis->Motors6020.motor[1],0);
		MotorFillData(&chassis->Motors6020.motor[2],0);
		MotorFillData(&chassis->Motors6020.motor[3],0);
		MotorFillData(&chassis->Motors3508.motor[0],0);
		MotorFillData(&chassis->Motors3508.motor[1],0);
		MotorFillData(&chassis->Motors3508.motor[2],0);
		MotorFillData(&chassis->Motors3508.motor[3],0);
		MotorCanOutput(can2, 0x1ff);
		MotorCanOutput(can2, 0x200);
	}
}

float FittedSpeed(float power_input)
{
	float SpeedOutput;
	float power;
	
	if(power <= 50)		power = 50;
	if(power >= 120)	power = 120;
	
//	SpeedOutput = 0.004015905663f*targetpower*targetpower*targetpower
//				- 1.693680731034f*targetpower*targetpower
//				+ 267.660539950240f*targetpower
//				- 6491.526664511785f;

	SpeedOutput = - 0.0000229f 	*power*power*power*power
				  + 0.00872f 	*power*power*power
				  - 1.182f		*power*power
				  + 114.4f		*power
				  -610;

	return SpeedOutput;
}

/**
  * @brief  ������������������ϵ�µķ��������������ģ�ͽǶ�
  */
void VectorSolve(float vx, float vy, float* vectorAngle, float* vectorModule, uint8_t id)
{
	float vxDivideVy;
	float angle;
	static float lastAngle[5];
	float module;
	
	if((vx >0.1f || vx < -0.1f) || (vy >0.1f || vy < -0.1f))//������ջ�����������ƽ���ٶȺ������ĽǶȺ�ģֵ
	{
		vxDivideVy = (float)vx / (float)vy;
		angle =  57.29f * atan(vxDivideVy);
		module  =  sqrt(vx * vx + vy * vy);
		
		if(vx >= 0 && vy < 0)//�����Ǻ���ϵ�µĽǶ�ֵת��Ϊ0-180���0-(-180��)
		{
			angle = 180 + angle;
		}			
		else if(vx < 0 && vy < 0 )
		{		
			angle = -180 + angle;
		}
		lastAngle[id] = angle;
	}
	else
	{
		angle = lastAngle[id];
		module  =  0;
	}

	*vectorAngle  = angle;
	*vectorModule = module;
}

/**
  * @brief  �޸Ķ��ֵ���PID�����Ľӿں���
  */
void ChangeChassisPID(SwerveChassis* chassis, PIDParameters pid)
{
	for(int i =0;i<4;i++)
	{
		chassis->Motors6020.TurnPID[i].Kp = pid.TurnKp;
		chassis->Motors6020.TurnPID[i].Ki = pid.TurnKi;
		chassis->Motors6020.TurnPID[i].Kd = pid.TurnKd;
		chassis->Motors3508.RunPID[i].Kp  = pid.SpeedKp;
		chassis->Motors3508.RunPID[i].Ki  = pid.SpeedKi;
		chassis->Motors3508.RunPID[i].Kd  = pid.SpeedKd;
	}
}

/**
  * @brief  �������������ض������λ��ת��Ϊ+-180��ĽǶ�
  */
float EcdtoAngle(int16_t offset, int16_t ecd)
{
	float angle;
	
	if(offset < 4096 && ecd > offset + 4096)
	{
		ecd = ecd - 8192;
	}
	else if(ecd < offset - 4096)
	{
		ecd = ecd + 8192;
	}

	angle = K_ECD_TO_ANGLE * (ecd - offset);

	return angle;
}

/**
  * @brief  ���ݶ��������ǶȺͷ����Ƕȵļн��ж���СĿ��Ƕ�
  */
float FindBestTargetAngle(float targetAngle, float feedbackAngle, uint8_t* flag)
{
	float vectorDeltaAngle;
	float deltaAngle;
	
	vectorDeltaAngle = targetAngle - feedbackAngle;

	if((vectorDeltaAngle)>180)
		vectorDeltaAngle = vectorDeltaAngle - 360.0f;
	else if ((vectorDeltaAngle)<-180)
		vectorDeltaAngle = 360 + vectorDeltaAngle;

	if(targetAngle >= 0.0f)
	{
		deltaAngle = vectorDeltaAngle - 180.0f;
		if((deltaAngle)>180)
			deltaAngle = deltaAngle - 360.0f;
		else if ((deltaAngle)<-180)
			deltaAngle = 360 + deltaAngle;
	}
	else if(targetAngle < 0.0f)
	{
		deltaAngle = vectorDeltaAngle + 180.0f;
		if((deltaAngle)>180)
			deltaAngle = deltaAngle - 360.0f;
		else if ((deltaAngle)<-180)
			deltaAngle = 360 + deltaAngle;
	}
	
	if(myABS(vectorDeltaAngle) > myABS(deltaAngle))
	{
		if(targetAngle >= 0.0f)
			targetAngle = targetAngle - 180.0f;
		else if(targetAngle < 0.0f)
			targetAngle = targetAngle + 180.0f;
		*flag = 1;
	}
	else 
	{
		*flag = 0;
	}
	return targetAngle;
}
float FindBestTargetAngle_special(float targetAngle, float feedbackAngle, uint8_t* flag,uint8_t MotorNum)
{
//	float vectorDeltaAngle;	
//	vectorDeltaAngle = targetAngle - feedbackAngle;	
//	if((vectorDeltaAngle)>180)
//		vectorDeltaAngle = vectorDeltaAngle - 360.0f;
//	else if ((vectorDeltaAngle)<-180)
//		vectorDeltaAngle = 360 + vectorDeltaAngle;		
//	*flag=0;
//	return targetAngle;
	if(MotorNum >= 1)
	{
		if((targetAngle > -10 && targetAngle <= 180)||(targetAngle < -170 && targetAngle >= -180))
		{
			*flag = 0;
		}
		else
		{
			targetAngle = targetAngle + 180;
			*flag = 1;
		}
	}
	else
	{
		if((targetAngle >= -180 && targetAngle <= 10)||(targetAngle > 170 && targetAngle <= 180))
		{
			*flag = 0;
		}
		else
		{
			targetAngle = targetAngle - 180;
			*flag = 1;
		}
	}
	return targetAngle;
}
