//#include "power.h"
//#include "pid.h"
//float BasePID_BaseControl(BasePID_Object* base_pid, float Chassis_Power_Buffer, float Chassis_Power){
//	float Target_Power_Buff=55;
//  base_pid->Error = -Target_Power_Buff+Chassis_Power_Buffer;
//	base_pid->KpPart = base_pid->Error * base_pid->Kp;
//	base_pid->KiPart += base_pid->Error * base_pid->Ki;
//	if(base_pid->Error > base_pid->KiPartDetachment)
//	{
//		base_pid->KiPart = 0;
//	}
//	else if(base_pid->Error < -(base_pid->KiPartDetachment))
//	{
//		base_pid->KiPart = 0;
//	}
//	base_pid->KdPart = (-1) * base_pid->Kd * Chassis_Power ;
//	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
//}

//float BasePID_PowerControl(BasePID_Object* base_pid, float Chassis_Power_Buffer, float Chassis_Power){
//	float Target_Power_Buffer=10;
//  base_pid->Error = -Target_Power_Buffer+Chassis_Power_Buffer;
//	base_pid->KpPart = base_pid->Error * base_pid->Kp;
//	base_pid->KiPart += base_pid->Error * base_pid->Ki;
//	if(base_pid->Error > base_pid->KiPartDetachment)
//	{
//		base_pid->KiPart = 0;
//	}
//	else if(base_pid->Error < -(base_pid->KiPartDetachment))
//	{
//		base_pid->KiPart = 0;
//	}
//	base_pid->KdPart = (-1) * base_pid->Kd * Chassis_Power ;
//	base_pid->Out = base_pid->KpPart + base_pid->KiPart + base_pid->KdPart;
//}
