#ifndef ALL_CHASSIS_H
#define ALL_CHASSIS_H
#include "pid.h"
#include "motor.h"
#include "holder.h"
#include "check.h"
typedef struct 
{
	struct
	{
		uint8_t Enable;    					//< 使能状态			
		uint8_t isRefereeUpdating; 			//< 裁判系统是否在更新数据
		uint8_t DriveMode;					//< 操作模式
		uint8_t ChassisState;				//< 底盘状态
	}Flag;
	struct
  {
		float speed_limit;              // 根据不同等级底盘功率限制得到的速度
		float theoretical_power_sum;    // 想让缓冲能量保持在10J所需要的底盘总功率理论值
		float target_require_power_sum; // 直接进行速度PID计算后得到的控制电流值之和，用以表示功率实际值
		float scaling_ratio;            // 缩放比例 = 功率理论值 / 功率实际值
		float initial_give_power[4];    // initial power from PID calculation
		float scaled_give_power[4];
		struct
		{
		float torque_term_k1;        // k1
		float speed_term_k2;         // k2
		float CONSTANT_COEFFICIENT;
		float TORQUE_COEFFICIENT;		
		}coefficient;
		struct
		{
			uint8_t powermngmt_chassis_out; // 电管底盘接口的输出
			uint16_t energy_buffer;         // 缓冲能量
			uint16_t max_power;             // 功率上限
			float real_time_power;          // 实时功率
		}refereeData;
   }Power;
	struct 
	{
		Motor motor[4];
		BasePID_Object RunPID[4];
		BasePID_Object FollowPID;			
	}Motors;
	//< 控制底盘运动所需要的所有数据
	struct 
	{
		float Vx;			//< 前后运动的速度
		float Vy;		    //< 左右运动的速度
		float Vomega;		//< 旋转的角速度
		float brain_vx;
		float brain_vy;
		float		Vx_Sensitivity;
		float		Vy_Sensitivity;
		float		Vomega_Sensitivity;	
    uint8_t is;		
				struct
		{
		uint8_t flag_up_down_slope;
		uint8_t flag_up_up_slope;
		uint8_t flag_up_stat_slope;
		uint8_t flag_down_stat_slope;
		uint8_t flag_down_up_slope;
		uint8_t flag_down_down_slope;	
		}Slope_Flag;
		float angle_to_lidar;
		
	}Movement;
}AllChassis;
extern float speed1;//自旋速度
void AllChassisInit(AllChassis* chassis, BasePID_Object run_pid);
void Lidar_Allchassis_control(AllChassis* chassis,Check_Robot_State *CheckRobotState,Brain_t* brain,RC_Ctrl* rc_ctrl);
void Check_Slope(AllChassis *chassis,Holder_t* holder);
extern float angle_to_holder;
extern AllChassis allchassis;
# endif
