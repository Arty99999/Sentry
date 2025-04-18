#include "all_chassis.h"
#include "hardware_config.h"
#include "check.h"
#include "brain.h"
#include "Holder.h"
#include "math.h"
#define min(a, b) ((a) < (b) ? (a) : (b))
#define AtR 0.0174532f //<  3.1415 /180 �Ƕ��� ת��Ϊ������
float total_power;	   // �ܹ���
float power_bili;	   // ����ϵ�������������ʿ���
float max_power;	   // �����?
float raw_vx;
float raw_vy;
float raw_vx_lidar;
float raw_vy_lidar;
float vtemp;		   // ��VXVY�õ�
float angle_to_holder; // ���״��̨����ϵ�����������ϵ�ĽǶ�?
float speed1 = 0;	   // �����ٶ�
float chassis_feedforward = 1;
float lidar_check_speed; // �״��������?
int cnt_3508;
uint32_t band_time;
int band_number[20] = {20, 78, 44, 70, 32, 65, 19, 99, 23, 66, 86, 19, 30, 6, 12, 56, 36, 74, 50, 27};
int band_number_ins;

void Speed_Poweroutput_Control_New(AllChassis *chassis);
void ALLChassisSetSpeed(AllChassis *chassis, float canAngle);
void ALLChassisSetSpeedTwo(AllChassis *chassis, float canAngle);
void Speed_Poweroutput_Control(AllChassis *chassis);

AllChassis allchassis = {
	.Power.coefficient = {
		.torque_term_k1 = 2.17e-07,
		.speed_term_k2 = 3.45e-07,
		.CONSTANT_COEFFICIENT = 0.8275f,
		.TORQUE_COEFFICIENT = 1.99688994e-6f}};

/**
 * @brief  ȫ���ֵ������˶�ѧ��Inverse Kinematics ,����chassis�ṹ���е�movement�ṹ������?�١�
 * @notec  ��������δ���й��ʿ��Ƶĵ���������д��Motor�ṹ���С�
 */
extern int Flag_Follow;
float nmm;
float Change_angel(float vx, float vy, float Can_angle);
void Lidar_Allchassis_control(AllChassis *chassis, Check_Robot_State *CheckRobotState, Brain_t *brain, RC_Ctrl *rc_ctrl)
{
	static int cnu, cnl, clock = 0;
	// ��������
	band_time++;
	if (band_time % 1000 == 0)
		band_number_ins++;
	if (band_number_ins == 20)
		band_number_ins = 0;
	speed1 = 3000 + band_number[band_number_ins] * 50;
	if (rc_ctrl->rc.s1 == 2)
	{
		if (CheckRobotState->Check_Usart.Check_lidar == 0) // �״�����
		{

			if (referee2022.game_status.game_progress != 4)
			{
				chassis->Movement.Vx = 0;
				chassis->Movement.Vy = 0;
				chassis->Movement.Vomega = speed1;

				ALLChassisSetSpeed(chassis, Holder.Motors6020.motor[0].Data.Angle);
			}
			else if (referee2022.game_status.game_progress == 4) // ������ʼ��
			{
				chassis->Movement.Vx = 0;
				chassis->Movement.Vy = 0;
				chassis->Movement.Vomega = speed1;
			}
		}
		else
		{

			if (brain->Lidar.mode == 0) // ֹͣ
			{
				chassis->Movement.Vx = 0;
				chassis->Movement.Vy = 0;

				chassis->Movement.Vomega = 0;
				ALLChassisSetSpeed(chassis, Holder.Motors6020.motor[0].Data.Angle);
			}
			else if (brain->Lidar.mode == 1) // ��ͨƽ��
			{

				chassis->Movement.Vx = brain->Lidar.vx / 1.25;
				chassis->Movement.Vy = brain->Lidar.vy / 1.25;
				// if (brain->Lidar.mode==To_fortress&&brain->Lidar.Arrive==1)
				//	nmm=Change_angel(-chassis->Movement.Vx,-chassis->Movement.Vy,Holder.Motors6020.motor[0].Data.Angle);
				// chassis->Movement.Vomega=1000;
				// chassis->Movement.Vomega=BasePID_AngleControlFollow(&pid_follow,0,-Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);

				ALLChassisSetSpeed(chassis, Holder.Motors6020.motor[0].Data.Angle);
			}
			else if (brain->Lidar.mode == 2) // ����ƽ��
			{
				chassis->Movement.Vx = chassis->Movement.brain_vx;
				chassis->Movement.Vy = chassis->Movement.brain_vy;
				// nmm=Change_angel(-chassis->Movement.Vx,-chassis->Movement.Vy,Holder.Motors6020.motor[0].Data.Angle);
				// if (chassis->Movement.Vx==0)				chassis->Movement.Vomega=BasePID_AngleControlFollow(&pid_follow,0,-Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);
				// else chassis->Movement.Vomega=BasePID_AngleControlFollow(&pid_follow,-nmm,-Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);
				ALLChassisSetSpeed(chassis, Holder.Motors6020.motor[0].Data.Angle);
			}
			else if (brain->Lidar.mode == 3) // ԭ������
			{
				chassis->Movement.Vx = 0;
				chassis->Movement.Vy = 0;
				chassis->Movement.Vomega = 0;
				ALLChassisSetSpeed(chassis, Holder.Motors6020.motor[0].Data.Angle);
			}
		}
	}
	else
	{
		brain->Lidar.angle_to_lidar = 0;
		angle_to_holder = 0;
		chassis->Movement.Vx = -(rc_Ctrl.rc.ch0 - 1024) * 5;
		chassis->Movement.Vy = -(rc_Ctrl.rc.ch1 - 1024) * 5;
		if (rc_Ctrl.rc.sw > 1400)
			chassis->Movement.Vomega = 7000;
		else if (rc_Ctrl.rc.sw < 600)
			chassis->Movement.Vomega = -7000;
		else if (rc_Ctrl.rc.sw > 600 && rc_Ctrl.rc.sw < 1400)
			chassis->Movement.Vomega = BasePID_AngleControlFollow(&pid_follow, 0, -Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);

		//	nmm=Change_angel(-chassis->Movement.Vx,-chassis->Movement.Vy,Holder.Motors6020.motor[0].Data.Angle);
		//	if (chassis->Movement.Vx==0)				chassis->Movement.Vomega=BasePID_AngleControlFollow(&pid_follow,0,-Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);
		//	else chassis->Movement.Vomega=BasePID_AngleControlFollow(&pid_follow,-nmm,-Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);
		// Check_Slope(&allchassis,&Holder);
		ALLChassisSetSpeed(chassis, Holder.Motors6020.motor[0].Data.Angle);
		//		atan(chassis->Movement.Vx/chassis->Movement.Vy)*57.3
		//	chassis->Movement.Vomega=BasePID_AngleControlFollow(&pid_follow,0,-Holder.Motors6020.motor[0].Data.Angle, Holder.Motors6020.motor[0].Data.SpeedRPM);
	}
}

void ALLChassisSetSpeed(AllChassis *chassis, float canAngle)
{
	float angle = -canAngle * AtR - Holder.Motors6020.motor[0].Data.SpeedRPM * 0.0026; // * Holder.Motors6020.motor[0].Data.SpeedRPM*chassis_feedforward;
																					   //	raw_vx=chassis->Movement.Vx*cos(-angle_to_lidar)-chassis->Movement.Vy*sin(-angle_to_lidar);
																					   //	raw_vy=chassis->Movement.Vx*sin(-angle_to_lidar)+chassis->Movement.Vy*cos(-angle_to_lidar);//��ͼ����ϵ����������ϵ������
	raw_vx_lidar = chassis->Movement.Vx * cos(-Brain.Lidar.angle_to_lidar) - chassis->Movement.Vy * sin(-Brain.Lidar.angle_to_lidar);
	raw_vy_lidar = chassis->Movement.Vx * sin(-Brain.Lidar.angle_to_lidar) + chassis->Movement.Vy * cos(-Brain.Lidar.angle_to_lidar); // ��������ϵ���״��̨������ϵ

	float rotated_vy = (raw_vx_lidar * sin(angle) + raw_vy_lidar * cos(angle));
	float rotated_vx = (raw_vx_lidar * cos(angle) - raw_vy_lidar * sin(angle)); // �״��̨������ϵ����������ϵ

	vtemp = rotated_vx;
	rotated_vx = rotated_vy;
	rotated_vy = -vtemp;

	chassis->Motors.motor[0].Data.Target = -1 * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target = -1 * rotated_vx + 1 * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target = 1 * rotated_vx + 1 * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target = 1 * rotated_vx + (-1) * rotated_vy + 1 * chassis->Movement.Vomega;

	Speed_Poweroutput_Control(chassis);
}

/**
 * @brief  ȫ���ֵ��̶����������˶�ѧ���㣬Inverse Kinematics ,����chassis�ṹ���е�movement�ṹ������?�١�
 * @notec  ��������δ���й��ʿ��Ƶĵ���������д��Motor�ṹ���С�
 *
 * ����45���˶�ʱ���˶�����ʱ�����Ħ�������ٶȻ��������Ҫ����ģ��ѹ����ۼ������������Ͽ��ܻ��ܵø��죿������������?
 * ���� �� ��ֱ��ʱ��ǰ����������Ҫת��������Ҫ��Ҫ�������������ܽ�ʡ���ٹ��ʣ�������ʱ�໬�̶Ȼ᲻��ܴ�?
 */
void ALLChassisSetSpeedTwo(AllChassis *chassis, float canAngle)
{
	float angle = canAngle * AtR; // �������������canAngleΪ0.0f
	float rotated_vy = (chassis->Movement.Vx * sin(angle) + chassis->Movement.Vy * cos(angle));
	float rotated_vx = (chassis->Movement.Vx * cos(angle) - chassis->Movement.Vy * sin(angle));

	chassis->Motors.motor[0].Data.Target = 1 * rotated_vx + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[1].Data.Target = (-1) * rotated_vy + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[2].Data.Target = (-1) * rotated_vx + 1 * chassis->Movement.Vomega;
	chassis->Motors.motor[3].Data.Target = 1 * rotated_vy + 1 * chassis->Movement.Vomega;

	Speed_Poweroutput_Control(chassis);
}

void Speed_Poweroutput_Control(AllChassis *chassis)
{
	chassis->Power.target_require_power_sum = 0;
	for (int8_t i = 0; i < 4; i++)
	{
		chassis->Motors.motor[i].Data.Output = BasePID_SpeedControl((BasePID_Object *)(chassis->Motors.RunPID + i), chassis->Motors.motor[i].Data.Target, chassis->Motors.motor[i].Data.SpeedRPM);
		if (i <= 1)
		{
			chassis->Motors.motor[i].Data.Output += chassis->Movement.Slope_Flag.flag_up_up_slope * 4000 - chassis->Movement.Slope_Flag.flag_up_down_slope * 1500 + chassis->Movement.Slope_Flag.flag_up_stat_slope * 1500;
			chassis->Motors.motor[i].Data.Output -= chassis->Movement.Slope_Flag.flag_down_up_slope * 4000 - chassis->Movement.Slope_Flag.flag_down_down_slope * 1500 + chassis->Movement.Slope_Flag.flag_down_stat_slope * 1500;
		}
		else
		{
			chassis->Motors.motor[i].Data.Output -= chassis->Movement.Slope_Flag.flag_up_up_slope * 4000 - chassis->Movement.Slope_Flag.flag_up_down_slope * 1500 + chassis->Movement.Slope_Flag.flag_up_stat_slope * 1500;
			chassis->Motors.motor[i].Data.Output += chassis->Movement.Slope_Flag.flag_down_up_slope * 4000 - chassis->Movement.Slope_Flag.flag_down_down_slope * 1500 + chassis->Movement.Slope_Flag.flag_down_stat_slope * 1500;
		}
		chassis->Power.initial_give_power[i] = chassis->Motors.motor[i].Data.Output * chassis->Power.coefficient.TORQUE_COEFFICIENT * chassis->Motors.motor[i].Data.SpeedRPM +
											   chassis->Power.coefficient.speed_term_k2 * chassis->Motors.motor[i].Data.SpeedRPM * chassis->Motors.motor[i].Data.SpeedRPM +
											   chassis->Power.coefficient.torque_term_k1 * chassis->Motors.motor[i].Data.Output * chassis->Motors.motor[i].Data.Output + chassis->Power.coefficient.CONSTANT_COEFFICIENT;
		if (chassis->Power.initial_give_power[i] < 0) // negative Power not included (transitory)
			continue;
		chassis->Power.target_require_power_sum += chassis->Power.initial_give_power[i];
	}

	chassis->Power.refereeData.max_power = referee2022.game_robot_status.chassis_power_limit + (referee2022.power_heat_data.chassis_power_buffer - 15) * 2;
	chassis->Power.scaling_ratio = chassis->Power.refereeData.max_power / chassis->Power.target_require_power_sum;
	chassis->Power.scaling_ratio = LIMIT(chassis->Power.scaling_ratio, 0, 1);

	for (uint8_t i = 0; i < 4; i++)
	{

		if (chassis->Power.scaling_ratio == 1)
			continue;

		else
		{

			chassis->Power.scaled_give_power[i] = chassis->Power.initial_give_power[i] * chassis->Power.scaling_ratio; // get scaled Power
			if (chassis->Power.scaled_give_power[i] < 0)
				continue;

			float b = chassis->Power.coefficient.TORQUE_COEFFICIENT * chassis->Motors.motor[i].Data.SpeedRPM;
			float c = chassis->Power.coefficient.speed_term_k2 * chassis->Motors.motor[i].Data.SpeedRPM * chassis->Motors.motor[i].Data.SpeedRPM - chassis->Power.scaled_give_power[i] + chassis->Power.coefficient.CONSTANT_COEFFICIENT;

			if (chassis->Motors.motor[i].Data.Output > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * chassis->Power.coefficient.torque_term_k1 * c)) / (2 * chassis->Power.coefficient.torque_term_k1);
				chassis->Motors.motor[i].Data.Output = temp;
			}
			else
			{
				float temp = (-b - sqrt(b * b - 4 * chassis->Power.coefficient.torque_term_k1 * c)) / (2 * chassis->Power.coefficient.torque_term_k1);
				chassis->Motors.motor[i].Data.Output = temp;
			}

			//        if ((ABS(chassis->movement.motor_output[0]) < 900) && (ABS(chassis->movement.motor_output[1]) < 900) && (ABS(chassis->movement.motor_output[2]) < 900) && (ABS(chassis->movement.motor_output[3]) < 900)) {
			//            for (uint8_t i = 0; i < 4; i++) {
			//                chassis->wheelMotor.m3508[i].treatedData.motor_output = 0;
			//            }
		}
	}

	for (uint8_t i = 0; i < 4; i++)
		MotorFillData(&chassis->Motors.motor[i], chassis->Motors.motor[i].Data.Output);
}
/**
 * @brief  ���ֵ��̳�ʼ�������������ĸ����̵�������ҿ������?��PID������
 */
void AllChassisInit(AllChassis *chassis, BasePID_Object run_pid)
{
	for (int i = 1; i <= 4; i++) //< ��ʼ���ĸ�����Ͷ�Ӧ��pid�ṹ��
	{
		MotorInit(&chassis->Motors.motor[i - 1], 0, Motor3508, CAN2, (0x200 + i)); //< ���ֵ��̵Ĵ�����Ĭ�Ϲ�����0x200�ϵ�
		BasePID_Init(&chassis->Motors.RunPID[i - 1], run_pid.Kp, run_pid.Ki, run_pid.Kd, run_pid.KiPartDetachment);
	}
	chassis->Movement.Vomega_Sensitivity = 1;
	chassis->Movement.Vx_Sensitivity = 1;
	chassis->Movement.Vy_Sensitivity = 1;
}
void Check_Slope(AllChassis *chassis, Holder_t *holder)
{
	static int Cnt_stat, Cnt_up_slope, Cnt_down_slope;
	if (holder->Pitch.GYRO_Angle - holder->Pitch.Can_Angle > 8)
	{
		Cnt_up_slope++;
		Cnt_stat = 0;
		Cnt_down_slope = 0;
	}
	else if (holder->Pitch.GYRO_Angle - holder->Pitch.Can_Angle < -8)
	{
		Cnt_down_slope++;
		Cnt_stat = 0;
		Cnt_up_slope = 0;
	}
	else
	{
		Cnt_up_slope = 0;
		Cnt_down_slope = 0;
		Cnt_stat++;
	}
	if (Cnt_up_slope > 100)
	{
		if (allchassis.Movement.Vy > 200)
		{
			chassis->Movement.Slope_Flag.flag_up_up_slope = 1;
			chassis->Movement.Slope_Flag.flag_up_down_slope = 0;
			chassis->Movement.Slope_Flag.flag_up_stat_slope = 0;
		}
		else if (allchassis.Movement.Vy < -200)
		{
			chassis->Movement.Slope_Flag.flag_up_up_slope = 0;
			chassis->Movement.Slope_Flag.flag_up_down_slope = 1;
			chassis->Movement.Slope_Flag.flag_up_stat_slope = 0;
		}
		else
		{
			chassis->Movement.Slope_Flag.flag_up_up_slope = 0;
			chassis->Movement.Slope_Flag.flag_up_down_slope = 0;
			chassis->Movement.Slope_Flag.flag_up_stat_slope = 1;
		}
		Cnt_up_slope = 100;
	}
	else if (Cnt_down_slope > 100)
	{
		if (allchassis.Movement.Vy > 200)
		{
			chassis->Movement.Slope_Flag.flag_down_down_slope = 1;
			chassis->Movement.Slope_Flag.flag_down_up_slope = 0;
			chassis->Movement.Slope_Flag.flag_down_stat_slope = 0;
		}
		else if (allchassis.Movement.Vy < -200)
		{
			chassis->Movement.Slope_Flag.flag_down_down_slope = 0;
			chassis->Movement.Slope_Flag.flag_down_up_slope = 1;
			chassis->Movement.Slope_Flag.flag_down_stat_slope = 0;
		}
		else
		{
			chassis->Movement.Slope_Flag.flag_down_down_slope = 0;
			chassis->Movement.Slope_Flag.flag_down_up_slope = 0;
			chassis->Movement.Slope_Flag.flag_down_stat_slope = 1;
		}
		Cnt_down_slope = 100;
	}
	else if (Cnt_stat > 100)
	{
		Cnt_stat = 100;
		chassis->Movement.Slope_Flag.flag_up_up_slope = 0;
		chassis->Movement.Slope_Flag.flag_up_down_slope = 0;
		chassis->Movement.Slope_Flag.flag_down_up_slope = 0;
		chassis->Movement.Slope_Flag.flag_down_down_slope = 0;
		chassis->Movement.Slope_Flag.flag_up_stat_slope = 0;
		chassis->Movement.Slope_Flag.flag_down_stat_slope = 0;
	}
}
double fmod_positive(double x, double y)
{
	double result = fmod(x, y);
	return result < 0 ? result + y : result;
}

// �Ƕȹ淶���� [-��, ��]
double normalize_angle(double theta)
{
	double normalized = fmod_positive(theta + 3.1416, 2 * 3.1416) - 3.1416;
	return normalized;
}
float angle;
float a1, a2, a3, a4, abs1, abs2, abs3, abs4;
#define abs(x) ((x) > 0 ? (x) : (-(x)))
float Change_angel(float vx, float vy, float Can_angle)
{
	// float a1,a2,a3,a4,abs1,abs2,abs3,abs4;
	Can_angle = Can_angle / 57.3;
	if (vy == 0)
		return (Can_angle * 57.3);
	angle = atan2(vx, vy); // ʹ�ø���ȷ�Ļ���ת�Ƕ�ϵ��

	a1 = normalize_angle(angle + 3.1416 / 2);
	a2 = normalize_angle(angle - 3.1416 / 2);
	a3 = normalize_angle(angle - 3.1416);
	a4 = angle;

	abs1 = min(abs(Can_angle - a1), 2 * 3.1416 - abs(Can_angle - a1));
	abs2 = min(abs(Can_angle - a2), 2 * 3.1416 - abs(Can_angle - a2));
	abs3 = min(abs(Can_angle - a3), 2 * 3.1416 - abs(Can_angle - a3));
	abs4 = min(abs(Can_angle - a4), 2 * 3.1416 - abs(Can_angle - a4));

	if (min(abs1, min(abs2, min(abs3, abs4))) == abs1)
		return a1 * 57.3;
	else if (min(abs1, min(abs2, min(abs3, abs4))) == abs2)
		return a2 * 57.3;
	else if (min(abs1, min(abs2, min(abs3, abs4))) == abs3)
		return a3 * 57.3;
	else
		return a4 * 57.3;
}
