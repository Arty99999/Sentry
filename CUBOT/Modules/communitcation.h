#ifndef COM_H
#define COM_H
#include "stm32h7xx_hal.h"
#define RX_BUFF_LENGH 30
void Remote_Init(void);
 typedef struct { 
   struct     {       
   uint16_t sw;
	 uint16_t ch0;       
   uint16_t ch1;       
   uint16_t ch2;      
   uint16_t ch3;      
   uint8_t  s1; 
	 uint8_t  s1_last; 
   uint8_t  s2;
	 uint8_t  s2_last; 
	 int16_t left_VR; //拨杆变换值
	 int16_t left_HR;
	 int16_t right_VR;
	 int16_t right_HR;
	 int16_t left_RR;//拨轮变化值
    }rc; 
 
   struct     {      
   int16_t x;       
   int16_t y;       
   int16_t z;         
   uint8_t press_l;     
   uint8_t press_r; 
	 uint8_t press_l_flag;     
   uint8_t press_r_flag; 
	 }mouse; 
 
    struct  {     
    uint16_t v;  
    }keyboard;
	uint8_t  key[18];
	uint8_t  keyflag[18];	
	uint32_t key_filter_cnt[18];
 }RC_Ctl_t; 
 typedef struct
{
	uint16_t ID_0X201;
	uint16_t ID_0X202;
	uint16_t ID_0X203;
	uint16_t ID_0X204;
	uint16_t ID_0X205;
	uint16_t ID_0X206;
	uint16_t ID_0X207;
	uint16_t ID_0X208;
	uint16_t ID_0X1FF;
} Motor_ID_Info;
 typedef struct
{
	uint16_t 	Remote;
	uint16_t  Referee;
	Motor_ID_Info	Can1;
  Motor_ID_Info Can2;
	uint16_t  Vision;
	uint16_t  Super_cap;
	uint16_t  Slave_Broad;
	uint16_t  Power_Meter;
	uint16_t Uart4;
	uint16_t Uart5;
	uint16_t Uart7;
	uint16_t Uart8;
	 } Debug_Info;
 #define key_W        key[0]
#define key_A        key[1]
#define key_S        key[2]
#define key_D        key[3]
#define key_shift    key[4]
#define key_ctrl     key[5]
#define key_Q        key[6]
#define key_E        key[7]
#define key_V        key[8]
#define key_F        key[9]
#define key_G        key[10]
#define key_C        key[11]
#define key_R        key[12]
#define key_B        key[13]
#define key_Z        key[14]
#define key_X        key[15]
 
#define key_W_flag        keyflag[0]
#define key_A_flag        keyflag[1]
#define key_S_flag        keyflag[2]
#define key_D_flag        keyflag[3]
#define key_shift_flag    keyflag[4]
#define key_ctrl_flag     keyflag[5]
#define key_Q_flag        keyflag[6]
#define key_E_flag        keyflag[7]
#define key_V_flag        keyflag[8]
#define key_F_flag        keyflag[9]
#define key_G_flag        keyflag[10]
#define key_C_flag        keyflag[11]
#define key_R_flag        keyflag[12]
#define key_B_flag        keyflag[13]
#define key_Z_flag        keyflag[14]
#define key_X_flag        keyflag[15]

#define W_Num        0
#define A_Num        1
#define S_Num        2
#define D_Num        3
#define shift_Num    4
#define ctrl_Num     5
#define Q_Num        6
#define E_Num        7
#define V_Num        8
#define F_Num        9
#define G_Num        10
#define C_Num        11
#define R_Num        12
#define B_Num        13
#define Z_Num        14
#define X_Num        15
extern Debug_Info Check;
extern Debug_Info Flag;
extern Debug_Info Debug;
extern  RC_Ctl_t RC_Ctl;
extern int16_t Raw_Can_angle;
extern uint8_t dbus_meta_data[RX_BUFF_LENGH];
void Remote_deal(uint8_t * dbus_meta_data);
void Can1_Init(void);
void Can2_Init(void);
void MX_FDCAN2_Filter_Init(void);
void MX_FDCAN1_Filter_Init(void);
void USART1_IDLE_CALLBACK();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
uint8_t FDCAN1_send_msg_load(float * number);
uint8_t FDCAN2_send_msg_chassis(float * number);
uint8_t FDCAN2_send_msg_Supercap(uint8_t state);
#endif 