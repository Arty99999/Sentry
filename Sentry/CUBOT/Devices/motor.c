/**@file    motor.c
* @brief    设备层，电机控制代码。设备调用驱动层产生的数据结构进行再设备层完成构建，不暴露給用户
* @details  通过调用CAN驱动层接收处理和发送电机的相关数据
* @date        2021-10-10
* @version     V1.0
* @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
**********************************************************************************
* @attention
* 硬件平台: STM32H750VBT \n
* SDK版本：-++++
* @par 修改日志:
* <table>
* <tr><th>Date       <th>Version  <th>Author    <th>Description
* <tr><td>2021-10-10  <td>1.0      <td>RyanJiao  <td>完成收发函数编写
* </table>
*
**********************************************************************************
 ==============================================================================
                          How to use this driver
 ==============================================================================

    添加driver_can.h

    1. 创建Motor结构体，作为电机实例。

    1. 调用MotorInit()，初始化电机静态数据。

    2. 在启动CAN时注册的CANx_rxCallBack回调中判断ID并添加MotorRxCallback()，接收电机动态数据。

    3. 经过PID计算后产生待发送数据OutputCurrent。

    4. 发送数据应当调用MotorFillData()填写对应控制ID下的待发送数据

    5. 所有数据填写完毕后调用MotorCanOutput()发送对应CAN设备下特定控制ID的CAN数据

  ********************************************************************************
    * @attention
    * 硬件平台: STM32H750VBT \n
    * SDK版本：-++++
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding
  * through your new brief.
***********************************************************************************/
#include "motor.h"
#include "check.h"
int Q_index = 0;
uint32_t can1_cnt;
uint32_t can2_cnt;
float Ecd_sum   = 0;
float Speed_sum = 0;
/**
 * @brief 对应大疆电机不同控制ID的CAN数据发送缓存区
 */
CAN_TxBuffer txBuffer0x200forCAN1 = {
    .Identifier = 0x200};
CAN_TxBuffer txBuffer0x1FFforCAN1 = {
    .Identifier = 0x1ff};
CAN_TxBuffer txBuffer0x2FFforCAN1 = {
    .Identifier = 0x2ff};
CAN_TxBuffer txBuffer0x200forCAN2 = {
    .Identifier = 0x200};
CAN_TxBuffer txBuffer0x1FFforCAN2 = {
    .Identifier = 0x1ff};
CAN_TxBuffer txBuffer0x2FFforCAN2 = {
    .Identifier = 0x2ff};

/**
 * @brief  编码器解算函数，编码器刻度转换为角度
 */

		
static void MotorEcdtoAngle(Motor *motor)
{
    static int8_t num = 0;
    int16_t total_ecd,a,b;
    if (motor->Param.CanId == 0x205 && motor->Param.CanNumber == CAN2) {
        if (motor->Data.Ecd < 1000 && motor->Data.Ecd >= 0 && motor->Data.LastEcd < 8192 && motor->Data.LastEcd >7191) {
            num++;
            if (num == 2) num = 0;
        } else if (motor->Data.LastEcd < 1000 && motor->Data.LastEcd >= 0 && motor->Data.Ecd < 8192 && motor->Data.Ecd > 7191) {
            num--;
            if (num == -2) num = 0;
        }
        if (num == 0)
            total_ecd = motor->Data.Ecd ;
        else
            total_ecd = motor->Data.Ecd  + motor->Param.EcdFullRange+1;
        if (total_ecd > (motor->Param.EcdOffset + motor->Param.EcdFullRange))
            total_ecd -= motor->Param.EcdFullRange*2+1;
        motor->Data.Angle = K_ECD_TO_ANGLE * (total_ecd - motor->Param.EcdOffset) / 2;
				
				
				
    } else {
        if ((&motor->Param)->EcdOffset < ((&motor->Param)->EcdFullRange / 2)) {
            if (motor->Data.Ecd > (motor->Param.EcdOffset + motor->Param.EcdFullRange / 2))
                motor->Data.Ecd -= motor->Param.EcdFullRange;
        } else {
            if (motor->Data.Ecd < (motor->Param.EcdOffset - motor->Param.EcdFullRange / 2))
                motor->Data.Ecd += motor->Param.EcdFullRange;
        }
				a=motor->Data.Ecd - motor->Param.EcdOffset;
				b=motor->Data.LastEcd - motor->Param.EcdOffset;
				if (a < -3996 && a >= -4096 && b < 4096 && b >3096)
				motor->Data.RoundCnt++;
				else if (b < -3996 && b >= -4096 && a < 4096 && a >3096)
				motor->Data.RoundCnt--;
	
        motor->Data.Angle = K_ECD_TO_ANGLE * (motor->Data.Ecd - motor->Param.EcdOffset);
    }
		motor->Data.TotalAngle=360*motor->Data.RoundCnt+motor->Data.Angle;
		
}

/**
 * @brief  电机输出限幅。
 */
static void MotorOutputLimit(Motor *motor)
{
    if ((&motor->Data)->Output > motor->Param.CurrentLimit)
        (&motor->Data)->Output = motor->Param.CurrentLimit;
    else if ((&motor->Data)->Output < (-motor->Param.CurrentLimit))
        (&motor->Data)->Output = (-motor->Param.CurrentLimit);
}

/**
 * @brief  针对C610和C620电调的控制ID，将待发送数据填入CAN发送缓存区的函数指针。
 */
static uint8_t CAN_fill_3508_2006_data(CAN_Object can, MotorData motor_data, uint16_t id)
{
    if (can.Handle == &hfdcan1) {
        if (id >= 0x201 && id <= 0x204) {
            txBuffer0x200forCAN1.Data[(id - 0x201) * 2]     = motor_data.Output >> 8;
            txBuffer0x200forCAN1.Data[(id - 0x201) * 2 + 1] = motor_data.Output & 0xff;
        } else if (id >= 0x205 && id <= 0x208) {
            txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2]     = motor_data.Output >> 8;
            txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;
        }
    } else if (can.Handle == &hfdcan2) {
        if (id >= 0x201 && id <= 0x204) {
            txBuffer0x200forCAN2.Data[(id - 0x201) * 2]     = motor_data.Output >> 8;
            txBuffer0x200forCAN2.Data[(id - 0x201) * 2 + 1] = motor_data.Output & 0xff;
        } else if (id >= 0x205 && id <= 0x208) {
            txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2]     = motor_data.Output >> 8;
            txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;
        } else if (id >= 0x209 && id <= 0x20B) {
            txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2]     = motor_data.Output >> 8;
            txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2 + 1] = motor_data.Output & 0xff;
        }
    }
    return 0;
}

/**
 * @brief  针对GM6020电调的控制ID，将待发送数据填入CAN发送缓存区的函数指针。
 */
static uint8_t CAN_fill_6020_data(CAN_Object can, MotorData motor_data, uint16_t id)
{
    if (can.Handle == &hfdcan1) {
        if (id >= 0x205 && id <= 0x208) {
            txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2]     = motor_data.Output >> 8;
            txBuffer0x1FFforCAN1.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;
        } else if (id >= 0x209 && id <= 0x20B) {
            txBuffer0x2FFforCAN1.Data[(id - 0x209) * 2]     = motor_data.Output >> 8;
            txBuffer0x2FFforCAN1.Data[(id - 0x209) * 2 + 1] = motor_data.Output & 0xff;
        }
    } else if (can.Handle == &hfdcan2) {
        if (id >= 0x205 && id <= 0x208) {
            txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2]     = motor_data.Output >> 8;
            txBuffer0x1FFforCAN2.Data[(id - 0x205) * 2 + 1] = motor_data.Output & 0xff;
        } else if (id >= 0x209 && id <= 0x20B) {
            txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2]     = motor_data.Output >> 8;
            txBuffer0x2FFforCAN2.Data[(id - 0x209) * 2 + 1] = motor_data.Output & 0xff;
        }
    }
    return 0;
}

/**
 * @brief  电机数据更新回调函数，只在motor.c文件内调用。（大疆电机反馈报文格式相同）
 */
static uint8_t CAN_update_data(MotorData *motor, CAN_RxBuffer rxBuffer)
{
    motor->LastEcd       = motor->Ecd; //< 更新编码器角度前记录上个周期的编码器角度
    motor->RawEcd        = rxBuffer.Data[0] << 8 | rxBuffer.Data[1];
    motor->SpeedRPM      = rxBuffer.Data[2] << 8 | rxBuffer.Data[3];
    motor->TorqueCurrent = rxBuffer.Data[4] << 8 | rxBuffer.Data[5];
    motor->Temperature   = rxBuffer.Data[6];
    motor->Ecd           = motor->RawEcd;

    return 0;
}

/**
 * @brief 注册电机设备到CAN设备链表上
 */
static void CAN_RegisteMotor(CAN_Object *canx, Motor *motor)
{
    list_add(&motor->list, (&canx->DevicesList));
}

/**
 * @brief 将电机结构体从CAN设备表上删除
 */
void CAN_DeleteMotor(Motor *motor)
{
    list_del(&(motor->list)); //< 判断无误后从链表中删除该设备
}

/**
 * @brief  电机初始化，设置静态参数，包括编码器零位，电机类型和id。
 */
void MotorInit(Motor *motor, uint16_t ecd_Offset, MotorType type, CanNumber canx, uint16_t id)
{
    (&motor->Param)->EcdOffset = ecd_Offset;
    (&motor->Param)->MotorType = type;
    (&motor->Param)->CanId     = id;
    (&motor->Param)->CanNumber = canx;

    if (canx == CAN1)
        CAN_RegisteMotor(&can1, motor);
    else if (canx == CAN2)
        CAN_RegisteMotor(&can2, motor);

    switch (type) {
        case Motor3508: {
            (&motor->Param)->CurrentLimit = CURRENT_LIMIT_FOR_3508;
            (&motor->Param)->EcdFullRange = ECD_RANGE_FOR_3508;
            motor->MotorUpdate            = CAN_update_data;
            motor->FillMotorData          = CAN_fill_3508_2006_data;
            break;
        }
        case Motor6020: {
            (&motor->Param)->CurrentLimit = CURRENT_LIMIT_FOR_6020;
            (&motor->Param)->EcdFullRange = ECD_RANGE_FOR_6020;
            motor->MotorUpdate            = CAN_update_data;
            motor->FillMotorData          = CAN_fill_6020_data;
            break;
        }
        case Motor2006: {
            (&motor->Param)->CurrentLimit = CURRENT_LIMIT_FOR_2006;
            (&motor->Param)->EcdFullRange = ECD_RANGE_FOR_2006;
            motor->MotorUpdate            = CAN_update_data;
            motor->FillMotorData          = CAN_fill_3508_2006_data;
            break;
        }
        default:;
    }
}

/**
 * @brief  根据canID在设备链表中寻找对应的电机
 */
static Motor *MotorFind(uint16_t canid, CAN_Object canx)
{
    Motor *motor = NULL;
    list_t *node = NULL;

    for (node = canx.DevicesList.next; //< 对循环链表遍历一圈
         node != (canx.DevicesList.prev->next);
         node = node->next) {
        motor = list_entry(node, Motor, list); //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
        if (motor->Param.CanId == canid) {
            return motor;
        }
    }
    return NULL;
}

/**
 * @brief  电机接收回调业务逻辑, 更新电机动态数据，进行编码器角度变换
 */
void MotorRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer)
{
    uint32_t id;
    Motor *temp_motor = NULL;
    id                = rxBuffer.Header.Identifier;
    temp_motor        = MotorFind(id, canx);
    if (temp_motor != NULL) {
    
					temp_motor->Data.Online_check.Cnt++;
		      temp_motor->Data.Online_check.StatusCnt=0;
        temp_motor->MotorUpdate(&temp_motor->Data, rxBuffer);
        MotorEcdtoAngle(temp_motor);
    }
}

/**
 * @brief 获得电机结构体中的ID
 */
uint16_t MotorReturnID(Motor motor)
{
    return motor.Param.CanId;
}

/**
 * @brief  将motor_data.Output限幅后填入发送缓存区等待发送。
 */
int16_t amp_test;
void MotorFillData(Motor *motor, int32_t output)
{
    motor->Data.Output = output;
    MotorOutputLimit(motor);
    amp_test = motor->Data.Output;

    if (motor->Param.CanNumber == CAN1)
        motor->FillMotorData(can1, motor->Data, motor->Param.CanId);
    else if (motor->Param.CanNumber == CAN2)
        motor->FillMotorData(can2, motor->Data, motor->Param.CanId);
}

/**
 * @brief  将特定ID的CAN_TxBuffer发送出去。
 */
uint16_t MotorCanOutput(CAN_Object can, int16_t IDforTxBuffer)
{
    switch (IDforTxBuffer) {
        case 0x200: {
            if (can.Handle == &hfdcan1)
                CAN_Send(&can, &txBuffer0x200forCAN1);
            else if (can.Handle == &hfdcan2)
                CAN_Send(&can, &txBuffer0x200forCAN2);
            break;
        }
        case 0x1ff: {
            if (can.Handle == &hfdcan1)
                CAN_Send(&can, &txBuffer0x1FFforCAN1);
            else if (can.Handle == &hfdcan2)
                CAN_Send(&can, &txBuffer0x1FFforCAN2);
            break;
        }
        case 0x2ff: {
            if (can.Handle == &hfdcan1)
                CAN_Send(&can, &txBuffer0x2FFforCAN1);
            else if (can.Handle == &hfdcan2)
                CAN_Send(&can, &txBuffer0x2FFforCAN2);
            break;
        }
        default:;
    }
    return 0;
}
/**
  * @brief  CAN1接收中断回调
  */
uint8_t CAN1_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can1, (*rxBuffer)); 
	return 0;
}
/**
  * @brief  CAN2接收中断回调
  */
uint8_t CAN2_rxCallBack(CAN_RxBuffer* rxBuffer)
{
	MotorRxCallback(can2, (*rxBuffer)); 	
	return 0;
}
