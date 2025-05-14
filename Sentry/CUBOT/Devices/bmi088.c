/**
 **********************************************************************************
 * @file        bmi088.c
 * @brief       应用层，参考
 * @details
 * @date        2024-07-30
 * @version     V1.0
 * @copyright   Copyright CUMT-CUBOT (c) 2024
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2024-07-30   <td>1.0         <td>EmberLuo    <td>创建初始版本
 * </table>
 *
 **********************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bmi088.h"
#include "user_lib.h"
BMI088_t bmi088 =
    {
        .bmi088Accel =
            {
                .spiHandler = &hspi1,
                .chipSelect =
                    {
                        .cs_pin = GPIO_PIN_2,
                        .gpiox = GPIOD, // 紫板引脚

                        //										    .cs_pin = GPIO_PIN_2,
                        //                        .gpiox  = GPIOD,//外置小板引脚
                    },
                .RxCallBackSPI = bmi088Accel_CallBack,

            },
        .bmi088Gyro =
            {
                .spiHandler = &hspi1,
                .chipSelect =
                    {
                        .cs_pin = GPIO_PIN_4,
                        .gpiox = GPIOA, // 紫板引脚

                        //											 .cs_pin = GPIO_PIN_4,
                        //                        .gpiox  = GPIOA,//外置小板引脚

                    },
                .RxCallBackSPI = bmi088Gyro_CallBack,

            },
        .bmi088_Data = {.Init = BMI088_Init,
                        .Read = BMI088_Read},
        .bmi088_Data.mode = IT_MODE};
float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;

int16_t bmi088_cali_count = 0;

static void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(bmi088.bmi088Accel.chipSelect.gpiox, bmi088.bmi088Accel.chipSelect.cs_pin, GPIO_PIN_RESET);
}
static void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(bmi088.bmi088Accel.chipSelect.gpiox, bmi088.bmi088Accel.chipSelect.cs_pin, GPIO_PIN_SET);
}

static void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(bmi088.bmi088Gyro.chipSelect.gpiox, bmi088.bmi088Gyro.chipSelect.cs_pin, GPIO_PIN_RESET);
}
static void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(bmi088.bmi088Gyro.chipSelect.gpiox, bmi088.bmi088Gyro.chipSelect.cs_pin, GPIO_PIN_SET);
}

static uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(SPI_t[0].spiHandler, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len, Device_MODE mode)
{
    static uint8_t b[30];
    BMI088_ACCEL_NS_L();
    b[0] = (reg) | 0x80;
    b[1] = (reg) | 0x80;
    for (int i = 0; i < len; i++)
        b[i + 2] = 0x55;
    if (mode == BLOCK_MODE)
    {
        HAL_SPI_TransmitReceive(SPI_t[0].spiHandler, b, data, len + 2, 1000);
        BMI088_ACCEL_NS_H();
    }
    else if (mode == IT_MODE)
        HAL_SPI_TransmitReceive_IT(SPI_t[0].spiHandler, b, data, len + 2);
}

void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len, Device_MODE mode)
{
    static uint8_t b[30];
    b[0] = (reg) | 0x80;
    for (int i = 0; i < len; i++)
        b[i + 1] = 0x55;
    BMI088_GYRO_NS_L();
    if (mode == BLOCK_MODE)
    {
        HAL_SPI_TransmitReceive(SPI_t[0].spiHandler, b, data, len + 1, 1000);
        BMI088_GYRO_NS_H();
    }
    else if (mode == IT_MODE)
        HAL_SPI_TransmitReceive_IT(SPI_t[0].spiHandler, b, data, len + 1);
}
#elif defined(BMI088_USE_IIC)
#endif

static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void BMI088_Calibrate_Offset(IMU_InitData_t *bmi088_data);

uint8_t BMI088_Init(IMU_InitData_t *bmi088_data)
{
    BMI088_t *bmi088 = (BMI088_t *)bmi088_data;
    SPI_RegisterSlave(&SPI_t[0], &(bmi088->bmi088Accel));
    SPI_RegisterSlave(&SPI_t[0], &(bmi088->bmi088Gyro));
    error = BMI088_NO_ERROR;
    error |= BMI088_AccInit();
    error |= BMI088_GyroInit();
    BMI088_Calibrate_Offset(bmi088_data);
    return error;
}

void BMI088_Calibrate_Offset(IMU_InitData_t *bmi088_data)
{
    static float startTime;
    static uint16_t CaliTimes = 6000;
    static float gyroDiff[3], gNormDiff;
    float g_norm;
    uint8_t buf[10] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = DWT_GetTimeline_s();
    do
    {
        if (DWT_GetTimeline_s() - startTime > 12)
        {
            // 在线校准超时，使用预置值校准
            bmi088_data->gyro_offset[0] = BMI088_GxOFFSET;
            bmi088_data->gyro_offset[1] = BMI088_GyOFFSET;
            bmi088_data->gyro_offset[2] = BMI088_GzOFFSET;
            g_norm = BMI088_gNORM;
            bmi088_data->temp_when_cali = 70;
            break;
        }

        DWT_Delay_s(0.005);
        g_norm = 0;
        bmi088_data->gyro_offset[0] = 0;
        bmi088_data->gyro_offset[1] = 0;
        bmi088_data->gyro_offset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 8, BLOCK_MODE);
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088_data->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088_data->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088_data->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

            gNormTemp = sqrtf(bmi088_data->accel[0] * bmi088_data->accel[0] +
                              bmi088_data->accel[1] * bmi088_data->accel[1] +
                              bmi088_data->accel[2] * bmi088_data->accel[2]);
            g_norm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8, BLOCK_MODE);
            if (buf[1] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[4]) << 8) | buf[3];
                bmi088_data->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088_data->gyro_offset[0] += bmi088_data->gyro[0];
                bmi088_raw_temp = (int16_t)((buf[6]) << 8) | buf[5];
                bmi088_data->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088_data->gyro_offset[1] += bmi088_data->gyro[1];
                bmi088_raw_temp = (int16_t)((buf[8]) << 8) | buf[7];
                bmi088_data->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088_data->gyro_offset[2] += bmi088_data->gyro[2];
            }

            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = bmi088_data->gyro[j];
                    gyroMin[j] = bmi088_data->gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (bmi088_data->gyro[j] > gyroMax[j])
                        gyroMax[j] = bmi088_data->gyro[j];
                    if (bmi088_data->gyro[j] < gyroMin[j])
                        gyroMin[j] = bmi088_data->gyro[j];
                }
            }

            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                //            {
                break;
            //            }

            DWT_Delay_s(0.0005);
        }

        g_norm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            bmi088_data->gyro_offset[i] /= (float)CaliTimes;

        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2, BLOCK_MODE);
        bmi088_raw_temp = (int16_t)((buf[2] << 3) | (buf[3] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088_data->temp_when_cali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        bmi088_cali_count++;

    } while (gNormDiff > 0.5f ||
             fabsf(g_norm - 9.8f) > 0.9f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088_data->gyro_offset[0]) > 0.01f ||
             fabsf(bmi088_data->gyro_offset[1]) > 0.01f ||
             fabsf(bmi088_data->gyro_offset[2]) > 0.01f);
    // 若出不了while，说明校准条件恶劣，超时则使用预置值校准
    bmi088_data->accel_scale = 9.81f / g_norm;
}

uint8_t BMI088_AccInit(void)
{
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay_s(0.001);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay_s(0.001);
    // accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay_s(0.08);
    // check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay_s(0.001);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    DWT_Delay_s(0.001);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(BMI088_Accel_Init_Table[write_reg_num][0], BMI088_Accel_Init_Table[write_reg_num][1]);
        DWT_Delay_s(0.001);

        BMI088_accel_read_single_reg(BMI088_Accel_Init_Table[write_reg_num][0], res);
        DWT_Delay_s(0.001);

        if (res != BMI088_Accel_Init_Table[write_reg_num][1])
        {
            error |= BMI088_Accel_Init_Table[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t BMI088_GyroInit(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay_s(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay_s(0.001);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay_s(0.08);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay_s(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    DWT_Delay_s(0.001);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(BMI088_Gyro_Init_Table[write_reg_num][0], BMI088_Gyro_Init_Table[write_reg_num][1]);
        DWT_Delay_s(0.001);

        BMI088_gyro_read_single_reg(BMI088_Gyro_Init_Table[write_reg_num][0], res);
        DWT_Delay_s(0.001);

        if (res != BMI088_Gyro_Init_Table[write_reg_num][1])
        {
            write_reg_num--;
            error |= BMI088_Accel_Init_Table[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_Read(IMU_InitData_t *bmi088_data, Device_MODE mode)
{
    static uint8_t buf[40];
    static int16_t bmi088_raw_temp;

    if (mode == IT_MODE)
    {
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 34, IT_MODE);

        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088_data->Raw_accel[0] = bmi088_raw_temp;
        bmi088_data->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088_data->accel_scale;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088_data->Raw_accel[1] = bmi088_raw_temp;
        bmi088_data->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088_data->accel_scale;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        bmi088_data->Raw_accel[2] = bmi088_raw_temp;
        bmi088_data->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088_data->accel_scale;

        bmi088_raw_temp = (int16_t)((buf[18] << 3) | (buf[19] >> 5));

        if (bmi088_raw_temp > 1023)
        {
            bmi088_raw_temp -= 2048;
        }

        bmi088_data->temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    else if (mode == BLOCK_MODE)
    {
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6, BLOCK_MODE);

        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        bmi088_data->Raw_accel[0] = bmi088_raw_temp;
        bmi088_data->accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088_data->accel_scale;
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088_data->Raw_accel[1] = bmi088_raw_temp;
        bmi088_data->accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088_data->accel_scale;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088_data->Raw_accel[2] = bmi088_raw_temp;
        bmi088_data->accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088_data->accel_scale;

        BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8, BLOCK_MODE);
        if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {

            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088_data->Raw_gyro[0] = bmi088_raw_temp;
            bmi088_data->gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088_data->gyro_offset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088_data->Raw_gyro[1] = bmi088_raw_temp;
            bmi088_data->gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088_data->gyro_offset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088_data->Raw_gyro[2] = bmi088_raw_temp;
            bmi088_data->gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088_data->gyro_offset[2];
        }
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2, BLOCK_MODE);

        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

        if (bmi088_raw_temp > 1023)
        {
            bmi088_raw_temp -= 2048;
        }

        bmi088_data->temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }
}

#if defined(BMI088_USE_SPI)

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h_spi)
{
    SPI_Instance_t *SPI = NULL;
    SPI_Slave_t *SPI_Slave = NULL;
    list_t *node = NULL;

    for (int i = 0; i < 6; i++)
        if (SPI_t[i].spiHandler == h_spi)
            SPI = &SPI_t[i];

    for (node = SPI->DevicesList.next; //< 对循环链表遍历一圈
         node != (SPI->DevicesList.prev->next);
         node = node->next)
    {
        SPI_Slave = list_entry(node, SPI_Slave_t, list); //< 输入链表头部所在结点、被嵌入链表的结构体类型、被嵌入链表的结构体类型中链表结点的名称：即可返回嵌入头部所在结点的结构体
        if (HAL_GPIO_ReadPin(SPI_Slave->chipSelect.gpiox, SPI_Slave->chipSelect.cs_pin) == GPIO_PIN_RESET)
        {
            SPI_Slave->RxCallBackSPI(SPI_Slave);
        }
    }
}

void bmi088Accel_CallBack(struct _SPI_Slave_t *SPI_Slave)
{
    static uint8_t BUF1[9];
    IMU_InitData_t *bmi088_Data = (IMU_InitData_t *)((char *)SPI_Slave - offsetof(BMI088_t, bmi088Accel));
    BMI088_ACCEL_NS_H();
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, BUF1, 8, IT_MODE);
    for (int i = 0; i < 3; i++)
    {
        bmi088_Data->Raw_gyro[i] = (int16_t)((BUF1[4 + 2 * i]) << 8) | BUF1[3 + 2 * i];
        bmi088_Data->gyro[i] = bmi088_Data->Raw_gyro[i] * BMI088_GYRO_SEN - bmi088_Data->gyro_offset[i];
    }
}

void bmi088Gyro_CallBack(struct _SPI_Slave_t *SPI_Slave)
{

    BMI088_GYRO_NS_H();
}
#elif defined(BMI088_USE_IIC)

#endif
