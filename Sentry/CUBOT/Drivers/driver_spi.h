#ifndef _DRIVER_SPI_H_
#define _DRIVER_SPI_H_

#include "gpio.h"
#include "spi.h"
#include <stdlib.h>
#include <stdint.h>
#include "linux_list.h" 

enum SPI_LIST {
    DEVICE_SPI1,
    DEVICE_SPI2,
    DEVICE_SPI3,
    DEVICE_SPI4,
    DEVICE_SPI5,
    DEVICE_SPI6,
    SPI_DEVICE_NUM,
};

/**
 * @brief SPI片选信号对应的GPIO引脚号
 */
typedef struct
{
    GPIO_TypeDef *gpiox; // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;     // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
} SPI_Chip_Select_t;

/**
 * @brief   SPI接收缓冲区
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} SPI_RxBuffer_t;

/**
 * @brief	SPI发送缓冲区
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} SPI_TxBuffer_t;

/**
 * @brief SPI从机结构体
 */
typedef struct _SPI_Slave_t {
    SPI_RxBuffer_t rxBuffer;                      // 接收缓存区结构体
    SPI_TxBuffer_t txBuffer;                      // 发送缓存区结构体
    SPI_Chip_Select_t chipSelect;                 // 片选GPIO引脚结构体
    void (*RxCallBackSPI)(struct _SPI_Slave_t *); // 接收回调函数
	  list_t   list;
	  SPI_HandleTypeDef* spiHandler;
} SPI_Slave_t;

/**
 * @brief SPI实例结构体
 */
typedef struct _SPI_Instance_t {
	  SPI_HandleTypeDef* spiHandler; 
	  list_t   DevicesList;
} SPI_Instance_t;

/**
 * @brief   SPI回调函数
 */
typedef void (*SPI_RxCallback)(SPI_Slave_t *);


void SPI_RegisterSlave(SPI_Instance_t *master, SPI_Slave_t *slave);



extern SPI_Instance_t SPI_t[6];

#endif

