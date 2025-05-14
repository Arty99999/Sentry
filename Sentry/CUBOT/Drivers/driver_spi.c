/**
 **********************************************************************************
 * @file        driver_spi.c
 * @brief       驱动层，面向SPI从机的初始化与配置流程，后续参考HNU的开源对代码进行了修改
 * @details
 * @date        2024-07-26
 * @version     V1.0
 * @copyright   Copyright CUMT-CUBOT (c) 2024
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2024-07-26   <td>1.0         <td>EmberLuo    <td>创建初始版本
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    1. Include the "driver_spi.h" header file in your project.
        在你的项目中包含"driver_spi.h"头文件。
    2. Create an instance of SPI_Slave_t for each SPI slave device you want to communicate with.
        为要与之通信的每个SPI从设备创建一个SPI_Slave_t实例。
    3. Initialize each SPI slave using the SPI_Slavelnit() function, providing the necessary parameters.
        使用SPI_Slavelnit()函数初始化每个SPI从属，提供必要的参数，如SPI句柄、从属配置和接收回调函数。
    4. Set the SPI mode for each slave using SPL_SetMode(), choosing between DMA, interrupt, or blocking modes.
        使用SPL_SetMode()设置每个从机的SPI模式，在DMA、中断或阻塞模式之间进行选择。
    5. Use the provided functions to communicate with the SPI slaves:SPI_Transmit() to send data SPI_Recv() to receive data SPI_TransRecv() for fullduplex communication.
        使用提供的函数与SPI从站通信:SPI_Transmit()发送数据SPI_Recv()接收数据SPI_TransRecv()进行全双工通信
    6. Implement the necessary callback functions to handle received data or transmission completion events.
        实现必要的回调函数来处理接收到的数据或传输完成事件。

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
 * @exception   SPI_NSS有两种模式，SPI_NSS_Hard和SPI_NSS_Soft。
 *              SPI_NSS_Hard，硬件自动拉高拉低片选，在速率上是远比软件方式控制要高的，
 *              缺点是当STM32为主设备时，同一个SPI上面只能接一个从设备。
 *              SPI_NSS_Soft，软件控制，GPIO控制片选拉高拉低，优点是一个SPI上面可以挂多个设备。
 *              SPI_NSS_Soft也是使用较多的方式, 本驱动暂时只考虑这个模式。
 * @todo        考虑将从机编入链表
 **********************************************************************************
 */

#include "driver_spi.h"
SPI_Instance_t SPI_t[6]={{.spiHandler=&hspi1,.DevicesList={&(SPI_t[0].DevicesList),&(SPI_t[0].DevicesList)}}};

void SPI_RegisterSlave(SPI_Instance_t *master, SPI_Slave_t *slave)
{
   list_add(&slave->list, (&master->DevicesList));
	slave->spiHandler=master->spiHandler;
}