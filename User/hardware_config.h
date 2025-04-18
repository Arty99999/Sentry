#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_
#include "driver.h"
#include "user_lib.h"
#define LIMIT(LIMIT_x, x_min, x_max)        ((LIMIT_x) < (x_min) ? (x_min) : ((LIMIT_x) > (x_max) ? (x_max) : (LIMIT_x))) 

/**
  * @brief  完成剩余的驱动器配置，开启驱动器
  */
void HardwareConfig(void);


#endif



