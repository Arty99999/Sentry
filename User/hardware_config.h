#ifndef HARDWARE_CONFIG_H_
#define HARDWARE_CONFIG_H_
#include "driver.h"
#include "user_lib.h"
#define LIMIT(LIMIT_x, x_min, x_max)        ((LIMIT_x) < (x_min) ? (x_min) : ((LIMIT_x) > (x_max) ? (x_max) : (LIMIT_x))) 

/**
  * @brief  ���ʣ������������ã�����������
  */
void HardwareConfig(void);


#endif



