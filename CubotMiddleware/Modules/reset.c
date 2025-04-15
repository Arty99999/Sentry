#include "stm32h7xx.h"                  // Device header
uint8_t SoftResetFlag=0;

void SoftReset(void)					//软件复位
{	
	__set_FAULTMASK(1); 				//关闭所有中断 
	 NVIC_SystemReset(); 				//复位
}
