#include "stm32h7xx.h"                  // Device header
uint8_t SoftResetFlag=0;

void SoftReset(void)					//�����λ
{	
	__set_FAULTMASK(1); 				//�ر������ж� 
	 NVIC_SystemReset(); 				//��λ
}
