
#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"   
#include "stdint.h"
void TIM_Interrupt_Init(TIM_TypeDef* TIMx, unsigned int arr, unsigned int psc);
void TIM1_IRQHandler(void);
#endif
