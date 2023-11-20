#include "stm32f10x.h"                  // Device header
#include "sys.h"
#include "Timer.h"
void TIM_Interrupt_Init(TIM_TypeDef* TIMx, unsigned int arr, unsigned int psc)
{
	if(TIMx==TIM1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	}
	else if(TIMx==TIM2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
	else if(TIMx==TIM3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	}
	else if(TIMx==TIM4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	}
	
	TIM_InternalClockConfig(TIMx);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	
	if(TIMx==TIM1)
	{
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	}
	else if(TIMx==TIM2)
	{
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	}
	else if(TIMx==TIM3)
	{
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	}
	else if(TIMx==TIM4)
	{
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	}
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIMx, ENABLE);
}

//interrupt Service fuction temp
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		 PCout(13)=~PCout(13);
	}
}















