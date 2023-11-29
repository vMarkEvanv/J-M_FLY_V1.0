#include "Motor_Ctrl.h"
#include "stm32f10x.h"                  // Device header
#include "sys.h"
/******************************************************************/
/*函数名：FLY_PWM_Port_Init;***************************************/
/*功能：无人机电机驱动IO初始化;*************/
/*输入：无;********************************************************/
/*输出：无;***************************************/
/******************************************************************/
void FLY_PWM_Port_Init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1440 - 1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	PCout(13)=0;
}

/******************************************************************/
/*函数名：Set_Duty;***************************************/
/*功能：设置IO脉冲输出;*************/
/*输入：M0,M1,M2,M3(0.0~1000.0);********************************************************/
/*输出：无;***************************************/
/******************************************************************/
void Set_Duty(int M0, int M1, int M2, int M3){
	M0 = M0*(50.0/1000.0) + 50.0;
	M1 = M1*(50.0/1000.0) + 50.0;
	M2 = M2*(50.0/1000.0) + 50.0;
	M3 = M3*(50.0/1000.0) + 50.0;

	if(M0 >= 100){
		M0 = 100;
	}
	if(M1 >= 100){
		M1 = 100;
	}
	if(M2 >= 100){
		M2 = 100;
	}
	if(M3 >= 100){
		M3 = 100;
	}
	
	if(M0 <= 50){
		M0 = 50;
	}
	if(M1 <= 50){
		M1 = 50;
	}
	if(M2 <= 50){
		M2 = 50;
	}
	if(M3 <= 50){
		M3 = 50;
	}
	//printf("%d,%d\n",M0,M1);
	TIM_SetCompare1(TIM2, M0);
	TIM_SetCompare2(TIM2, M1);
	TIM_SetCompare3(TIM2, M2);
	TIM_SetCompare4(TIM2, M3);
}
