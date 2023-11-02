#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "ICM_42688_P.h"
#include "sys.h"
#include "delay.h"
#include "Interrupt.h"
extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern TEMP Temp;
int main(void)
{
	delay_init();
	OLED_Init();
	OLED_Clear();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	ICM_Port_Init();
	//while(ICM_Gyroscope_Reset()==1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	PAout(1)=1;
	EXIT_INT();
	ICM_INIT();
	
	GYRO_ACC_TEMP_GET();
	while(1)
	{
		
		
		//GPIO_ResetBits(GPIOA,GPIO_Pin_1);

	}
}
