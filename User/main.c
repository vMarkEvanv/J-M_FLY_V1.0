#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "ICM_42688_P.h"
#include "sys.h"
#include "delay.h"
#include "Interrupt.h"
#include "myiic.h"
#include "bmp280.h"
#include "usart.h"
extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern TEMP Temp;
extern BMP_280 bmp280;
int main(void)
{
	delay_init();
	uart_init(115200);
	
	
	OLED_Init();
	OLED_Clear();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	ICM_Port_Init();
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	PCout(13)=1;
	//PAout(1)=1;
	EXIT_INT();
	ICM_INIT();
	IIC_Init();
	bmp280Init();
	GYRO_ACC_TEMP_GET();
	//OLED_ShowSignedNum(1, 1, 1234, 5);
	while(1)
	{
		bmp280GetData(&bmp280.pressure,&bmp280.temperature,&bmp280.asl);
		//printf("%.2f,%.2f,%.2f\r\n",bmp280.pressure,bmp280.temperature,bmp280.asl);
		
		//GPIO_ResetBits(GPIOA,GPIO_Pin_1);

	}
}
