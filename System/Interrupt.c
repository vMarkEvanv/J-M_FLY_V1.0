#include "stm32f10x.h"                  // Device header
#include "Interrupt.h"
#include "sys.h"
#include "oled.h"
#include "ICM_42688_P.h"
#include "bmp280.h"
extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern TEMP Temp;
extern BMP_280 bmp280;
void EXTI_INT(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;// | GPIO_Pin_1
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
		
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;// | EXTI_Line1;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

//		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//		NVIC_Init(&NVIC_InitStructure);
}
void EXTI0_IRQHandler(void)
{
		if (EXTI_GetITStatus(EXTI_Line0) == SET)
		{
			/*如果出现数据乱跳的现象，可再次判断引脚电平，以避免抖动*/
			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)
			{
				PCout(13)=~PCout(13);
				GYRO_ACC_TEMP_GET();
				printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",Acc_Get.X,Acc_Get.Y,Acc_Get.Z,Gyro_Get.X,Gyro_Get.Y,Gyro_Get.Z);
//				OLED_ShowNum(0, 0, Acc_Get.X, 5, 8, 1);
//				OLED_ShowNum(0,8,  Acc_Get.Y, 5, 8, 1);
//				OLED_ShowNum(0,16,  Acc_Get.Z, 5, 8, 1);
//				OLED_ShowNum(1, 7, Gyro_Get.X, 5, 4, 1);
//				OLED_ShowNum(2, 7, Gyro_Get.Y, 5, 4, 1);
//				OLED_ShowNum(3, 7, Gyro_Get.Z, 5, 4, 1);
//				OLED_ShowNum(1, 1, Temp.T, 5, 4, 1);
//				OLED_ShowNum(0,24,  (int)bmp280.asl, 5, 8, 1);
//				OLED_Refresh();
			}
			EXTI_ClearITPendingBit(EXTI_Line0);
		}
}
