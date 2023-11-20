#include "stm32f10x.h"                  // Device header
#include "FLY_Control_Logic.h"
#include "ICM_42688_P.h"
#include "bmp280.h"
#include "myiic.h"
#include "UART_Select.h"
#include "sdio_sdcard.h"   
#include "CH9141.h"
#include "bmp280.h"
#include "sys.h"
extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern TEMP Temp;
extern BMP_280 bmp280;
ATTU attu;
ATTU last_attu;
ATTU delta_attu;

void FLY_BIOS_INIT(){
	//ָʾ�Ƴ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	PCout(13)=1;
	
	//������������ʼ��(δ��������)
	CH9141_Port_Init();
	
	//��̬���г�ʼ��
	ICM_Port_Init();
	ICM_INIT();
	
	//��ѹ�Ƴ�ʼ��
	IIC_Init();
	while(!bmp280Init());
	
	//����ѡ������ʼ��
	SELECT_Port_INIT();
	
	//ѡ��Դ���ڳ�ʼ��
	UART_Init(115200);
	
	//��ϻ�ӳ�ʼ��
	SD_Init();
// 	while(SD_Init())//��ⲻ��SD��
//	{
//		PCout(13)=~PCout(13);
//	}
	

}


void Attitude_Calculate(){
	attu.X = Gyro_Get.X;
}

void TIM4_Interrupt_Init(unsigned int arr, unsigned int psc)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_InternalClockConfig(TIM4);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM4, ENABLE);
	
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		PCout(13) = ~PCout(13);
		bmp280GetData(&bmp280.pressure,&bmp280.temperature,&bmp280.asl);
		GYRO_ACC_TEMP_GET();
		printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",Acc_Get.X,Acc_Get.Y,Acc_Get.Z,Gyro_Get.X,Gyro_Get.Y,Gyro_Get.Z);
	}
}