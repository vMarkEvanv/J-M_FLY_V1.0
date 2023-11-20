#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "ICM_42688_P.h"
#include "sys.h"
#include "delay.h"
#include "Interrupt.h"
#include "myiic.h"
#include "bmp280.h"
#include "usart.h"
#include "CH9141.h"
#include "UART_Select.h"

extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern TEMP Temp;
extern BMP_280 bmp280;
int main(void)
{
	delay_init();
	uart_init(115200);
	SELECT_Port_INIT();
	CH9141_Port_Init();
	CH9141_UART_Init(115200);
	CH9141_EN();
	CH9141_Init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	PCout(13)=1;
	//OLED_Init();
	//OLED_ShowChar(1,1,'a');
	//PAout(1)=1;
	ICM_Port_Init();
	EXIT_INT();
	ICM_INIT();
	
	IIC_Init();
	while(!bmp280Init());

	
	//GYRO_ACC_TEMP_GET();
	//OLED_ShowSignedNum(1, 1, 1234, 5);
	while(1)
	{
		//printf("123");
		bmp280GetData(&bmp280.pressure,&bmp280.temperature,&bmp280.asl);
		

	}
}

//#include "sys.h"
//#include "delay.h"
//#include "usart.h"  		 	 
//#include "sdio_sdcard.h"   
// 
////通过串口打印SD卡相关信息
//void show_sdcard_info(void)
//{
//	switch(SDCardInfo.CardType)
//	{
//		case SDIO_STD_CAPACITY_SD_CARD_V1_1:printf("Card Type:SDSC V1.1\r\n");break;
//		case SDIO_STD_CAPACITY_SD_CARD_V2_0:printf("Card Type:SDSC V2.0\r\n");break;
//		case SDIO_HIGH_CAPACITY_SD_CARD:printf("Card Type:SDHC V2.0\r\n");break;
//		case SDIO_MULTIMEDIA_CARD:printf("Card Type:MMC Card\r\n");break;
//	}	
//  	printf("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//制造商ID
// 	printf("Card RCA:%d\r\n",SDCardInfo.RCA);								//卡相对地址
//	printf("Card Capacity:%d MB\r\n",(u32)(SDCardInfo.CardCapacity>>20));	//显示容量
// 	printf("Card BlockSize:%d\r\n\r\n",SDCardInfo.CardBlockSize);			//显示块大小
//}  
// int main(void)
// {


//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOC,GPIO_Pin_13);
//	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//	PCout(13)=1;	 
//	u8 key;		 
//	u32 sd_size;
//	u8 t=0;	
//	u8 *buf=0;
//  

//	delay_init();	    	 //延时函数初始化	  
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
//	uart_init(115200);	 	//串口初始化为115200 
// 	while(SD_Init())//检测不到SD卡
//	{
//		PCout(13)=~PCout(13);
//	}
//	show_sdcard_info();	//打印SD卡相关信息
//	while(1)
//	{
//		
//	}   
//}
