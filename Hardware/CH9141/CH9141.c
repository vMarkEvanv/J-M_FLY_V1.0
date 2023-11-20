#include "CH9141.h"
#include "stm32f10x.h"                  // Device header
#include "sys.h"
#include "stdio.h"	
#include "delay.h"
#include "stdarg.h"	 	 	 
#include "string.h"	 
#include "usart.h"
#include "stdlib.h"

#define CH9141_AT PAout(5)
#define CH9141_SLEEP PAout(4)

#define CH9141_RTS PBin(14)
#define CH9141_CTS PBin(13)

/**************************************************/
/*函数名：CH9141_Port_Init;***********************/
/*功能：初始化硬件IO通道;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void CH9141_Port_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//先使能外设IO PORTB时钟 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
  GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);						 //PB10,PB11 输出高	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
	PBout(14)=1;
	CH9141_SLEEP = 0;
	CH9141_AT = 0; 
 }


 /**************************************************/
/*函数名：CH9141_Init;***********************/
/*功能：初始化蓝牙设置;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void CH9141_Init(){
	CH9141_SLEEP = 1;
	CH9141_AT = 1; 
 }