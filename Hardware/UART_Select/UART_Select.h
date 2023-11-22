#ifndef __UART_SELECT_H
#define __UART_SELECT_H
#include "stdio.h"	
#include "sys.h" 
#define USART3_MAX_RECV_LEN		60					//最大接收缓存字节数
#define USART3_MAX_SEND_LEN		600					//最大发送缓存字节数
#define USART3_RX_EN 			1					//0,不接收;1,接收.

#define OTHER_EN() {GPIO_SetBits(GPIOA,GPIO_Pin_6);GPIO_ResetBits(GPIOA,GPIO_Pin_7);GPIO_ResetBits(GPIOB,GPIO_Pin_1);}
#define CH9141_EN() {GPIO_SetBits(GPIOA,GPIO_Pin_7);GPIO_ResetBits(GPIOA,GPIO_Pin_6);GPIO_ResetBits(GPIOB,GPIO_Pin_1);}
#define OFFCIRCLE_EN() {GPIO_SetBits(GPIOB,GPIO_Pin_1);GPIO_ResetBits(GPIOA,GPIO_Pin_7);GPIO_ResetBits(GPIOA,GPIO_Pin_6);}
	
extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//发送缓冲,最大USART3_MAX_SEND_LEN字节
extern vu16 USART3_RX_STA;   						//接收数据状态

void SELECT_Port_INIT(void);
void UART_Init(u32 bound);
void USART3_RX_Data(void);
#endif
