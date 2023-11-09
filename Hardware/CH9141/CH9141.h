#ifndef __CH9141_H
#define __CH9141_H
#include "stdio.h"	
#include "sys.h" 

#define USART3_MAX_RECV_LEN		60					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		600					//����ͻ����ֽ���
#define USART3_RX_EN 			1					//0,������;1,����.
 
extern u8  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern vu16 USART3_RX_STA;   						//��������״̬


void USART3_RX_Data(void);
void CH9141_Port_Init(void);
void CH9141_Init(void);
void CH9141_UART_Init(u32 bound);
void CH9141_Init(void);
#endif
