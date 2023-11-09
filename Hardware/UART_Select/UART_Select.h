#ifndef __UART_SELECT_H
#define __UART_SELECT_H

#define OTHER_EN() {GPIO_ResetBits(GPIOA,GPIO_Pin_6);GPIO_SetBits(GPIOA,GPIO_Pin_7);GPIO_SetBits(GPIOB,GPIO_Pin_1);}
#define CH9141_EN() {GPIO_ResetBits(GPIOA,GPIO_Pin_7);GPIO_SetBits(GPIOA,GPIO_Pin_6);GPIO_SetBits(GPIOB,GPIO_Pin_1);}
#define OFFCIRCLE_EN() {GPIO_ResetBits(GPIOB,GPIO_Pin_1);GPIO_SetBits(GPIOA,GPIO_Pin_7);GPIO_SetBits(GPIOA,GPIO_Pin_6);}
	
void SELECT_Port_INIT(void);

#endif
