#ifndef __FLY_CONTROL_LOGIC_H
#define __FLY_CONTROL_LOGIC_H

typedef struct{
	double X;
	double Y;
	double Z;
}ATTU;


void FLY_BIOS_INIT(void);
void TIM4_Interrupt_Init(unsigned int arr, unsigned int psc);
void TIM4_IRQHandler(void);
#endif
