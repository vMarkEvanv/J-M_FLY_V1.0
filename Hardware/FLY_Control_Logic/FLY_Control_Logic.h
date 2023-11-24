#ifndef __FLY_CONTROL_LOGIC_H
#define __FLY_CONTROL_LOGIC_H



typedef struct{
	int M0;
	int M1;
	int M2;
	int M3;
}M_PWM;

void FLY_BIOS_INIT(void);
void Attitude_Calculate(void);

void TIM4_Interrupt_Init(unsigned int arr, unsigned int psc);
void TIM4_IRQHandler(void);
#endif
