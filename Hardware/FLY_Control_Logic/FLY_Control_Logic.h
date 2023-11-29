#ifndef __FLY_CONTROL_LOGIC_H
#define __FLY_CONTROL_LOGIC_H


typedef struct{
	double X;
	double Y;
	double Z;
}FIX_VALUE;
typedef struct{
	int M0;
	int M1;
	int M2;
	int M3;
}M_PWM;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}PID;

void FLY_BIOS_INIT(void);
void Attitude_Calculate(void);
void Load_Attu_PID(double pitch, double row, double yaw, double goal_pitch, double goal_row, double goal_yaw);

void TIM4_Interrupt_Init(unsigned int arr, unsigned int psc);
void TIM4_IRQHandler(void);
#endif
