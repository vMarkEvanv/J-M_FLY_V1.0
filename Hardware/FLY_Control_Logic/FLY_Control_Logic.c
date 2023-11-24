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
#include "math.h"
#include "Motor_Ctrl.h"

extern GYRO Gyro_Get;
extern ACC Acc_Get;
extern ACC Acc_Last;

extern BMP_280 bmp280;

unsigned int trust_val = 0;

M_PWM m_pwm;
M_PWM pitch_pwm;
M_PWM base_pwm;
M_PWM row_pwm;
M_PWM yaw_pwm;



/******************************************************************/
/*函数名：FLY_BIOS_INIT;***************************************/
/*功能：飞控基本传感器初始化;*************/
/*输入：无;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
void FLY_BIOS_INIT(){
	//指示灯初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	PCout(13)=1;
	
	//蓝牙调试器初始化(未启动工作)
	CH9141_Port_Init();
	
	//姿态传感初始化
	ICM_Port_Init();
	ICM_INIT();
	
	//气压计初始化
	IIC_Init();
	//while(!bmp280Init());
	
	//串口选择器初始化
	SELECT_Port_INIT();
	
	//选择源串口初始化
	UART_Init(115200);
	
	//黑匣子初始化
	SD_Init();
	
//	FIXED_VALUE.X =0;
//	FIXED_VALUE.Y =0;
//	FIXED_VALUE.Z =0;

//	double temp_x = 0;
//	double temp_y = 0;
//	double temp_z = 0;
//	
//	int n = 10000;
//	while(n--){
//		GYRO_ACC_TEMP_GET();
//		temp_x += Gyro_Get.X;
//		temp_y += Gyro_Get.Y;
//		temp_z += Gyro_Get.Z;
//	}
//	FIXED_VALUE.X = temp_x/10000.0;
//	FIXED_VALUE.Y = temp_y/10000.0;
//	FIXED_VALUE.Z = temp_z/10000.0;
	
	
// 	while(SD_Init())//检测不到SD卡
//	{
//		PCout(13)=~PCout(13);
//	}
	//CH9141_Init();
	//CH9141_EN();
	FLY_PWM_Port_Init();
	base_pwm.M0=50;
	base_pwm.M1=50;
	base_pwm.M2=50;
	base_pwm.M3=50;
	Set_Duty(0,0,0,0);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
	delay_ms(500);
}

#define pi 3.1415926
double toDegrees(double radians) {
    return radians * (180.0 / pi);
}
/******************************************************************/
/*函数名：ICM_Gyroscope_INIT;***************************************/
/*功能：ICM陀螺仪初始化;*************/
/*输入：无;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
void Attitude_Calculate(){
//	if(attu.pitch > 180.0){
//		attu.pitch = attu.pitch - 360.0;
//	}
//	else if(attu.pitch <= -180.0){
//		attu.pitch = 360.0 + attu.pitch;
//	}
//	
//	if(attu.row > 180.0){
//		attu.row = attu.row - 360.0;
//	}
//	else if(attu.row <= -180.0){
//		attu.row = 360.0 + attu.row;
//	}
//	
//	if(attu.yaw > 180.0){
//		attu.yaw = attu.yaw - 360.0;
//	}
//	else if(attu.yaw <= -180.0){
//		attu.yaw = 360.0 + attu.yaw;
//	}
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
		//PCout(13) = ~PCout(13);
		GYRO_ACC_TEMP_GET();
		Attitude_Calculate();
		
//		pitch_pwm.M0 =  -30*attu.pitch*base_pwm.M0/180.0;
//		pitch_pwm.M1 =  30*attu.pitch*base_pwm.M1/180.0;
		m_pwm.M0 = base_pwm.M0 + pitch_pwm.M0;
		m_pwm.M1 = base_pwm.M1 + pitch_pwm.M1;
		
		if(m_pwm.M0<0){
			m_pwm.M0 = 0;
		}
		if(m_pwm.M1<0){
			m_pwm.M1 = 0;
		}
		printf("%d,%d\n",m_pwm.M0,m_pwm.M1);
		Set_Duty(m_pwm.M0,m_pwm.M1,m_pwm.M2,m_pwm.M3);
		
	}
}