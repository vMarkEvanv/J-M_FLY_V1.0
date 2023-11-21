#include "stm32f10x.h"                  // Device header
#include "oled.h"
#include "ICM_42688_P.h"
#include "sys.h"
#include "delay.h"
#include "Interrupt.h"

#include "bmp280.h"
#include "usart.h"
#include "CH9141.h"
#include "FLY_Control_Logic.h"
#include "bmp.h"


int main(void)
{
	delay_init();
	uart_init(115200);
	FLY_BIOS_INIT();
	
	OLED_Init();
	OLED_ColorTurn(0);//0正常显示，1 反色显示
  OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
	OLED_Refresh();
	OLED_ShowPicture(0,0,128,32,BMP2,1);
	OLED_Refresh();
	TIM4_Interrupt_Init(20,7200);
	while(1)
	{
		
		
		

	}
}


