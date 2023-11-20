#include "ICM_42688_P.h"
#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "sys.h"

unsigned char ICM_42688_Addr_AD0_LOW_READ = 0xD1;   //AD0低电平地址的读
unsigned char ICM_42688_Addr_AD0_HIGH_READ = 0xD3;	 //AD0高电平地址的读
unsigned char ICM_42688_Addr_AD0_LOW_WRITE = 0xD0;	 //AD0低电平地址的写
unsigned char ICM_42688_Addr_AD0_HIGH_WRITE = 0xD2; //AD0高电平地址的写

GYRO Gyro_Get;
ACC Acc_Get;
TEMP Temp;

void IIC_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}
/**************************************************/
/*函数名：ICM_Port_Init;***********************/
/*功能：初始化硬件IIC通道;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void ICM_Port_Init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C1, &I2C_InitStructure);
	
	I2C_Cmd(I2C1, ENABLE);				 

}	  

/******************************************************************/
/*函数名：ICM_IIC_WRITE_BYTE;***************************************/
/*功能：写一个字节;*************/
/*输入：RA：寄存器地址 data_byte:数据;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
unsigned char ICM_IIC_WRITE_BYTE(unsigned char RA, unsigned char data_byte){
	
	I2C_GenerateSTART(I2C1, ENABLE);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, ICM_42688_Addr_AD0_LOW_WRITE, I2C_Direction_Transmitter);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1, RA);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C1, data_byte);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C1, ENABLE);
}

/******************************************************************/
/*函数名：ICM_IIC_READ_BYTE;***************************************/
/*功能：读一个字节;*************/
/*输入：RA：寄存器地址;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
unsigned char ICM_IIC_READ_BYTE(unsigned char RA, unsigned char *data){
		uint8_t Data;
	
	I2C_GenerateSTART(I2C1, ENABLE);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, ICM_42688_Addr_AD0_LOW_WRITE, I2C_Direction_Transmitter);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1, RA);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C1, ENABLE);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1, ICM_42688_Addr_AD0_LOW_WRITE, I2C_Direction_Receiver);
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	IIC_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C1);
	
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	*data = Data;
	return 0;
}

/******************************************************************/
/*函数名：ICM_INIT;***************************************/
/*功能：ICM芯片初始化;*************/
/*输入：无;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
unsigned char ICM_INIT(){
	if(ICM_IIC_WRITE_BYTE(DEVICE_CONFIG,0x00)) return 1;//Software reset configuration and SPI mode selection
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(DRIVE_CONFIG,0x05)) return 1;//Control the communication speed(I guess)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_CONFIG,0x02)) return 1;//interrupt settings
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(PWR_MGMT0,0x0F)) return 1;//power register of sensors(it won't working if we don't turn it on)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_CONFIG1,0x00)) return 1;//this register is to set the interrupt port's Interrupt pulse duration (more details on datasheet)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_SOURCE0,0x08)) return 1;//setting interrupt port's interrupt source
	delay_ms(50);
	if(ICM_Gyroscope_INIT()) return 1;//陀螺仪初始化
	delay_ms(50);
	if(ICM_ACC_INIT()) return 1;//加速度计初始化
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(SELF_TEST_CONFIG,0x1F)) return 1;//自检
	delay_ms(50);
	return 0;
}

/******************************************************************/
/*函数名：ICM_Gyroscope_INIT;***************************************/
/*功能：ICM陀螺仪初始化;*************/
/*输入：无;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
unsigned char ICM_Gyroscope_INIT(){
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG0,0x06)) return 1;//调整采样率和ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG1,0x16)) return 1;//调整带宽和滤波次数
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(GYRO_ACCEL_CONFIG0,0x11)) return 1;//调整陀螺仪和加速度计的低通滤波器带宽
	delay_ms(50);
	return 0;
}

/******************************************************************/
/*函数名：ICM_ACC_INIT;***************************************/
/*功能：ICM加速度计初始化;*************/
/*输入：无;********************************************************/
/*输出：无;***************************************/
/******************************************************************/
unsigned char ICM_ACC_INIT(){
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG0,0x06)) return 1;//调整采样率和ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG1,0x0D)) return 1;//调整带宽和滤波次数
	delay_ms(50);
	return 0;
	
}

/******************************************************************/
/*函数名：GYRO_ACC_TEMP_GET;***************************************/
/*功能：获取陀螺仪，加速度计，温度数据;*************/
/*输入：无;********************************************************/
/*输出：无;***************************************/
/******************************************************************/
unsigned char GYRO_ACC_TEMP_GET(){
	unsigned char temp = 0;
	short Counting_Temp = 0;
	//温度读取
	if(ICM_IIC_READ_BYTE(TEMP_DATA1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(TEMP_DATA0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Temp.T = (Counting_Temp / 132.48) + 25;
	
	//X轴陀螺仪读取
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.X = (Counting_Temp*1.0)/32767.0*2000;
	
	//Y轴陀螺仪读取
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Y = (Counting_Temp*1.0)/32767.0*2000;
	
	//Z轴陀螺仪读取
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Z = (Counting_Temp*1.0)/32767.0*2000;
	
	//X轴加速度计读取 ±16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.X = (Counting_Temp*1.0)/32767.0*16.0*9.8;
	
	//Y轴加速度计读取 ±16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Y = (Counting_Temp*1.0)/32767.0*16.0*9.8;
	
	//Z轴加速度计读取 ±16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Z = (Counting_Temp*1.0)/32767.0*16.0*9.8;
	
	return 0;
}
