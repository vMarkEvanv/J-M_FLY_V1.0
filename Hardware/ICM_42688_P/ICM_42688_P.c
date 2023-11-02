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

void ICM_IIC_Delay(void){
	delay_us(2);
}
/**************************************************/
/*函数名：Port_Init;***********************/
/*功能：初始化硬件IIC通道;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void ICM_Port_Init(){
	//delay_init();
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
  GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);						 //PB10,PB11 输出高	

}	  

/************************************************************************/
/*函数名：ICM_42688_START;***********************************************/
/*功能：通过IIC协议连接ICM_42688_P模块;**********************************/
/*输入：无;**************************************************************/
/*输出：0：开始信号发送成功，并且得到应答 1：未得到应答;*****************/
/************************************************************************/
void ICM_42688_START(){
	ICM_SDA_OUT();
	ICM_IIC_SDA=1;	  	  
	ICM_IIC_SCL=1;
	ICM_IIC_Delay();
 	ICM_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	ICM_IIC_Delay();
	ICM_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}

/**************************************************/
/*函数名：ICM_42688_STOP;***********************/
/*功能：停止IIC;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void ICM_42688_STOP(){
	ICM_SDA_OUT();//sda线输出
	ICM_IIC_SCL=0;
	ICM_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	ICM_IIC_Delay();
	ICM_IIC_SCL=1; 
	ICM_IIC_SDA=1;//发送I2C总线结束信号
	ICM_IIC_Delay();							   	
}

/**************************************************/
/*函数名：ICM_IIC_Wait_Ack;************************/
/*功能：等待应答信号到来;**************************/
/*输入：无;****************************************/
/*输出：1，接收应答失败 0，接收应答成功;***********/
/**************************************************/
unsigned char ICM_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	ICM_SDA_IN();      //SDA设置为输入  
	ICM_IIC_SDA=1;ICM_IIC_Delay();	   
	ICM_IIC_SCL=1;ICM_IIC_Delay();	 
	while(ICM_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			ICM_42688_STOP();
			return 1;
		}
	}
	ICM_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 

/**************************************************/
/*函数名：ICM_IIC_Ack;************************/
/*功能：产生ACK应答;**************************/
/*输入：无;****************************************/
/*输出：无;***********/
/**************************************************/
void ICM_IIC_Ack(void)
{
	ICM_IIC_SCL=0;
	ICM_SDA_OUT();
	ICM_IIC_SDA=0;
	ICM_IIC_Delay();
	ICM_IIC_SCL=1;
	ICM_IIC_Delay();
	ICM_IIC_SCL=0;
}

/**************************************************/
/*函数名：ICM_IIC_NAck;************************/
/*功能：不产生ACK应答		;**************************/
/*输入：无;****************************************/
/*输出：无;***********/
/**************************************************/  
void ICM_IIC_NAck(void)
{
	ICM_IIC_SCL=0;
	ICM_SDA_OUT();
	ICM_IIC_SDA=1;
	ICM_IIC_Delay();
	ICM_IIC_SCL=1;
	ICM_IIC_Delay();
	ICM_IIC_SCL=0;
}			

/******************************************************************/
/*函数名：ICM_IIC_Send_Byte;***************************************/
/*功能：IIC发送一个字节,返回从机有无应答;**************************/
/*输入：无;********************************************************/
/*输出：无;***************************************/
/******************************************************************/  		  
void ICM_IIC_Send_Byte(unsigned char txd)
{                        
    u8 t;   
		ICM_SDA_OUT(); 	    
    ICM_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        ICM_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		    ICM_IIC_SCL=1;
		    ICM_IIC_Delay(); 
		    ICM_IIC_SCL=0;	
		    ICM_IIC_Delay();
    }	 
} 	

/******************************************************************/
/*函数名：ICM_IIC_Read_Byte;***************************************/
/*功能：读1个字节，ack=1时，发送ACK，ack=0，发送nACK ;*************/
/*输入：ack:是否发送应答;********************************************************/
/*输出：data;***************************************/
/******************************************************************/

unsigned char ICM_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	ICM_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        ICM_IIC_SCL=0; 
        ICM_IIC_Delay();
				ICM_IIC_SCL=1;
        receive<<=1;
        if(ICM_READ_SDA)receive++;   
					ICM_IIC_Delay(); 
				}					 
    if (!ack)
        ICM_IIC_NAck();//发送nACK
    else
        ICM_IIC_Ack(); //发送ACK   
    return receive;
}

/******************************************************************/
/*函数名：ICM_IIC_WRITE_BYTE;***************************************/
/*功能：写一个字节;*************/
/*输入：RA：寄存器地址 data_byte:数据;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
unsigned char ICM_IIC_WRITE_BYTE(unsigned char RA, unsigned char data_byte){
	
	ICM_42688_START();
	ICM_IIC_Send_Byte(ICM_42688_Addr_AD0_LOW_WRITE);
	if(ICM_IIC_Wait_Ack()){
		return 1;
	}
	ICM_IIC_Send_Byte(RA);
	if(ICM_IIC_Wait_Ack()){return 1;}
	ICM_IIC_Send_Byte(data_byte);
	if(ICM_IIC_Wait_Ack()){return 1;}
	ICM_42688_STOP();
	return 0;
}

/******************************************************************/
/*函数名：ICM_IIC_READ_BYTE;***************************************/
/*功能：读一个字节;*************/
/*输入：RA：寄存器地址;********************************************************/
/*输出：0 成功 1 失败;***************************************/
/******************************************************************/
unsigned char ICM_IIC_READ_BYTE(unsigned char RA, unsigned char *data){
	ICM_42688_START();
	ICM_IIC_Send_Byte(ICM_42688_Addr_AD0_LOW_WRITE);
	if(ICM_IIC_Wait_Ack()){return 1;}
	ICM_IIC_Send_Byte(RA);
	if(ICM_IIC_Wait_Ack()){return 1;}
	
	ICM_42688_START();
	ICM_IIC_Send_Byte(ICM_42688_Addr_AD0_LOW_READ);
	if(ICM_IIC_Wait_Ack()){
		return 1;
	}
	*data = ICM_IIC_Read_Byte(0);
	ICM_42688_STOP();
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
