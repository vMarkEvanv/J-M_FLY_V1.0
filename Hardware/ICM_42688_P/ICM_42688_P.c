#include "ICM_42688_P.h"
#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "sys.h"
#include "FLY_Control_Logic.h"
#include "math.h"
#define pi 3.1415926

#define RAD2DEG 57.295779513

ATTU attu;

unsigned char ICM_42688_Addr_AD0_LOW_READ = 0xD1;   //AD0低电平地址的读
unsigned char ICM_42688_Addr_AD0_HIGH_READ = 0xD3;	 //AD0高电平地址的读
unsigned char ICM_42688_Addr_AD0_LOW_WRITE = 0xD0;	 //AD0低电平地址的写
unsigned char ICM_42688_Addr_AD0_HIGH_WRITE = 0xD2; //AD0高电平地址的写

GYRO Gyro_Get;
GYRO Gyro_Last;
ACC Acc_Get;
TEMP Temp;
ACC Acc_Last;
extern FIX_VALUE FIXED_VALUE;

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
	I2C_InitStructure.I2C_ClockSpeed = 1000000;
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
	return 0;
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
	if(ICM_IIC_WRITE_BYTE(DRIVE_CONFIG,0x05)) return 2;//Control the communication speed(I guess)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_CONFIG,0x02)) return 3;//interrupt settings
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(PWR_MGMT0,0x0F)) return 4;//power register of sensors(it won't working if we don't turn it on)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_CONFIG1,0x00)) return 5;//this register is to set the interrupt port's Interrupt pulse duration (more details on datasheet)
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(INT_SOURCE0,0x08)) return 6;//setting interrupt port's interrupt source
	delay_ms(50);
	if(ICM_Gyroscope_INIT()) return 7;//陀螺仪初始化
	delay_ms(50);
	if(ICM_ACC_INIT()) return 8;//加速度计初始化
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(SELF_TEST_CONFIG,0x1F)) return 9;//自检
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
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG1,0x1A)) return 1;//调整带宽和滤波次数
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
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG0,0x66)) return 1;//调整采样率和ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG1,0x0D)) return 1;//调整带宽和滤波次数
	delay_ms(50);
	return 0;
	
}

double myabs(double f){
	if(f<0){
		return -f;
	}
	return f;
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
	
	Acc_Last.X = Acc_Get.X;
	Acc_Last.Y = Acc_Get.Y;
	Acc_Last.Z = Acc_Get.Z;
	
	//温度读取
	if(ICM_IIC_READ_BYTE(TEMP_DATA1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(TEMP_DATA0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Temp.T = (Counting_Temp / 132.48) + 25;
	
	//X轴陀螺仪读取 ±2000dps
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.X = (Counting_Temp*1.0)/32767.0*2000;
	
	//Y轴陀螺仪读取 ±2000dps
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Y = (Counting_Temp*1.0)/32767.0*2000;
	
	//Z轴陀螺仪读取 ±2000dps
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
	Acc_Get.X = (Counting_Temp*1.0)/32767.0*2.0*9.8;
	
	//Y轴加速度计读取 ±16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Y = (Counting_Temp*1.0)/32767.0*2.0*9.8;
	
	//Z轴加速度计读取 ±16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Z = (Counting_Temp*1.0)/32767.0*2.0*9.8;
	
	//陀螺仪修正
//	Gyro_Get.X -= FIXED_VALUE.X;
//	Gyro_Get.Y -= FIXED_VALUE.Y;
//	Gyro_Get.Z -= FIXED_VALUE.Z;
	Gyro_Get.X -= 101.38;
	Gyro_Get.Y -= 110.84;
	Gyro_Get.Z -= 118.94;
	//printf("%.2f,%.2f,%.2f\r\n",Gyro_Get.X,Gyro_Get.Y,Gyro_Get.Z);
	//滤波
	if(myabs(Gyro_Get.X) <= 0.3){Gyro_Get.X = 0;}
	if(myabs(Gyro_Get.Y) <= 0.3){Gyro_Get.Y = 0;}
	if(myabs(Gyro_Get.Z) <= 0.3){Gyro_Get.Z = 0;}
	
	
	//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",myabs(Gyro_Get.X - Gyro_Last.X),myabs(Gyro_Get.Y - Gyro_Last.Y),myabs(Gyro_Get.Z - Gyro_Last.Z),Gyro_Get.X,Gyro_Get.Y,Gyro_Get.Z);

	return 0;
}







/******************************************************************/
/*函数名：Kalman_Filter_X;***************************************/
/*功能：pitch的卡尔曼滤波;*************/
/*输入：无;********************************************************/
/*输出：无;***************************************/
/******************************************************************/
void Kalman_Filter_X()
{
	static float Accel;
	static float Gyro;
	
	static float angle_dot;
	static float Q_angle=0.001;// 过程噪声的协方差
	static float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	static float R_angle=0.5;// 测量噪声的协方差 既测量偏差
	static float dt=0.05;//                 
	static char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Accel=atan2(Acc_Get.Y,Acc_Get.Z)*RAD2DEG;
	Gyro=Gyro_Get.X/16.4;
	

	attu.pitch+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - attu.pitch;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	attu.pitch	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

/******************************************************************/
/*函数名：Kalman_Filter_Y;***************************************/
/*功能：row的卡尔曼滤波;*************/
/*输入：无;********************************************************/
/*输出：无;***************************************/
/******************************************************************/
void Kalman_Filter_Y()
{
	static float Accel;
	static float Gyro;
	
	static float angle_dot;
	static float Q_angle=0.001;// 过程噪声的协方差
	static float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	static float R_angle=0.5;// 测量噪声的协方差 既测量偏差
	static float dt=0.05;//                 
	static char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	Accel=-atan2(Acc_Get.X,Acc_Get.Z)*RAD2DEG;
	Gyro=Gyro_Get.Y/16.4;
	

	attu.row+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - attu.row;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	attu.row	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}
