#include "ICM_42688_P.h"
#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "sys.h"

unsigned char ICM_42688_Addr_AD0_LOW_READ = 0xD1;   //AD0�͵�ƽ��ַ�Ķ�
unsigned char ICM_42688_Addr_AD0_HIGH_READ = 0xD3;	 //AD0�ߵ�ƽ��ַ�Ķ�
unsigned char ICM_42688_Addr_AD0_LOW_WRITE = 0xD0;	 //AD0�͵�ƽ��ַ��д
unsigned char ICM_42688_Addr_AD0_HIGH_WRITE = 0xD2; //AD0�ߵ�ƽ��ַ��д

GYRO Gyro_Get;
ACC Acc_Get;
TEMP Temp;

void ICM_IIC_Delay(void){
	delay_us(2);
}
/**************************************************/
/*��������Port_Init;***********************/
/*���ܣ���ʼ��Ӳ��IICͨ��;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void ICM_Port_Init(){
	//delay_init();
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTBʱ�� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
  GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);						 //PB10,PB11 �����	

}	  

/************************************************************************/
/*��������ICM_42688_START;***********************************************/
/*���ܣ�ͨ��IICЭ������ICM_42688_Pģ��;**********************************/
/*���룺��;**************************************************************/
/*�����0����ʼ�źŷ��ͳɹ������ҵõ�Ӧ�� 1��δ�õ�Ӧ��;*****************/
/************************************************************************/
void ICM_42688_START(){
	ICM_SDA_OUT();
	ICM_IIC_SDA=1;	  	  
	ICM_IIC_SCL=1;
	ICM_IIC_Delay();
 	ICM_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	ICM_IIC_Delay();
	ICM_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************************************/
/*��������ICM_42688_STOP;***********************/
/*���ܣ�ֹͣIIC;************/
/*���룺��;****************************************/
/*�������;****************************************/
/**************************************************/
void ICM_42688_STOP(){
	ICM_SDA_OUT();//sda�����
	ICM_IIC_SCL=0;
	ICM_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	ICM_IIC_Delay();
	ICM_IIC_SCL=1; 
	ICM_IIC_SDA=1;//����I2C���߽����ź�
	ICM_IIC_Delay();							   	
}

/**************************************************/
/*��������ICM_IIC_Wait_Ack;************************/
/*���ܣ��ȴ�Ӧ���źŵ���;**************************/
/*���룺��;****************************************/
/*�����1������Ӧ��ʧ�� 0������Ӧ��ɹ�;***********/
/**************************************************/
unsigned char ICM_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	ICM_SDA_IN();      //SDA����Ϊ����  
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
	ICM_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 

/**************************************************/
/*��������ICM_IIC_Ack;************************/
/*���ܣ�����ACKӦ��;**************************/
/*���룺��;****************************************/
/*�������;***********/
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
/*��������ICM_IIC_NAck;************************/
/*���ܣ�������ACKӦ��		;**************************/
/*���룺��;****************************************/
/*�������;***********/
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
/*��������ICM_IIC_Send_Byte;***************************************/
/*���ܣ�IIC����һ���ֽ�,���شӻ�����Ӧ��;**************************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/  		  
void ICM_IIC_Send_Byte(unsigned char txd)
{                        
    u8 t;   
		ICM_SDA_OUT(); 	    
    ICM_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
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
/*��������ICM_IIC_Read_Byte;***************************************/
/*���ܣ���1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK ;*************/
/*���룺ack:�Ƿ���Ӧ��;********************************************************/
/*�����data;***************************************/
/******************************************************************/

unsigned char ICM_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	ICM_SDA_IN();//SDA����Ϊ����
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
        ICM_IIC_NAck();//����nACK
    else
        ICM_IIC_Ack(); //����ACK   
    return receive;
}

/******************************************************************/
/*��������ICM_IIC_WRITE_BYTE;***************************************/
/*���ܣ�дһ���ֽ�;*************/
/*���룺RA���Ĵ�����ַ data_byte:����;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
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
/*��������ICM_IIC_READ_BYTE;***************************************/
/*���ܣ���һ���ֽ�;*************/
/*���룺RA���Ĵ�����ַ;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
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
/*��������ICM_INIT;***************************************/
/*���ܣ�ICMоƬ��ʼ��;*************/
/*���룺��;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
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
	if(ICM_Gyroscope_INIT()) return 1;//�����ǳ�ʼ��
	delay_ms(50);
	if(ICM_ACC_INIT()) return 1;//���ٶȼƳ�ʼ��
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(SELF_TEST_CONFIG,0x1F)) return 1;//�Լ�
	delay_ms(50);
	return 0;
}

/******************************************************************/
/*��������ICM_Gyroscope_INIT;***************************************/
/*���ܣ�ICM�����ǳ�ʼ��;*************/
/*���룺��;********************************************************/
/*�����0 �ɹ� 1 ʧ��;***************************************/
/******************************************************************/
unsigned char ICM_Gyroscope_INIT(){
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG0,0x06)) return 1;//���������ʺ�ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(GYRO_CONFIG1,0x16)) return 1;//����������˲�����
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(GYRO_ACCEL_CONFIG0,0x11)) return 1;//���������Ǻͼ��ٶȼƵĵ�ͨ�˲�������
	delay_ms(50);
	return 0;
}

/******************************************************************/
/*��������ICM_ACC_INIT;***************************************/
/*���ܣ�ICM���ٶȼƳ�ʼ��;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
unsigned char ICM_ACC_INIT(){
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG0,0x06)) return 1;//���������ʺ�ODR
	delay_ms(50);
	if(ICM_IIC_WRITE_BYTE(ACCEL_CONFIG1,0x0D)) return 1;//����������˲�����
	delay_ms(50);
	return 0;
	
}

/******************************************************************/
/*��������GYRO_ACC_TEMP_GET;***************************************/
/*���ܣ���ȡ�����ǣ����ٶȼƣ��¶�����;*************/
/*���룺��;********************************************************/
/*�������;***************************************/
/******************************************************************/
unsigned char GYRO_ACC_TEMP_GET(){
	unsigned char temp = 0;
	short Counting_Temp = 0;
	//�¶ȶ�ȡ
	if(ICM_IIC_READ_BYTE(TEMP_DATA1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(TEMP_DATA0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Temp.T = (Counting_Temp / 132.48) + 25;
	
	//X�������Ƕ�ȡ
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.X = (Counting_Temp*1.0)/32767.0*2000;
	
	//Y�������Ƕ�ȡ
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Y = (Counting_Temp*1.0)/32767.0*2000;
	
	//Z�������Ƕ�ȡ
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(GYRO_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Gyro_Get.Z = (Counting_Temp*1.0)/32767.0*2000;
	
	//X����ٶȼƶ�ȡ ��16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_X1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_X0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.X = (Counting_Temp*1.0)/32767.0*16.0*9.8;
	
	//Y����ٶȼƶ�ȡ ��16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Y0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Y = (Counting_Temp*1.0)/32767.0*16.0*9.8;
	
	//Z����ٶȼƶ�ȡ ��16g
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z1,&temp))return 1;
	Counting_Temp = temp;
	Counting_Temp = Counting_Temp << 8;
	temp = 0;
	if(ICM_IIC_READ_BYTE(ACCEL_DATA_Z0,&temp))return 1;
	Counting_Temp = Counting_Temp|temp ;temp = 0;
	Acc_Get.Z = (Counting_Temp*1.0)/32767.0*16.0*9.8;
	
	return 0;
}
