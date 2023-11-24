#ifndef __ICM_42688_P_H
#define __ICM_42688_P_H

#include "stm32f10x.h"                  // Device header

//IO方向设置
#define ICM_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define ICM_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO操作函数
#define ICM_IIC_SCL    PBout(6) 		//SCL
#define ICM_IIC_SDA    PBout(7) 		//SDA	 
#define ICM_READ_SDA   PBin(7) 		//输入SDA 

/**USER BANK 0 REGISTER MAP – DESCRIPTIONS**/
#define DEVICE_CONFIG 0X11		//Serial IF: R/W	 Reset value: 0x00
#define DRIVE_CONFIG  0X13		//Serial IF: R/W   Reset value: 0x05
#define INT_CONFIG    0X14		//Serial IF: R/W	 Reset value: 0x00
#define FIFO_CONFIG 	0X16		//Serial IF: R/W	 Reset value: 0x00
#define TEMP_DATA1	  0X1D		//Serial IF: SYNCR Reset value: 0x80
#define TEMP_DATA0 		0X1E		//Serial IF: SYNCR Reset value: 0x00

#define ACCEL_DATA_X1 0X1F		//Serial IF: SYNCR Reset value: 0x80
#define ACCEL_DATA_X0 0X20		//Serial IF: SYNCR Reset value: 0x00
#define ACCEL_DATA_Y1 0X21		//Serial IF: SYNCR Reset value: 0x80
#define ACCEL_DATA_Y0 0X22		//Serial IF: SYNCR Reset value: 0x00
#define ACCEL_DATA_Z1 0X23		//Serial IF: SYNCR Reset value: 0x80
#define ACCEL_DATA_Z0 0X24		//Serial IF: SYNCR Reset value: 0x00

#define GYRO_DATA_X1  0X25		//Serial IF: SYNCR Reset value: 0x80
#define GYRO_DATA_X0  0X26		//Serial IF: SYNCR Reset value: 0x00
#define GYRO_DATA_Y1  0X27		//Serial IF: SYNCR Reset value: 0x80
#define GYRO_DATA_Y0  0X28		//Serial IF: SYNCR Reset value: 0x00
#define GYRO_DATA_Z1  0X29		//Serial IF: SYNCR Reset value: 0x80
#define GYRO_DATA_Z0  0X2A		//Serial IF: SYNCR Reset value: 0x00

#define TMST_FSYNCH   0X2B		//Serial IF: SYNCR Reset value: 0x00
#define TMST_FSYNCL   0X2C    //Serial IF: SYNCR Reset value: 0x00
#define INT_STATUS    0X2D    //Serial IF: R/C   Reset value: 0x10
#define FIFO_COUNTH   0X2E    //Serial IF: R     Reset value: 0x00
#define FIFO_COUNTL   0X2F    //Serial IF: R     Reset value: 0x00
#define FIFO_DATA     0X30    //Serial IF: R     Reset value: 0xFF
#define APEX_DATA0    0X31    //Serial IF: SYNCR Reset value: 0x00
#define APEX_DATA1    0X32    //Serial IF: SYNCR Reset value: 0x00
#define APEX_DATA2    0X33    //Serial IF: R     Reset value: 0x00
#define APEX_DATA3    0X34    //Serial IF: R     Reset value: 0x04
#define APEX_DATA4    0X35    //Serial IF: R     Reset value: 0x00
#define APEX_DATA5    0X36    //Serial IF: R     Reset value: 0x00
#define INT_STATUS2   0X37    //Serial IF: R/C   Reset value: 0x00
#define INT_STATUS3   0X38    //Serial IF: R/C   Reset value: 0x00

#define SIGNAL_PATH_RESET   0X4B    //Serial IF: W/C   Reset value: 0x00

#define INTF_CONFIG0  0X4C    //Serial IF: R/W   Reset value: 0x30
#define INTF_CONFIG1  0X4D    //Serial IF: R/W   Reset value: 0x91
#define PWR_MGMT0     0X4E    //Serial IF: R/W   Reset value: 0x00
#define GYRO_CONFIG0  0X4F    //Serial IF: R/W   Reset value: 0x06
#define ACCEL_CONFIG0 0X50    //Serial IF: R/W   Reset value: 0x06
#define GYRO_CONFIG1  0X51    //Serial IF: R/W   Reset value: 0x16

#define GYRO_ACCEL_CONFIG0   0X52    //Serial IF: R/W   Reset value: 0x11

#define ACCEL_CONFIG1 0X53    //Serial IF: R/W   Reset value: 0x0D
#define TMST_CONFIG   0X54    //Serial IF: R/W   Reset value: 0x23
#define APEX_CONFIG0  0X56    //Serial IF: R/W   Reset value: 0x82
#define SMD_CONFIG    0X57    //Serial IF: R/W   Reset value: 0x00
#define FIFO_CONFIG1  0X5F    //Serial IF: R/W   Reset value: 0x00
#define FIFO_CONFIG2  0X60    //Serial IF: R/W   Reset value: 0x00
#define FIFO_CONFIG3  0X61    //Serial IF: R/W   Reset value: 0x00
#define FSYNC_CONFIG  0X62    //Serial IF: R/W   Reset value: 0x10
#define INT_CONFIG0   0X63    //Serial IF: R/W   Reset value: 0x00
#define INT_CONFIG1   0X64    //Serial IF: R/W   Reset value: 0x10
#define INT_SOURCE0   0X65    //Serial IF: R/W   Reset value: 0x10
#define INT_SOURCE1   0X66    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE3   0X68    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE4   0X69    //Serial IF: R/W   Reset value: 0x00

#define FIFO_LOST_PKT0   0X6C    //Serial IF: R   Reset value: 0x00

#define FIFO_LOST_PKT1   0X6D    //Serial IF: R   Reset value: 0x00
#define SELF_TEST_CONFIG   0X70    //Serial IF: R/W   Reset value: 0x00
#define WHO_AM_I   0X75   //Serial IF: R   Reset value: 0x47
#define REG_BANK_SEL   0X76    //Serial IF: R/W   Reset value: 0x00


/**USER BANK 1 REGISTER MAP – DESCRIPTIONS**/
#define SENSOR_CONFIG0   0X03    //Serial IF: R/W   Reset value: 0x80
#define GYRO_CONFIG_STATIC2   0X0B    //Serial IF: R/W   Reset value: 0xA0
#define GYRO_CONFIG_STATIC3   0X0C    //Serial IF: R/W   Reset value: 0x0D
#define GYRO_CONFIG_STATIC4   0X0D    //Serial IF: R/W   Reset value: 0xAA
#define GYRO_CONFIG_STATIC5   0X0E    //Serial IF: R/W   Reset value: 0x80
#define GYRO_CONFIG_STATIC6   0X0F    //Serial IF: R/W   Reset value: 0xXX (Factory trimmed on an individual device basis)
#define GYRO_CONFIG_STATIC7   0X10    //Serial IF: R/W   Reset value: 0xXX (Factory trimmed on an individual device basis)
#define GYRO_CONFIG_STATIC8   0X11    //Serial IF: R/W   Reset value: 0xXX (Factory trimmed on an individual device basis)
#define GYRO_CONFIG_STATIC9   0X12    //Serial IF: R/W   Reset value: 0xXX (Factory trimmed on an individual device basis)
#define GYRO_CONFIG_STATIC10  0X13    //Serial IF: R/W   Reset value: 0x11
#define XG_ST_DATA   0X5F    //Serial IF: R/W   Reset value: 0xXX (The value in this register indicates the self-test output generated during manufacturing tests)
#define YG_ST_DATA   0X60    //Serial IF: R/W   Reset value: 0xXX (The value in this register indicates the self-test output generated during manufacturing tests)
#define ZG_ST_DATA   0X61    //Serial IF: R/W   Reset value: 0xXX (The value in this register indicates the self-test output generated during manufacturing tests)
#define TMSTVAL0   0X62    //Serial IF: R   Reset value: 0x00
#define TMSTVAL1   0X63    //Serial IF: R   Reset value: 0x00
#define TMSTVAL2   0X64    //Serial IF: R   Reset value: 0x00

#define INTF_CONFIG4   0X7A    //Serial IF: R/W   Reset value: 0x83
#define INTF_CONFIG5   0X7B    //Serial IF: R/W   Reset value: 0x00
#define INTF_CONFIG6   0X7C    //Serial IF: R/W   Reset value: 0x5F

/**USER BANK 2 REGISTER MAP – DESCRIPTIONS**/
#define ACCEL_CONFIG_STATIC2   0X03    //Serial IF: R/W   Reset value: 0x30
#define ACCEL_CONFIG_STATIC3   0X04    //Serial IF: R/W   Reset value: 0x40
#define ACCEL_CONFIG_STATIC4   0X05    //Serial IF: R/W   Reset value: 0x62
#define XA_ST_DATA   0X3B    //Serial IF: R/W   Reset value: 0xXX (The value in this register indicates the self-test output generated during manufacturing tests)
#define YA_ST_DATA   0X3C    //Serial IF: R/W   Reset value: 0xXX (The value in this register indicates the self-test output generated during manufacturing tests)
#define ZA_ST_DATA   0X3D    //Serial IF: R/W   Reset value: 0xXX (The value in this register indicates the self-test output generated during manufacturing tests)

/**USER BANK 4 REGISTER MAP – DESCRIPTIONS**/
#define APEX_CONFIG1   0X40    //Serial IF: R/W   Reset value: 0xA2
#define APEX_CONFIG2   0X41    //Serial IF: R/W   Reset value: 0x85
#define APEX_CONFIG3   0X42    //Serial IF: R/W   Reset value: 0x51
#define APEX_CONFIG4   0X43    //Serial IF: R/W   Reset value: 0xA4
#define APEX_CONFIG5   0X44    //Serial IF: R/W   Reset value: 0x8C
#define APEX_CONFIG6   0X45    //Serial IF: R/W   Reset value: 0x5C
#define APEX_CONFIG7   0X46    //Serial IF: R/W   Reset value: 0x45
#define APEX_CONFIG8   0X47    //Serial IF: R/W   Reset value: 0x5B
#define APEX_CONFIG9   0X48    //Serial IF: R/W   Reset value: 0x00
#define ACCEL_WOM_X_THR   0X4A    //Serial IF: R/W   Reset value: 0x00
#define ACCEL_WOM_Y_THR   0X4B    //Serial IF: R/W   Reset value: 0x00
#define ACCEL_WOM_Z_THR   0X4C    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE6   0X4D    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE7   0X4E    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE8   0X4F    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE9   0X50    //Serial IF: R/W   Reset value: 0x00
#define INT_SOURCE10  0X51    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER0  0X77    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER1  0X78    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER2  0X79    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER3  0X7A    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER4  0X7B    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER5  0X7C    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER6  0X7D    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER7  0X7E    //Serial IF: R/W   Reset value: 0x00
#define OFFSET_USER8  0X7F    //Serial IF: R/W   Reset value: 0x00

/**CONTROL DESCRIPTIONS**/
#define DEVICE_RESET 0X01 //DEVICE_CONFIG


typedef struct{
    double X;
    double Y;
    double Z;
}GYRO;
typedef struct{
    double X;
    double Y;
    double Z;
}ACC;
typedef struct{
    double T;
}TEMP;
typedef struct{
	double pitch;
	double row;
	double yaw;
}ATTU;

void ICM_Port_Init(void);

unsigned char ICM_IIC_Read_Byte(unsigned char ack);
unsigned char ICM_IIC_WRITE_BYTE(unsigned char RA, unsigned char data_byte);

unsigned char ICM_INIT(void);
unsigned char ICM_Gyroscope_INIT(void);
unsigned char ICM_ACC_INIT(void);
unsigned char GYRO_ACC_TEMP_GET(void);
double myabs(double f);
void IIC_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
void Kalman_Filter_X(void);
void Kalman_Filter_Y(void);
#endif 
