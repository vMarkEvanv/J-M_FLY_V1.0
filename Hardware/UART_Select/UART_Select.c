#include "stm32f10x.h"                  // Device header
#include "UART_Select.h"
#include "sys.h"
#include "stm32f10x.h"                  // Device header

//串口接收缓存区 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.
u8  USART3_TX_BUF[USART3_MAX_SEND_LEN]; 			//发送缓冲,最大USART3_MAX_SEND_LEN字节
 
//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
vu16 USART3_RX_STA=0; 

void SELECT_Port_INIT(){

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
  GPIO_ResetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);		
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
}

/**************************************************/
/*函数名：UART_Init;***********************/
/*功能：初始化选择串口;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void UART_Init(u32 bound){
 
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	// GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //串口3时钟使能
 
 	USART_DeInit(USART3);  //复位串口3
		 //USART3_TX   PB10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化PB10
   
    //USART3_RX	  PB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化PB11
	
	USART_InitStructure.USART_BaudRate = bound;//波特率一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  
	USART_Init(USART3, &USART_InitStructure); //初始化串口	3
  
 
	USART_Cmd(USART3, ENABLE);                    //使能串口 
	
	//使能接收中断
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断   
	
	//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART3_RX_STA=0;		//清零
 }

/**************************************************/
/*函数名：USART3_IRQHandler;***********************/
/*功能：串口3中断服务程序;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/

void USART3_IRQHandler(void)
{
	u8 Res;	      
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
	{	 
		Res =USART_ReceiveData(USART3);	 
		if((USART3_RX_STA&0x8000)==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
		    if((USART3_RX_STA&0X7FFF)<USART3_MAX_RECV_LEN)	//还可以接收数据
				{
					if(Res!='!')
					{
						USART3_RX_BUF[USART3_RX_STA++]=Res;	//记录接收到的值
						//printf("%c\r\n",Res);
					}	
          else 
					{
						USART3_RX_STA|=0x8000;	//则信息接收完成了 
					} 					
				}
				else 
				{
					USART3_RX_STA|=0x8000;	//则信息接收完成了 
				} 
		}
		USART3_RX_Data();
		//PCout(13)=~PCout(13);
		//USART_SendData(USART1,Res);
	}  				 											 
}  

/**************************************************/
/*函数名：USART3_RX_Data;***********************/
/*功能：串口3服务;************/
/*输入：无;****************************************/
/*输出：无;****************************************/
/**************************************************/
void USART3_RX_Data()
{
	u8 len=0;
	if(USART3_RX_STA&0x8000)
		{					   
			len=USART3_RX_STA&0X7FFF;//得到此次接收到的数据长度
			USART3_RX_BUF[len]=0;	 	//加入结束符
			
			if(len>USART3_MAX_RECV_LEN-2)
			{
				len=USART3_MAX_RECV_LEN-1;
				USART3_RX_BUF[len]=0;	 	//加入结束符
			}
			
			USART3_RX_BUF[USART3_MAX_RECV_LEN-1]=0x01;
//			u3_printf("%s\r\n",USART3_RX_BUF);
			USART3_RX_STA=0;
		}
 
}