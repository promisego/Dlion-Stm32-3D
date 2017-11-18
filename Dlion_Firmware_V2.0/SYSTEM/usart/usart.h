#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 


/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：串口1驱动代码  版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/


//#define USART_REC_LEN  			256  	//定义最大接收字节数 256
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define RX_BUFFER_SIZE 128
typedef struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
}ring_buffer;
extern  ring_buffer rx_buffer;	
//extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart1_init(u32 bound);
void checkRx(void);
unsigned int MYSERIAL_available(void);
u8 MYSERIAL_read(void);
void MYSERIAL_flush(void);
#endif


