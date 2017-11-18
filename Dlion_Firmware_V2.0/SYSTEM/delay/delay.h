#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"


/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：delay延时代码   版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/


extern volatile unsigned long  timer4_millis;
#define millis() timer4_millis 	 
void delay_init(void);
void TIM4_Int_Init(u16 arr,u16 psc)	;
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























