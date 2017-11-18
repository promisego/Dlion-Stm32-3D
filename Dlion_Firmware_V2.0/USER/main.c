#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "beep.h" 
#include "adc.h"
#include "24cxx.h"
#include "flash.h" 
#include "spi.h"
#include "Dlion.h"      



/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：主函数   版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/



int main(void)
 {	
	SystemInit();			//系统初始化	
	delay_init();	    	//延时函数初始化
	NVIC_Configuration(); 	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	TIM4_Int_Init(9,7199);//72M  7199分频，1s需要10000（72000000/7199=10000Hz）个时钟频率，1ms需要10个，此处填9代表1ms
	uart1_init(115200);	 	//串口初始化为115200
	BEEP_Init();         	//初始化蜂鸣器端口
	BEEP=1;  
	delay_ms(10);
	BEEP=0;	
	SPI2_Init();		   	//初始化SD FLASH SPI接口 
	setup(); 
	loop();
 }
