#ifndef __SPI_H
#define __SPI_H
#include "sys.h"


/*******************************************************������Դ��3D��ӡѧϰʹ��************************************************
																												Dlion-3D��ӡ����
																												3D�����ƴ���---------������̳:www.3dbinmaker.com
																												�ļ�˵����SPI��������   �汾��V1.0
																												Copyright(C)�������ڿƼ����޹�˾
																												All rights reserved
***********************************************************************************************************************************/



 
 				  	    													  
void SPI2_Init(void);			 //��ʼ��SPI2��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI2�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI2���߶�дһ���ֽ�

void SPI1_Init(void);			 //��ʼ��SPI1��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1���߶�дһ���ֽ�
		 
#endif
