#include "step_motor.h" 
#include "delay.h"
u8 dir_x=0;
u8 dir_y=0;
u8 dir_z=0;

void STEP_Motor_Init(void)
{	 GPIO_InitTypeDef  GPIO_InitStructure;

 	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG, ENABLE);	 //使能GPIOA端口时钟
	 

	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
	 GPIO_Init(GPIOA, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;		
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_13;			
	 GPIO_Init(GPIOC, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;	
	 GPIO_Init(GPIOE, &GPIO_InitStructure);	 

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	 GPIO_Init(GPIOF, &GPIO_InitStructure);	 

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;		
	 GPIO_Init(GPIOG, &GPIO_InitStructure);	
	
	 disable_x();	  //关闭x驱动
	 disable_y();	  //关闭y驱动
	 disable_z();	  //关闭z驱动
	 disable_e1();	  //关闭e1驱动
	 disable_e2();	  //关闭e2驱动

	 subsection_x(1);
	 subsection_y(1);
	 subsection_z(1);
	 subsection_e1(1);
	 subsection_e2(1);
	 
	 PCout(10)=1;
	 PCout(7)=1;
	 PBout(10)=1;
	 PCout(0)=1;
	 PCout(13)=1;

	 PEout(6)=dir_x;
	 PCout(6)=dir_y;
	 PFout(11)=dir_z;

}

void step_x(void)
{ int i;
for(i=0;i<200;i++)
{ 	
	PCout(13)=0;
	delay_ms(1);
 	PCout(13)=1;
	delay_ms(1);
}
}

void step_y(void)
{ int i;
for(i=0;i<200;i++)
{ 	
	PCout(7)=0;
	delay_ms(1);
 	PCout(7)=1;
	delay_ms(1);
}
}

void step_z(void)
{ int i;
for(i=0;i<200;i++)
{ 	
	PBout(10)=0;
	delay_ms(1);
 	PBout(10)=1;
	delay_ms(1);
}
}
void change_dir_x(void)	
{ 	dir_x=~dir_x;
	PEout(6)=dir_x;
}
void change_dir_y(void)	
{ 
	dir_y=~dir_y;
	PCout(6)=dir_y;
}
void change_dir_z(void)	
{ 
	dir_z=~dir_z;
	PFout(11)=dir_z;
}
void dis_m_e2(void)
{
	disable_x();
}
void en_m_e2(void)
{
	enable_x();
}

void dis_m_y(void)
{
	disable_y();
}
void en_m_y(void)
{
	enable_y();
}

void dis_m_z(void)
{
	disable_z();
}
void en_m_z(void)
{
	enable_z();
}

void dis_m_e1(void)
{
	disable_e1();
}
void en_m_e1(void)
{
	enable_e1();
}

void dis_m_x(void)
{
	disable_e2();
}
void en_m_x(void)
{
	enable_e2();
}


