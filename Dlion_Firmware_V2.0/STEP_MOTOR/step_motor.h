#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H
#include "sys.h"

#define MOTOR_ENABLE 0
#define MOTOR_DISABLE 1

#define SUBSECTION_H 0
#define SUBSECTION_L 1

void STEP_Motor_Init(void);

void dis_m_x(void);
void en_m_x(void);

void dis_m_y(void);
void en_m_y(void);

void dis_m_z(void);
void en_m_z(void);

void dis_m_e1(void);
void en_m_e1(void);

void dis_m_e2(void);
void en_m_e2(void);

void subsection_x(u8 m);
void subsection_y(u8 m);
void subsection_z(u8 m);
void subsection_e1(u8 m);
void subsection_e2(u8 m);

void step_x(void);
void step_y(void);
void step_z(void);

void change_dir_x(void);	
void change_dir_y(void);	
void change_dir_z(void);	



#endif
