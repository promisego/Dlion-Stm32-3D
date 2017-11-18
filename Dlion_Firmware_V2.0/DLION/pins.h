#ifndef PINS_H
#define PINS_H

//All these generations of Gen7 supply thermistor power
//via PS_ON, so ignore bad thermistor readings
//#define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE

#define X_DIR_PIN     PDout(11) 
#define X_STEP_PIN    PDout(12)     
#define X_ENABLE_PIN  PDout(13) 
#define X_MIN_PIN     PDin(6) 
//#define X_MAX_PIN     PGin(15)    
//#define X_MS1_PIN 	   PAout(14)
//#define X_MS2_PIN 	   PAout(13)
//#define X_MS3_PIN 	   PAout(9)

#define Y_DIR_PIN      PGout(2)
#define Y_STEP_PIN     PGout(3) 
#define Y_ENABLE_PIN   PGout(4)
#define Y_MIN_PIN      PGin(9) 
//#define Y_MAX_PIN      PGin(14)  
//#define Y_MS1_PIN      PGout(8) 
//#define Y_MS2_PIN      PGout(7) 
//#define Y_MS3_PIN      PGout(6) 

#define Z_DIR_PIN      PGout(5) 
#define Z_STEP_PIN     PGout(6) 
#define Z_ENABLE_PIN   PGout(7)
#define Z_MIN_PIN      PGin(10) 
//#define Z_MAX_PIN      PGin(13) 
//#define Z_MS1_PIN      PCout(4)
//#define Z_MS2_PIN      PCout(3)
//#define Z_MS3_PIN      PCout(2)

#define E0_DIR_PIN     PGout(8) 
#define E0_STEP_PIN    PCout(6)         
#define E0_ENABLE_PIN  PCout(7)     
//#define E0_MS1_PIN     PFout(9)
//#define E0_MS2_PIN 	   PFout(8)
//#define E0_MS3_PIN 	   PFout(7)

#define E1_DIR_PIN	   PAout(13) 
#define E1_STEP_PIN    PAout(14)	
#define E1_ENABLE_PIN   PAout(15) 
//#define E1_MS1_PIN      PEout(5) 
//#define E1_MS2_PIN      PEout(4) 
//#define E1_MS3_PIN      PEout(3)

#define  HEATER_0_PIN   TIM8->CCR2	  //E0_PWM T8_2 
//#define HEATER_1_PIN   TIM5->CCR3	  //E1_PWM T5_3
#define  FAN_PIN        TIM5->CCR1	  //BED_FAN T5_1
#define  HEATER_BED_PIN TIM8->CCR3	  //BED_PWM T8_3

#define  E0_FAN       PAout(1) // TIM5->CCR2  	//E0_FAN T5_2 
 
#define TEMP_0_PIN	   (Get_Adc(ADC_Channel_4)>>2)   						// AD3_4	E0_TEMP
//#define TEMP_1_PIN	 (Get_Adc(ADC_Channel_6)>>2)   						// AD3_6	E1_TEMP
#define TEMP_BED_PIN   (Get_Adc(ADC_Channel_5)>>2)   						// AD3_5  BED_TEMP

#endif
