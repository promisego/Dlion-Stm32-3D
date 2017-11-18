#include "delay.h"
#include "Dlion.h"
#include "stepper.h"
#include "Configuration_adv.h"
#include "Configuration.h"
#include "speed_lookuptable.h"
#include "usart.h"	
#include "spi.h"
#include "language.h"
#include "temperature.h"


/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：电机执行  版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/



#define DIGIPOT_CHANNELS {4,1,0,2,3} // X Y Z E0 E1 digipot channels to stepper driver mapping

static u8 subsection_x_value=1;
static u8 subsection_y_value=1;
static u8 subsection_z_value=1;
static u8 subsection_e0_value=1;
static u8 subsection_e1_value=1;


#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);  //使能TIMx
#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIM_ITConfig(TIM3,TIM_IT_Update, DISABLE); 


//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef ADVANCE
  static long advance_rate, advance, final_advance = 0;
  static long old_advance = 0;
  static long e_steps[3];
#endif
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short TIME3_nominal;
//static unsigned short step_loops_nominal;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
bool abort_on_endstop_hit = false;
#endif
#if defined X_MIN_PIN
static bool old_x_min_endstop=true;
#endif
#if defined X_MAX_PIN
static bool old_x_max_endstop=true;
#endif
#if defined Y_MIN_PIN
static bool old_y_min_endstop=true;
#endif
#if defined Y_MAX_PIN
static bool old_y_max_endstop=true;
#endif
#if defined Z_MIN_PIN
static bool old_z_min_endstop=true;
#endif
#if defined Z_MAX_PIN
static bool old_z_max_endstop=true;
#endif
static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define CHECK_ENDSTOPS  if(check_endstops)

#define MultiU24X24toH16(intRes, longIn1, longIn2) intRes= ((uint64_t)(longIn1) * (longIn2)) >> 24
#define MultiU16X8toH16(intRes, charIn1, intIn2) intRes = ((charIn1) * (intIn2)) >> 16
void checkHitEndstops(void)
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   printf(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     printf(" X:%f",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
    // LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "X");  ////////////////////////////////////////////////////
   }
   if(endstop_y_hit) {
     printf(" Y:%f",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   //  LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Y");   ////////////////////////////////////////////////////////////
   }
   if(endstop_z_hit) {
     printf(" Z:%f",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   //  LCD_MESSAGEPGM(MSG_ENDSTOPS_HIT "Z");  ////////////////////////////////////////////
   }
   printf("\n");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
   if (abort_on_endstop_hit)
   {
     card.sdprinting = false;
     card.closefile();
     quickStop();
     setTargetHotend0(0);
     setTargetHotend1(0);
     setTargetHotend2(0);
   }
#endif
 }
}

void endstops_hit_on_purpose(void)
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up(void) {
  ENABLE_STEPPER_DRIVER_INTERRUPT();  
}

void step_wait(void)
{
   u8 i;
    for( i=0; i < 6; i++){
    }
}
  

unsigned short calc_timer(unsigned short step_rate) 
{
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;
  
  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  } 
  
  if(step_rate < 32) step_rate = 32;
  timer = 2000000/step_rate - 1;
  if(timer < 100) { timer = 100; printf(MSG_STEPPER_TO_HIGH); printf("%d",step_rate); }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
void trapezoid_generator_reset(void)
{
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
    old_advance = advance >>8;  
  #endif
  deceleration_time = 0;
  // step_rate to timer interval
  TIME3_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  //step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
   TIM_SetAutoreload(TIM3, acceleration_time-1);
 // printf("acc_step_rate: %ld\r\n",acc_step_rate,);/
//    SERIAL_ECHO_START;
//    SERIAL_ECHOPGM("advance :");
//    SERIAL_ECHO(current_block->advance/256.0);
//    SERIAL_ECHOPGM("advance rate :");
//    SERIAL_ECHO(current_block->advance_rate/256.0);
//    SERIAL_ECHOPGM("initial advance :");
//  SERIAL_ECHO(current_block->initial_advance/256.0);
//    SERIAL_ECHOPGM("final advance :");
//    SERIAL_ECHOLN(current_block->final_advance/256.0);
    
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{ 
if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
{TIM_ClearITPendingBit(TIM3, TIM_IT_Update );  //清除TIMx的中断待处理位:TIM 中断源  
//printf("T3\n");  
  // If there is no current block, attempt to pop one from the buffer
 
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      current_block->busy = true;
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0; 
      
      #ifdef Z_LATE_ENABLE 
        if(current_block->steps_z > 0) {
          enable_z();
          TIM_SetAutoreload(TIM3, 2000-1);//1ms wait
          return;
        }
      #endif     
//      #ifdef ADVANCE
//      e_steps[current_block->active_extruder] = 0;
//      #endif
    } 
    else {
        TIM_SetAutoreload(TIM3, 2000-1);
    }    
  } 

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction en check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
      #if !defined COREXY  //NOT COREXY
        X_DIR_PIN=INVERT_X_DIR;
      #endif
      count_direction[X_AXIS]=-1;
      CHECK_ENDSTOPS
      {
				#if defined X_MIN_PIN
          bool x_min_endstop= X_MIN_PIN != X_ENDSTOPS_INVERTING ;
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) 
					{
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_min_endstop = x_min_endstop;
				#endif
      }
    }
    else { // +direction
      #if !defined COREXY  //NOT COREXY
       X_DIR_PIN=!INVERT_X_DIR;
      #endif
      
      count_direction[X_AXIS]=1;
      CHECK_ENDSTOPS 
      {  
				#if defined X_MAX_PIN
          bool x_max_endstop= X_MAX_PIN != X_ENDSTOPS_INVERTING;
          if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
            endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
            endstop_x_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_x_max_endstop = x_max_endstop;
				#endif
      }
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      #if !defined COREXY  //NOT COREXY
        Y_DIR_PIN=INVERT_Y_DIR;
      #endif
      count_direction[Y_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if defined(Y_MIN_PIN) //&& Y_MIN_PIN > -1
          bool y_min_endstop=Y_MIN_PIN != Y_ENDSTOPS_INVERTING;
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_min_endstop = y_min_endstop;
        #endif
      }
    }
    else { // +direction
      #if !defined COREXY  //NOT COREXY
        Y_DIR_PIN=!INVERT_Y_DIR;
      #endif
      count_direction[Y_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if defined(Y_MAX_PIN)// && Y_MAX_PIN > -1
          bool y_max_endstop=Y_MAX_PIN != Y_ENDSTOPS_INVERTING;
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
            endstop_y_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_y_max_endstop = y_max_endstop;
        #endif
      }
    }
    
    
   /* #ifdef COREXY  //coreXY kinematics defined
      if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) == 0)){  //+X is major axis
        X_DIR_PIN= !INVERT_X_DIR;
        Y_DIR_PIN= !INVERT_Y_DIR;
      }
      if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) != 0)){  //-X is major axis
        X_DIR_PIN= INVERT_X_DIR;
        Y_DIR_PIN= INVERT_Y_DIR;
      }      
      if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) == 0)){  //+Y is major axis
        X_DIR_PIN= !INVERT_X_DIR;
        Y_DIR_PIN= INVERT_Y_DIR;
      }        
      if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) != 0)){  //-Y is major axis
        X_DIR_PIN= INVERT_X_DIR;
        Y_DIR_PIN= !INVERT_Y_DIR;
      }  
    #endif //coreXY
  */  
    
    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      Z_DIR_PIN=INVERT_Z_DIR;
      
	  #ifdef Z_DUAL_STEPPER_DRIVERS
       // WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
      #endif
      
      count_direction[Z_AXIS]=-1;
      CHECK_ENDSTOPS
      {
        #if defined(Z_MIN_PIN)
          bool z_min_endstop= Z_MIN_PIN != Z_ENDSTOPS_INVERTING;
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
        #endif
      }
    }
    else { // +direction
      Z_DIR_PIN=!INVERT_Z_DIR;

	  #ifdef Z_DUAL_STEPPER_DRIVERS
       // WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
      #endif

      count_direction[Z_AXIS]=1;
      CHECK_ENDSTOPS
      {
        #if defined(Z_MAX_PIN)
          bool z_max_endstop=Z_MAX_PIN != Z_ENDSTOPS_INVERTING;
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            step_events_completed = current_block->step_event_count;
          }
          old_z_max_endstop = z_max_endstop;
        #endif
      }
    }
	
    #ifndef ADVANCE	 
	
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        REV_E_DIR();
        count_direction[E_AXIS]=-1;
      }
      else { // +direction
        NORM_E_DIR();
        count_direction[E_AXIS]=1;
      }
	 
    #endif //!ADVANCE
    
    
    {u8 i;
    for(i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 

      #ifdef ADVANCE 
	  
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          e_steps[current_block->active_extruder]--;
        }
        else {
          e_steps[current_block->active_extruder]++;
        }
      }   
	  
      #endif //ADVANCE

      #if !defined COREXY 
    
        counter_x += current_block->steps_x;
        if (counter_x > 0) {
          X_STEP_PIN= !INVERT_X_STEP_PIN;
          counter_x -= current_block->step_event_count;
          count_position[X_AXIS]+=count_direction[X_AXIS];   
          X_STEP_PIN= INVERT_X_STEP_PIN;
        }
  
        counter_y += current_block->steps_y;
        if (counter_y > 0) {
          Y_STEP_PIN= !INVERT_Y_STEP_PIN;
          counter_y -= current_block->step_event_count; 
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          Y_STEP_PIN= INVERT_Y_STEP_PIN;
        }
      #endif
  
  /*    #ifdef COREXY
        counter_x += current_block->steps_x;        
        counter_y += current_block->steps_y;
        
        if ((counter_x > 0)&&!(counter_y>0)){  //X step only
          X_STEP_PIN= !INVERT_X_STEP_PIN;
          Y_STEP_PIN= !INVERT_Y_STEP_PIN;
          counter_x -= current_block->step_event_count; 
          count_position[X_AXIS]+=count_direction[X_AXIS];         
          X_STEP_PIN= INVERT_X_STEP_PIN;
          Y_STEP_PIN= INVERT_Y_STEP_PIN;
        }
        
        if (!(counter_x > 0)&&(counter_y>0)){  //Y step only
          X_STEP_PIN= !INVERT_X_STEP_PIN;
          Y_STEP_PIN= !INVERT_Y_STEP_PIN;
          counter_y -= current_block->step_event_count; 
          count_position[Y_AXIS]+=count_direction[Y_AXIS];
          X_STEP_PIN= INVERT_X_STEP_PIN;
          Y_STEP_PIN= INVERT_Y_STEP_PIN;
        }        
        
        if ((counter_x > 0)&&(counter_y>0)){  //step in both axes
          if (((out_bits & (1<<X_AXIS)) == 0)^((out_bits & (1<<Y_AXIS)) == 0)){  //X and Y in different directions
            Y_STEP_PIN= !INVERT_Y_STEP_PIN;
            counter_x -= current_block->step_event_count;             
            Y_STEP_PIN= INVERT_Y_STEP_PIN;
            step_wait();
            count_position[X_AXIS]+=count_direction[X_AXIS];
            count_position[Y_AXIS]+=count_direction[Y_AXIS];
            Y_STEP_PIN= !INVERT_Y_STEP_PIN;
            counter_y -= current_block->step_event_count;
            Y_STEP_PIN= INVERT_Y_STEP_PIN;
          }
          else{  //X and Y in same direction
            X_STEP_PIN= !INVERT_X_STEP_PIN;
            counter_x -= current_block->step_event_count;             
            X_STEP_PIN= INVERT_X_STEP_PIN ;
            step_wait();
            count_position[X_AXIS]+=count_direction[X_AXIS];
            count_position[Y_AXIS]+=count_direction[Y_AXIS];
            X_STEP_PIN= !INVERT_X_STEP_PIN; 
            counter_y -= current_block->step_event_count;    
            X_STEP_PIN= INVERT_X_STEP_PIN;        
          }
        }
      #endif //corexy
  */    
      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        Z_STEP_PIN= !INVERT_Z_STEP_PIN;
        
		#ifdef Z_DUAL_STEPPER_DRIVERS
          Z2_STEP_PIN= !INVERT_Z_STEP_PIN;
        #endif
        
        counter_z -= current_block->step_event_count;
        count_position[Z_AXIS]+=count_direction[Z_AXIS];
        Z_STEP_PIN= INVERT_Z_STEP_PIN;
        
		#ifdef Z_DUAL_STEPPER_DRIVERS
        Z2_STEP_PIN= INVERT_Z_STEP_PIN;
        #endif
      }

      #ifndef ADVANCE  
	  
        counter_e += current_block->steps_e;
        if (counter_e > 0) {
          WRITE_E_STEP(!INVERT_E_STEP_PIN);
          counter_e -= current_block->step_event_count;
          count_position[E_AXIS]+=count_direction[E_AXIS];
          WRITE_E_STEP(INVERT_E_STEP_PIN);
        }
		
      #endif //!ADVANCE	 

      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
    }
	}
	{    // Calculare new timer value

    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long int)current_block->accelerate_until) 
	{ 
       MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
	//  printf("1:%ld\r\n",timer);//timer4_millis);
      TIM_SetAutoreload(TIM3, timer-1);
      acceleration_time += timer;
      #ifdef ADVANCE	  
        for(i=0; i < step_loops; i++) {
          advance += advance_rate;
        }
        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
        
      #endif
    } 
    else if (step_events_completed > (unsigned long int)current_block->decelerate_after) 
		{
	      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);
	      
	      if(step_rate > acc_step_rate) { // Check step_rate stays positive
	        step_rate = current_block->final_rate;
	      }
	      else {
	        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
	      }
	
	      // lower limit
	      if(step_rate < current_block->final_rate)
	        step_rate = current_block->final_rate;
	
	      // step_rate to timer interval
	      timer = calc_timer(step_rate);
		//  printf("2:%ld\r\n",timer);
	      TIM_SetAutoreload(TIM3, timer-1);
	      deceleration_time += timer;
	      #ifdef ADVANCE   
	        for(i=0; i < step_loops; i++) {
	          advance -= advance_rate;
	        }
	        if(advance < final_advance) advance = final_advance;
	        // Do E steps + advance steps
	        e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
	        old_advance = advance >>8;   			
	      #endif //ADVANCE	
	    }
	    else 
			{// printf("3:%ld\r\n",TIME3_nominal);
		      TIM_SetAutoreload(TIM3,TIME3_nominal-1);
		      // ensure we're running at the correct step rate, even if we just came off an acceleration
		    //  step_loops = step_loops_nominal;
		    }
   }     // Calculare new timer value
    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) 
	{
      current_block = NULL;
      plan_discard_current_block();
    }   
  } 
  }
}


void st_init(void)
{	 GPIO_InitTypeDef  GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
 	 //步进电机驱动引脚初始化
	// GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	 //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG, ENABLE);	 
	
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //速度为50MHz

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
	 GPIO_Init(GPIOA, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;		
	 GPIO_Init(GPIOC, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;			
	 GPIO_Init(GPIOD, &GPIO_InitStructure);	

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;	
	 GPIO_Init(GPIOG, &GPIO_InitStructure);	 

	//      E0_DET-->GPIOG.11,	EX_INPUT-->GPIOG.12,	

	//初始化限位引脚
	//		XMIN-->GPIOD.6,XMIN-->GPIOG.15,
	//      YMIN-->GPIOG.9,	YMIN-->GPIOG.14,
	//      ZMIN-->GPIOG.10,ZMIN-->GPIOG.13,
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOG,ENABLE);//使能PORTC,PORTE时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//PD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15|GPIO_Pin_9|GPIO_Pin_14|GPIO_Pin_10|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入 
	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOG9,10,14 ,15 ,13  


	
	 //电机驱动器使能   steppers default to disabled.
	 disable_x();	  
	 disable_y();	  
	 disable_z();	 
	 disable_e0();	  
	 disable_e1();	  

	 //初始化step pin
	 X_STEP_PIN=INVERT_X_STEP_PIN;
	 Y_STEP_PIN=INVERT_Y_STEP_PIN;
	 Z_STEP_PIN=INVERT_Z_STEP_PIN;
	 E0_STEP_PIN=INVERT_E_STEP_PIN;
	 E1_STEP_PIN=INVERT_E_STEP_PIN;


  // digipot_init(); //Initialize Digipot Motor Current
  //microstep_init(); //Initialize Microstepping Pins

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 0x4000; 
	TIM_TimeBaseStructure.TIM_Prescaler =35; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3 	
  ENABLE_STEPPER_DRIVER_INTERRUPT();  

  #ifdef ADVANCE	
 /* #if defined(TCCR0A) && defined(WGM01)
    TCCR0A &= ~(1<<WGM01);
    TCCR0A &= ~(1<<WGM00);
  #endif  
    e_steps[0] = 0;
    e_steps[1] = 0;
    e_steps[2] = 0;
    TIMSK0 |= (1<<OCIE0A);
 */
  #endif //ADVANCE	 //挤出电机提前机制
  
  enable_endstops(1); // Start with endstops active. After homing they can be disabled
 // sei();开启全局中断
}


// Block until all buffered steps are executed
void st_synchronize(void)
{
    while( blocks_queued()) {
    manage_heater();
    manage_inactivity();
  }
}

void st_set_position(const long x, const long y, const long z, const long e)
{
  CRITICAL_SECTION_START;
  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void st_set_e_position(const long e)
{
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;
  CRITICAL_SECTION_START;
  count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

void finishAndDisableSteppers(void)
{
  st_synchronize(); 
  disable_x(); 
  disable_y(); 
  disable_z(); 
  disable_e0(); 
  disable_e1(); 
}

void quickStop(void)
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}


void digipot_init(void) //Initialize Digipot Motor Current
{   const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;  
	int i;     
    for(i=0;i<=4;i++) 
      digipot_current(i,digipot_motor_current[i]);
}

void digipot_current(uint8_t driver, uint8_t current)
{
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
//	printf("%d:%d\r\n",digipot_ch[driver],current);
    digitalPotWrite(digipot_ch[driver], (uint8_t)current);
}

void digitalPotWrite(uint8_t address, uint8_t value) // From Arduino DigitalPotControl example
{
/*
    DIGIPOTSS_PIN=1; // take the SS pin low to select the chip
    SPI1_ReadWriteByte(address); //  send in the address and value via SPI:
    SPI1_ReadWriteByte(value);
    DIGIPOTSS_PIN=0; // take the SS pin high to de-select the chip:
	*/
}

void microstep_init(void)
{ int i;
  for(i=0;i<=4;i++) microstep_mode(i,8);
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2, int8_t ms3)
{/*
  if(ms1 > -1) switch(driver)
  {
    case 0:X_MS1_PIN=ms1 ; break;
    case 1:Y_MS1_PIN=ms1 ; break;
    case 2:Z_MS1_PIN=ms1 ; break;
    case 3:E0_MS1_PIN=ms1 ; break;
    case 4:E1_MS1_PIN=ms1 ; break;
	default:  break;
  }
  if(ms2 > -1) 
  switch(driver)
  {
    case 0:X_MS2_PIN=ms2 ; break;
    case 1:Y_MS2_PIN=ms2 ; break;
    case 2:Z_MS2_PIN=ms2 ; break;
    case 3:E0_MS2_PIN=ms2 ; break;
    case 4:E1_MS2_PIN=ms2 ; break;
	default:  break;
  }
    if(ms3 > -1) switch(driver)
  {
    case 0:X_MS3_PIN=ms3 ; break;
    case 1:Y_MS3_PIN=ms3 ; break;
    case 2:Z_MS3_PIN=ms3 ; break;
    case 3:E0_MS3_PIN=ms3 ; break;
    case 4:E1_MS3_PIN=ms3 ; break;
	default:  break;
  }
	*/
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{ switch(driver)
  {
    case 0: subsection_x_value=stepping_mode; break;
    case 1: subsection_y_value=stepping_mode; break;
    case 2: subsection_z_value=stepping_mode; break;
    case 3: subsection_e0_value=stepping_mode; break;
    case 4: subsection_e1_value=stepping_mode; break;
	default:  break;
  }
  switch(stepping_mode)
  {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
    case 32: microstep_ms(driver,MICROSTEP32); break;
    case 64: microstep_ms(driver,MICROSTEP64); break;
    case 128: microstep_ms(driver,MICROSTEP128); break;
	default:  break;
  }
}
void microstep_readings(void)
{
	printf("Motor_Subsection \n");
	printf("X: %d\n",subsection_x_value);
	printf("Y: %d\n",subsection_y_value);
	printf("Z: %d\n",subsection_z_value);
	printf("E0: %d\n",subsection_e0_value);
	printf("E1: %d\n",subsection_e1_value);
}

