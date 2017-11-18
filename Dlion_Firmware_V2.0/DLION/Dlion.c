#include "Dlion.h"
#include "delay.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "usart.h"
#include "beep.h"


/*******************************************************本程序开源供3D打印学习使用************************************************
																												Dlion-3D打印主板
																												3D二进制创客---------技术论坛:www.3dbinmaker.com
																												文件说明：Dlion核心处理算法  版本：V1.0
																												Copyright(C)深圳洛众科技有限公司
																												All rights reserved
***********************************************************************************************************************************/


//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M15  - update flash data (font data ; icon data and so on)
// M16  - screen_adjust
// M17  - Enable/Power all stepper motors 
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Dlion!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================public variables=============================
//===========================================================================
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply=100; //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

static const float  base_min_pos[3]   = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };  
static float  base_max_pos[3]   = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
static const float  base_home_pos[3]  = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
static float  max_length[3]	    = { X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH };
static const float  home_retract_mm[3]= { X_HOME_RETRACT_MM, Y_HOME_RETRACT_MM, Z_HOME_RETRACT_MM };
static const signed char home_dir[3]  = { X_HOME_DIR, Y_HOME_DIR, Z_HOME_DIR };

// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif
uint8_t active_extruder = 0;
int fanSpeed=0;
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=true;
  bool retracted=false;
  float retract_length=3, retract_feedrate=17*60, retract_zlift=0.8;
  float retract_recover_length=0, retract_recover_feedrate=8*60;
#endif
//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
 float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

volatile static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
volatile static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static bool comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;
static u8 tmp_extruder;


bool Stopped=false;

//#if NUM_SERVOS > 0
//  Servo servos[NUM_SERVOS];
//#endif
//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates(void);
bool setTargetedHotend(int code);


void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    printf("enqueing \"%s\"",cmdbuffer[bufindw]);
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}


void setup(void)
{
  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  st_init();    // Initialize stepper, this enables interrupts
  tp_init();    // Initialize 
  plan_init();  // Initialize planner;
}

 void loop(void)
{
 while(1)
{ 
		if(buflen < (BUFSIZE-1))
		 { 
			 get_command();
		 }

		 if(buflen) 
		 {
					process_commands();  
					if(buflen > 0)
					{
						buflen = (buflen-1);
						bufindr = (bufindr + 1)%BUFSIZE;
					}
		 }
  
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
 }
}

void get_command(void)
{ 
  while( MYSERIAL_available() > 0  && buflen < BUFSIZE)
  {	
    serial_char = MYSERIAL_read();
    if(serial_char == '\n' ||serial_char == '\r' ||(serial_char == ':' && comment_mode == false) ||serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) 
	    {
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode)
	    {
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL)	                                                       
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], PSTR("M110")) == NULL) )
		      {                                                                             
            SERIAL_ERROR_START;
            printf(MSG_ERR_LINE_NO);
            printf("%ld",gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            u8 checksum = 0;
            u8 count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');
            if( (u8)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
							{                                                                
									SERIAL_ERROR_START;
									printf(MSG_ERR_CHECKSUM_MISMATCH);
									printf(" checksum: %d\n\r",checksum);
									count = 0;
								  printf(" '");
									while(cmdbuffer[bufindw][count] != '*')
									{
										printf("%c",cmdbuffer[bufindw][count++]);
									}
									printf(" '\n\r ");
									checksum = 0;
									count = 0;
									while(cmdbuffer[bufindw][count] != '*')
									{ 
										printf("cmdbuffer:%d;",cmdbuffer[bufindw][count]);
										checksum = checksum^cmdbuffer[bufindw][count++];
									  printf(" checksum:%d \n\r",checksum);
									}
									///	printf("\n\r ");
						
									printf("%ld",gcode_LastN);
									FlushSerialRequestResend();
									serial_count = 0;
									return;
            }
            //if no errors, continue parsing
          }
          else
          {
            SERIAL_ERROR_START;
            printf(MSG_ERR_NO_CHECKSUM);
            printf("%ld",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }

          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            SERIAL_ERROR_START;
            printf(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            printf("%ld",gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        if((strchr(cmdbuffer[bufindw], 'G') != NULL))
		    {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
						{
							case 0:
							case 1:
							case 2:
							case 3:
								if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
									printf(MSG_OK);
									printf("\n");
								}
								else {
									printf(MSG_ERR_STOPPED);
								}
								break;
							default:
								break;
             }
        }
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;//sanse
      }
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}


 float code_value(void)
 {

  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long(void)
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}


void axis_is_at_home(int axis) 
{
  current_position[axis] = base_home_pos[axis] + add_homeing[axis];
  min_pos[axis] =          base_min_pos[axis] + add_homeing[axis];
  max_pos[axis] =          base_max_pos[axis] + add_homeing[axis];
}
#define HOMEAXIS_DO(LETTER) (( LETTER##_HOME_DIR==-1) || (LETTER##_HOME_DIR==1))
static void homeaxis(int axis) 
{
  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) 	//
   {
    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = 1.5 * max_length[axis] * home_dir[axis];
    feedrate = homing_feedrate[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm[axis] * home_dir[axis];
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm[axis] * home_dir[axis];
    feedrate = homing_feedrate[axis]/2 ;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
void process_commands(void)
{ unsigned long codenum; //throw away variable
  char *starpos = NULL;
  int8_t i;
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
		//  printf("\n");
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity();
      }
      break;
      #ifdef FWRETRACT	   //ONLY PARTIALLY TESTED
      case 10: // G10 retract
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];
        current_position[Z_AXIS]+=-retract_zlift;
        destination[E_AXIS]=current_position[E_AXIS]-retract_length;
        feedrate=retract_feedrate;
        retracted=true;
        prepare_move();
      }

      break;
      case 11: // G10 retract_recover
      if(!retracted)
      {
        destination[X_AXIS]=current_position[X_AXIS];
        destination[Y_AXIS]=current_position[Y_AXIS];
        destination[Z_AXIS]=current_position[Z_AXIS];

        current_position[Z_AXIS]+=retract_zlift;
        current_position[E_AXIS]+=-retract_recover_length;
        feedrate=retract_recover_feedrate;
        retracted=false;
        prepare_move();
      }
      break;
      #endif //FWRETRACT   //ONLY PARTIALLY TESTED
    case 28: //G28 Home all Axis one at a time
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;
      home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      #ifdef QUICK_HOME
      if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
      {
        current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
        feedrate = homing_feedrate[X_AXIS];
        if(homing_feedrate[Y_AXIS]<feedrate)
          feedrate =homing_feedrate[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        HOMEAXIS(X);
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      if(code_seen(axis_codes[X_AXIS]))
      {
        if(code_value_long() != 0) {
          current_position[X_AXIS]=code_value()+add_homeing[0];
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Y_AXIS]=code_value()+add_homeing[1];
        }
      }

      if(code_seen(axis_codes[Z_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value()+add_homeing[2];
        }
      }
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
      break;
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
	  #ifdef DEBUG_PRINTF
	  printf("relative_mode = true;");
	  #endif
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
             current_position[i] = code_value()+add_homeing[i];
             plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
    }
  }
  else if(code_seen('M'))
  {	switch( (int)code_value() )
    { 
  	 
	 
	    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
	    case 1: // M1 - Conditional stop - Wait for user button press on LCD
			{
		     #ifdef ULTIPANEL
		      codenum = 0;
		      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
		      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
		
		      st_synchronize();
		      previous_millis_cmd = millis();
		      if (codenum > 0){
		        codenum += millis();  // keep track of when we started waiting
		        while(millis()  < codenum && !LCD_CLICKED){
		          manage_heater();
		          manage_inactivity();
		        }
		      }else{
		        while(!LCD_CLICKED){
		          manage_heater();
		          manage_inactivity();
		        }
		      }
		   	  #endif
		    } break;
			case 15:
						break;
			case 16:
						break;
	    case 17:
	        enable_x();
	        enable_y();
	        enable_z();
	        enable_e0();
	        enable_e1();
            break;
	    case 31: //M31 take time since the start of the SD print or an M109 command
		      {
		     // char time[30];
		      unsigned long t=(stoptime-starttime)/1000;
		      int sec,min;
					stoptime=millis();
		      min=t/60;
		      sec=t%60;
		      printf("%d min, %d sec", min, sec);
		      SERIAL_ECHO_START;
		      autotempShutdown();
		      }
		      break;
		case 42: //M42 -Change pin status via gcode
		    /*  if (code_seen('S'))
		      {
		        int pin_status = code_value();
		        int pin_number = LED_PIN;
		        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
		          pin_number = code_value();
		        for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
		        {
		          if (sensitive_pins[i] == pin_number)
		          {
		            pin_number = -1;
		            break;
		          }
		        }
		      #if defined(FAN_PIN)
		        if (pin_number == FAN_PIN)
		          fanSpeed = pin_status;
		      #endif
		        if (pin_number > -1)
		        {
		          pinMode(pin_number, OUTPUT);
		          digitalWrite(pin_number, pin_status);
		          analogWrite(pin_number, pin_status);
		        }
		      }
			  */
		     break;
		case 104: // M104
		      if(setTargetedHotend(104))
					{
		        break;
		      }
		      if (code_seen('S')) 
					{	
						setTargetHotend(code_value(), tmp_extruder);
						
					}
		      setWatch();
		      break;
	    case 140: // M140 set bed temp
		      if (code_seen('S')) setTargetBed(code_value());
		      break;
	    case 105 : // M105
		      if(setTargetedHotend(105)){
		        break;
		      }
		      #if defined(TEMP_0_PIN)
			  		printf("ok T:%.1f /%.1f",degHotend(tmp_extruder),degTargetHotend(tmp_extruder));
		       // SERIAL_PROTOCOLPGM("ok T:");
		       // SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
		       // SERIAL_PROTOCOLPGM(" /");
		      //  SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
		      		#if defined(TEMP_BED_PIN)
			  			printf(" B:%.1f /%.1f",degBed(),degTargetBed());
		        //  SERIAL_PROTOCOLPGM(" B:");
		        //  SERIAL_PROTOCOL_F(degBed(),1);
		        //  SERIAL_PROTOCOLPGM(" /");
		        //  SERIAL_PROTOCOL_F(degTargetBed(),1);
		       		#endif //TEMP_BED_PIN
		      #else
		        SERIAL_ERROR_START;
		        printf(MSG_ERR_NO_THERMISTORS);
		      #endif
		
		      //  SERIAL_PROTOCOLPGM(" @:");
		        printf(" @:%d",getHeaterPower(tmp_extruder));
				    printf(" B@:%d\n",getHeaterPower(-1));
		      //  SERIAL_PROTOCOLPGM(" B@:");
		      //  SERIAL_PROTOCOL(getHeaterPower(-1));
		      //  SERIAL_PROTOCOLLN("");
			  return;
		    //  break;
		case 109:
			{// M109 - Wait for extruder heater to reach target.
				bool target_direction;
				long residencyStart;
				if(setTargetedHotend(109))break;
		        #ifdef AUTOTEMP
		        	 autotemp_enabled=false;
		        #endif
			    if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
		        #ifdef AUTOTEMP
			        if (code_seen('S')) autotemp_min=code_value();
			        if (code_seen('B')) autotemp_max=code_value();	
			        if (code_seen('F'))
			        {
			          autotemp_factor=code_value();
			          autotemp_enabled=true;
			        }
			    #endif
				setWatch();
      			codenum = millis();
		      	/* See if we are heating up or cooling down */
      			target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling
				#ifdef TEMP_RESIDENCY_TIME
		        	residencyStart = -1;
		        /* continue to loop until we have reached the target temp
		          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
			        while((residencyStart == -1) || (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) 
						  { if( (millis() - codenum) > 1000UL )
							  { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
							    printf("T:%.1f E:%d",degHotend(tmp_extruder),tmp_extruder);
					          //  SERIAL_PROTOCOLPGM("T:");
					          //  SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
					          //  SERIAL_PROTOCOLPGM(" E:");
					           // SERIAL_PROTOCOL((int)tmp_extruder);
					            #ifdef TEMP_RESIDENCY_TIME
					              printf(" W:");
					              if(residencyStart > -1)
					              {
					                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
					                 printf("%ld\n", codenum );
					              }
					              else
					              {
					                 printf("?\n");
					              }
					            #else
					              printf("\n");
					            #endif
					            codenum = millis();
					          }
							      manage_heater();
          					  manage_inactivity();
					         #ifdef TEMP_RESIDENCY_TIME
					            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
					              or when current temp falls outside the hysteresis after target temp was reached */
						         if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
						              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
						              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
						          {
						            residencyStart = millis();
						          }
					        #endif //TEMP_RESIDENCY_TIME
						  }
		     	#else
		        	while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) 
						  { if( (millis() - codenum) > 1000UL )
							  { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
							    printf("T:%.1f E:%d",degHotend(tmp_extruder),tmp_extruder);
					          //  SERIAL_PROTOCOLPGM("T:");
					          //  SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
					          //  SERIAL_PROTOCOLPGM(" E:");
					           // SERIAL_PROTOCOL((int)tmp_extruder);
					            #ifdef TEMP_RESIDENCY_TIME
					              printf(" W:");
					              if(residencyStart > -1)
					              {
					                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
					                 printf("%ld\n", codenum );
					              }
					              else
					              {
					                 printf("?\n");
					              }
					            #else
					              printf("\n");
					            #endif
					            codenum = millis();
					          }
							  manage_heater();
          					  manage_inactivity();
					         #ifdef TEMP_RESIDENCY_TIME
					            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
					              or when current temp falls outside the hysteresis after target temp was reached */
						         if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
						              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
						              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
						          {
						            residencyStart = millis();
						          }
					        #endif //TEMP_RESIDENCY_TIME
						  }
		      	#endif //TEMP_RESIDENCY_TIME
			    starttime=millis();
                previous_millis_cmd = millis();
			}break;
		case 190: // M190 - Wait for bed heater to reach target.
		    #if defined(TEMP_BED_PIN)
		        if (code_seen('S')) setTargetBed(code_value());
		        codenum = millis();
		        while(isHeatingBed())
		        {
		          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
		          {
		            float tt=degHotend(active_extruder);
								printf("T:%.1f E:%d B:%.1f\n",tt,active_extruder,degBed());
		           // SERIAL_PROTOCOLPGM("T:");
		           // SERIAL_PROTOCOL(tt);
		           // SERIAL_PROTOCOLPGM(" E:");
		          //  SERIAL_PROTOCOL((int)active_extruder);
		          //  SERIAL_PROTOCOLPGM(" B:");
		          //  SERIAL_PROTOCOL_F(degBed(),1);
		           // SERIAL_PROTOCOLLN("");
		            codenum = millis();
		          }
		          manage_heater();
		          manage_inactivity();
		        }
		        previous_millis_cmd = millis();
		    #endif
		    break;

	      case 106: //M106 Fan On
		   #if defined(FAN_PIN) 
		        if (code_seen('S')){
		           fanSpeed=constrain(code_value(),0,255);
				  
			//	   test(fanSpeed);
				  // soft_pwm[0]=fanSpeed;
		        }
		        else {
		          fanSpeed=255;
				 /// test(255);
				  //	soft_pwm[0]=255;
		        }
				
		   #endif //FAN_PIN
	        break;
	      case 107: //M107 Fan Off
	        	fanSpeed = 0;
			//	test(0);
			//	soft_pwm[0]=0;
	        break;
		  case 126: //M126 valve open
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_1_PIN) 
				    if (code_seen('S')){
			             ValvePressure=constrain(code_value(),0,255);
			          }
			          else {
			            ValvePressure=255;
			          }
			          
				#endif
			#endif
			break;
		  case 127: //M127 valve closed
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_1_PIN) 
			          ValvePressure = 0;
				#endif
			#endif
			break;
		 case 128://M128 valve open
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_2_PIN) 
			          if (code_seen('S')){
			             EtoPPressure=constrain(code_value(),0,255);
			          }
			          else {
			            EtoPPressure=255;
			          }
				#endif
			#endif
			break;
		  case 129: //M129 valve closed
			#ifdef BARICUDA
	        // PWM for HEATER_1_PIN
		        #if defined(HEATER_2_PIN) 
			          EtoPPressure = 0;
				#endif
			#endif
			break;
		 case 80: // M80 - ATX Power On
	         // SET_OUTPUT(PS_ON_PIN); //GND	////////////////////////////////////////////////////
	         //  WRITE(PS_ON_PIN, PS_ON_AWAKE);////////////////////////////////////////////////////
	        break;
	     case 81: // M81 - ATX Power Off
		 	break;
	     case 82:
	        axis_relative_modes[3] = false;
	        break;
	     case 83:
	        axis_relative_modes[3] = true;
	        break;
	     case 18: //compatibility
	     case 84: // M84
	        if(code_seen('S'))
			{
	          stepper_inactive_time = code_value() * 1000;
	        }
			else
			{
		        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
		        if(all_axis)
		        {
		          st_synchronize();
		          disable_e0();
		          disable_e1();
		        //  disable_e2();
		          finishAndDisableSteppers();
		        }
		        else
		        {
		          st_synchronize();
		          if(code_seen('X')) disable_x();
		          if(code_seen('Y')) disable_y();
		          if(code_seen('Z')) disable_z();
		         // #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
		            if(code_seen('E')) {
		              disable_e0();
		              disable_e1();
		            //  disable_e2();
		            }
		        //  #endif
		        }
		     }
			 break;
		  case 85: // M85
		     code_seen('S');
		     max_inactive_time = code_value() * 1000;
		     break;
		  case 92: // M92
		      for(i=0; i < NUM_AXIS; i++)
		      {
		        if(code_seen(axis_codes[i]))
		        {
		          if(i == 3) { // E
		            float value = code_value();
		            if(value < 20.0) {
		              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
		              max_e_jerk *= factor;
		              max_feedrate[i] *= factor;
		              axis_steps_per_sqr_second[i] *= factor;
		            }
		            axis_steps_per_unit[i] = value;
		          }
		          else {
		            axis_steps_per_unit[i] = code_value();
		          }
		        }
		      }
			 break;
	      case 115: // M115
		      printf(MSG_M115_REPORT);
		      break;
	      case 117: // M117 display message		/////////////////////////////////////////////////////
		      starpos = (strchr(strchr_pointer + 5,'*'));
		      if(starpos!=NULL)
		        *(starpos-1)='\0';
		      break;
		  case 114: // M114	 
				  printf("X:%f Y:%f Z:%f E:%f",current_position[X_AXIS],current_position[Y_AXIS],current_position[Z_AXIS],current_position[E_AXIS]);			
			    printf(MSG_COUNT_X);
				  printf("%f Y:%f Z:%f\n",((float)st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS],((float)st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS],((float)st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
		      break; 
		   case 120: // M120
		      enable_endstops(false) ;
		      break;
		   case 121: // M121
		      enable_endstops(true) ;
		      break;
		   case 119: // M119
		      printf(MSG_M119_REPORT);
			  printf("\n");
		      #if defined(X_MIN_PIN) 
		        printf(MSG_X_MIN);
				printf(((X_MIN_PIN==X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		       // (X_MIN_PIN==X_ENDSTOPS_INVERTING) ? (printf(MSG_ENDSTOP_HIT)) : (printf(MSG_ENDSTOP_OPEN));
		      #endif
		      #if defined(X_MAX_PIN)
		        printf(MSG_X_MAX);
		        printf(((X_MAX_PIN==X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Y_MIN_PIN)
		        printf(MSG_Y_MIN);
		        printf(((Y_MIN_PIN^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Y_MAX_PIN) 
		        printf(MSG_Y_MAX);
		        printf(((Y_MAX_PIN^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Z_MIN_PIN) 
		        printf(MSG_Z_MIN);
		        printf(((Z_MIN_PIN^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      #if defined(Z_MAX_PIN)
		        printf(MSG_Z_MAX);
		        printf(((Z_MAX_PIN^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
				printf("\n");
		      #endif
		      break;
			  //TODO: update for all axis, use for loop
		  case 201: // M201	 
		      for( i=0; i < NUM_AXIS; i++)
		      {
		        if(code_seen(axis_codes[i]))
		        {
		          max_acceleration_units_per_sq_second[i] = code_value();
		        }
		      }
		      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		      reset_acceleration_rates();
		      break;
	      case 202: // M202
		  	#if 0 // Not used for Sprinter/grbl gen6
		      for(i=0; i < NUM_AXIS; i++) {
		        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
		      }
			#endif
		      break;
	      case 203: // M203 max feedrate mm/sec
		      for( i=0; i < NUM_AXIS; i++) 
				  {
		        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
		      }
		      break; 
	      case 204: // M204 acclereration S normal moves T filmanent only moves
		      {
		        if(code_seen('S')) acceleration = code_value() ;
		        if(code_seen('T')) retract_acceleration = code_value() ;
		      }
	          break;
	      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
		    {
		      if(code_seen('S')) minimumfeedrate = code_value();
		      if(code_seen('T')) mintravelfeedrate = code_value();
		      if(code_seen('B')) minsegmenttime = code_value() ;
		      if(code_seen('X')) max_xy_jerk = code_value() ;
		      if(code_seen('Z')) max_z_jerk = code_value() ;
		      if(code_seen('E')) max_e_jerk = code_value() ;
		    }
	    	break;
	      case 206: // M206 additional homeing offset
		      for( i=0; i < 3; i++)
		      {
		        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
		      }
		      break;
        #ifdef FWRETRACT
		    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
		    {
		      if(code_seen('S'))
		      {
		        retract_length = code_value() ;
		      }
		      if(code_seen('F'))
		      {
		        retract_feedrate = code_value() ;
		      }
		      if(code_seen('Z'))
		      {
		        retract_zlift = code_value() ;
		      }
		    }break;
		    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
		    {
		      if(code_seen('S'))
		      {
		        retract_recover_length = code_value() ;
		      }
		      if(code_seen('F'))
		      {
		        retract_recover_feedrate = code_value() ;
		      }
		    }break;
		    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
		    {
		      if(code_seen('S'))
		      {
		        int t= code_value() ;
		        switch(t)
		        {
		          case 0: autoretract_enabled=false;retracted=false;break;
		          case 1: autoretract_enabled=true;retracted=false;break;
		          default:
		            SERIAL_ECHO_START;
		            printf(MSG_UNKNOWN_COMMAND);
		            printf("%d",cmdbuffer[bufindr]);
		            printf("\"");
		        }
		      }
		
		    }break;
   			#endif // FWRETRACT
		    #if EXTRUDERS > 1
			case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
			    {
			      if(setTargetedHotend(218)){
			        break;
			      }
			      if(code_seen('X'))
			      {
			        extruder_offset[X_AXIS][tmp_extruder] = code_value();
			      }
			      if(code_seen('Y'))
			      {
			        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
			      }
			      SERIAL_ECHO_START;
			      printf(MSG_HOTEND_OFFSET);
			      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
			      {
			         //SERIAL_ECHO(" ");
			         printf(" %f,%f",extruder_offset[X_AXIS][tmp_extruder],extruder_offset[Y_AXIS][tmp_extruder]);
			        // SERIAL_ECHO(",");
			       //  SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
			      }
			      printf("\n");
			    }break;
		   #endif
		   case 220: // M220 S<factor in percent>- set speed factor override percentage
			    {
			      if(code_seen('S'))
			      {
			        feedmultiply = code_value() ;
			      }
			    }
		      break;
		   case 221: // M221 S<factor in percent>- set extrude factor override percentage
			    {
			      if(code_seen('S'))
			      {
			        extrudemultiply = code_value() ;
			      }
			    }
		      break;
		   #if NUM_SERVOS > 0
		   case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
		      {
		        int servo_index = -1;
		        int servo_position = 0;
		        if (code_seen('P'))
		          servo_index = code_value();
		        if (code_seen('S')) {
		          servo_position = code_value();
		          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
		            servos[servo_index].write(servo_position);
		          }
		          else {
		            SERIAL_ECHO_START;
		            SERIAL_ECHO("Servo ");
		            SERIAL_ECHO(servo_index);
		            SERIAL_ECHOLN(" out of range");
		          }
		        }
		        else if (servo_index >= 0) {
		          SERIAL_PROTOCOL(MSG_OK);
		          SERIAL_PROTOCOL(" Servo ");
		          SERIAL_PROTOCOL(servo_index);
		          SERIAL_PROTOCOL(": ");
		          SERIAL_PROTOCOL(servos[servo_index].read());
		          SERIAL_PROTOCOLLN("");
		        }
		      }
		      break;
		    #endif // NUM_SERVOS > 0
		   // #if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
		    case 300: // M300
		    {
		      int beepP = 1000;
		      if(code_seen('P')) beepP = code_value();
						BEEP=1;  
						delay_ms(beepP);
						BEEP=0;
		    }
		    break;
		 //   #endif // M300
       #ifdef PIDTEMP
	    case 301: // M301
	      {
	        if(code_seen('P')) Kp = code_value();
	        if(code_seen('I')) Ki = scalePID_i(code_value());
	        if(code_seen('D')) Kd = scalePID_d(code_value());
	
	        #ifdef PID_ADD_EXTRUSION_RATE
	        if(code_seen('C')) Kc = code_value();
	        #endif
	
	        updatePID();
	        printf(MSG_OK);
					printf(" p:%f i:%f d:%f",Kp,unscalePID_i(Ki),unscalePID_d(Kd));
	       // SERIAL_PROTOCOL(" p:");
	       // SERIAL_PROTOCOL(Kp);
	       // SERIAL_PROTOCOL(" i:");
	       // SERIAL_PROTOCOL(unscalePID_i(Ki));
	       // SERIAL_PROTOCOL(" d:");
	       // SERIAL_PROTOCOL(unscalePID_d(Kd));
	        #ifdef PID_ADD_EXTRUSION_RATE
	       
	        //Kc does not have scaling applied above, or in resetting defaults
			printf(" c:%f",Kc);
	      //  SERIAL_PROTOCOL(Kc);
	        #endif
	        printf("\n");
	      }
	      break;
	    #endif //PIDTEMP
	    #ifdef PIDTEMPBED
	    case 304: // M304
	      {
	        if(code_seen('P')) bedKp = code_value();
	        if(code_seen('I')) bedKi = scalePID_i(code_value());
	        if(code_seen('D')) bedKd = scalePID_d(code_value());
	
	        updatePID();
					printf(MSG_OK);
					printf(" p:%f i:%f d:%f",Kp,unscalePID_i(bedKi,unscalePID_d(bedKd)));
					printf("\n");
	//         SERIAL_PROTOCOL(MSG_OK);
	//         SERIAL_PROTOCOL(" p:");
	//         SERIAL_PROTOCOL(bedKp);
	//         SERIAL_PROTOCOL(" i:");
	//         SERIAL_PROTOCOL(unscalePID_i(bedKi));
	//         SERIAL_PROTOCOL(" d:");
	//         SERIAL_PROTOCOL(unscalePID_d(bedKd));
	//         SERIAL_PROTOCOLLN("");
	      }
	      break;
	    #endif //PIDTEMP
	    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
		     {
		      #if defined(PHOTOGRAPH_PIN)
		//         const uint8_t NUM_PULSES=16;
		//         const float PULSE_LENGTH=0.01524;
		//         for(int i=0; i < NUM_PULSES; i++) {
		//           WRITE(PHOTOGRAPH_PIN, HIGH);
		//           _delay_ms(PULSE_LENGTH);
		//           WRITE(PHOTOGRAPH_PIN, LOW);
		//           _delay_ms(PULSE_LENGTH);
		//         }
		//         delay(7.33);
		//         for(int i=0; i < NUM_PULSES; i++) {
		//           WRITE(PHOTOGRAPH_PIN, HIGH);
		//           _delay_ms(PULSE_LENGTH);
		//           WRITE(PHOTOGRAPH_PIN, LOW);
		//           _delay_ms(PULSE_LENGTH);
		//         }
		      #endif
		     }
		    break;
		    case 302: // allow cold extrudes
		    {
		      allow_cold_extrudes(true);
		    }
		    break;
		    case 303: // M303 PID autotune
		    {
		      float temp = 150.0;
		      int e=0;
		      int c=5;
		      if (code_seen('E')) e=code_value();
		        if (e<0)
		          temp=70;
		      if (code_seen('S')) temp=code_value();
		      if (code_seen('C')) c=code_value();
		      PID_autotune(temp, e, c);
		    }
		    break;
		    case 400: // M400 finish all moves
		    {
		      st_synchronize();
		    }
		    break;
		    case 500: // M500 Store settings in EEPROM
		    {
		        Config_StoreSettings();////////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
		    case 501: // M501 Read settings from EEPROM
		    {
		        Config_RetrieveSettings();////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
		    case 502: // M502 Revert to default settings
		    {
		        Config_ResetDefault();//////////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
		    case 503: // M503 print settings currently in memory
		    {
		        Config_PrintSettings();///////////////////////////////////////////////////////////////////////////////////////////
		    }
		    break;
	    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
		 case 540:
		    {
		        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
		    }
		    break;
		#endif

	    #ifdef FILAMENTCHANGEENABLE
	    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
	    {
	        float target[4];
	        float lastpos[4];
	        target[X_AXIS]=current_position[X_AXIS];
	        target[Y_AXIS]=current_position[Y_AXIS];
	        target[Z_AXIS]=current_position[Z_AXIS];
	        target[E_AXIS]=current_position[E_AXIS];
	        lastpos[X_AXIS]=current_position[X_AXIS];
	        lastpos[Y_AXIS]=current_position[Y_AXIS];
	        lastpos[Z_AXIS]=current_position[Z_AXIS];
	        lastpos[E_AXIS]=current_position[E_AXIS];
	        //retract by E
	        if(code_seen('E'))
	        {
	          target[E_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_FIRSTRETRACT
	            target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
	          #endif
	        }
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        //lift Z
	        if(code_seen('Z'))
	        {
	          target[Z_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_ZADD
	            target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
	          #endif
	        }
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        //move xy
	        if(code_seen('X'))
	        {
	          target[X_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_XPOS
	            target[X_AXIS]= FILAMENTCHANGE_XPOS ;
	          #endif
	        }
	        if(code_seen('Y'))
	        {
	          target[Y_AXIS]= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_YPOS
	            target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
	          #endif
	        }
	
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        if(code_seen('L'))
	        {
	          target[E_AXIS]+= code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_FINALRETRACT
	            target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
	          #endif
	        }
	
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);
	
	        //finish moves
	        st_synchronize();
	        //disable extruder steppers so filament can be removed
	        disable_e0();
	        disable_e1();
	        disable_e2();
	        delay_ms(100);
	        cnt=0;
	         while(!lcd_clicked){
	          cnt++;
	           manage_heater();
	           manage_inactivity();
	           if(cnt==0)
	           {
				beep();
	           }
	         }
	
	        //return to normal
	        if(code_seen('L'))
	        {
	          target[E_AXIS]+= -code_value();
	        }
	        else
	        {
	          #ifdef FILAMENTCHANGE_FINALRETRACT
	            target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
	          #endif
	        }
	        current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
	        plan_set_e_position(current_position[E_AXIS]);
	        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
	        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
	        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
	        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
	    }
	    break;
	    #endif //FILAMENTCHANGEENABLE
	    case 907: // M907 Set digital trimpot motor current using axis codes.
	    {
	        for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
	        if(code_seen('B')) digipot_current(4,code_value());
	        if(code_seen('S')) for(i=0;i<=4;i++) digipot_current(i,code_value());
	    }
	    break;
	    case 908: // M908 Control digital trimpot directly.
	    {
	        uint8_t channel,current;
	        if(code_seen('P')) channel=code_value();
	        if(code_seen('S')) current=code_value();
	        digipot_current(channel, current);
	    }
	    break;
	    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
	    { 
	        if(code_seen('S')) for( i=0;i<=4;i++) microstep_mode(i,code_value());
	        for( i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
	        if(code_seen('B')) microstep_mode(4,code_value());
	        microstep_readings();
	    }
	    break;
	    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
	    {
	      if(code_seen('S')) switch((int)code_value())
	      {
	        case 1:
	          for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1,-1);
	          if(code_seen('B')) microstep_ms(4,code_value(),-1,-1);
	          break;
	        case 2:
	          for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value(),-1);
	          if(code_seen('B')) microstep_ms(4,-1,code_value(),-1);
	          break;
					case 3:
	          for(i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,-1,code_value());
	          if(code_seen('B')) microstep_ms(4,-1,-1,code_value());
	          break;
	      }
	      microstep_readings();
	
	    }
	    break;
	case 999: // M999: Restart after being stopped
	      Stopped = false;
	    //  lcd_reset_alert_level();//////////////////////////////////////
	      gcode_LastN = Stopped_gcode_LastN;
	      FlushSerialRequestResend();
	    break;		
	
	}//end switch(int) code_value();

  }
   else if(code_seen('T'))
   { tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      printf("T%d",tmp_extruder);
     // SERIAL_ECHO(tmp_extruder);
      printf(MSG_INVALID_EXTRUDER);
    }
	else 
	{volatile bool make_move = false;
      if(code_seen('F')) 
	  {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
	  #if EXTRUDERS > 1
	  if(tmp_extruder != active_extruder) 
	  {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
        // Offset extruder (only by XY)
        for(i = 0; i < 2; i++) 
					{
           current_position[i] = current_position[i] -
                                 extruder_offset[i][active_extruder] +
                                 extruder_offset[i][tmp_extruder];
        }
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
           prepare_move();
        }
      }	//end   if(tmp_extruder != active_extruder) 
	   #endif
	  SERIAL_ECHO_START;
      printf(MSG_ACTIVE_EXTRUDER);
      printf("%d",active_extruder);
	  printf("\n");
	}
  }//end else if(code_seen('T'))
  else
  { SERIAL_ECHO_START;
    printf(MSG_UNKNOWN_COMMAND);
    printf("%s",cmdbuffer[bufindr]);
    printf("\"");
  }
   ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL_flush();
  printf(MSG_RESEND);
  printf("%d\n",gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  printf(MSG_OK);
  printf("\n");
}

void get_coordinates()
{ int8_t i;
  volatile bool seen[4]={false,false,false,false};
  for( i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
	#ifdef DEBUG_PRINTF 
	printf("%d：%f\n",i,destination[i]);
	#endif

  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  	#ifdef DEBUG_PRINTF 
	printf("feedrate：%f\n",feedrate);
	#endif

  #ifdef FWRETRACT
  if(autoretract_enabled)
  if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
  {
    float echange=destination[E_AXIS]-current_position[E_AXIS];
    if(echange<-MIN_RETRACT) //retract
    {
      if(!retracted)
      {

      destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
      //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
      float correctede=-echange-retract_length;
      //to generate the additional steps, not the destination is changed, but inversely the current position
      current_position[E_AXIS]+=-correctede;
      feedrate=retract_feedrate;
      retracted=true;
      }

    }
    else
      if(echange>MIN_RETRACT) //retract_recover
    {
      if(retracted)
      {
      //current_position[Z_AXIS]+=-retract_zlift;
      //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
      float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
      current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
      feedrate=retract_recover_feedrate;
      retracted=false;
      }
    }

  }
  #endif //FWRETRACT
}
void get_arc_coordinates(void)
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}
void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void prepare_move(void)
{  int8_t i;
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
  for(i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(u8 isclockwise) {
	int8_t i;
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}
/*
#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN 
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif  
unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan(void)
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!X_ENABLE_PIN || !Y_ENABLE_PIN || !Z_ENABLE_PIN
    #if EXTRUDERS > 2
       || !E2_ENABLE_PIN
    #endif
    #if EXTRUDER > 1
       || !E1_ENABLE_PIN
    #endif
       || !E0_ENABLE_PIN) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }
    
    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...   
    {
        digitalWrite(CONTROLLERFAN_PIN, 0); 
        analogWrite(CONTROLLERFAN_PIN, 0); 
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED); 
    }
  }
}
#endif
*/
void manage_inactivity(void)
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
     //   disable_e2();
      }
    }
  }
  check_axes_activity();
}

void kill(void)
{
  CRITICAL_SECTION_START; // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
 // disable_e2();

//#if defined(PS_ON_PIN) && PS_ON_PIN > -1
//  pinMode(PS_ON_PIN,INPUT);
//#endif  
  SERIAL_ERROR_START;
  printf(MSG_ERR_KILLED);
//  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop(void)
{
  disable_heater();
  if(Stopped == false) {

    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    printf(MSG_ERR_STOPPED);
  }
}

bool IsStopped(void) 
{
	return Stopped; 
}
bool setTargetedHotend(int code)
{
  tmp_extruder = active_extruder;
  if(code_seen('T')) 
	{
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) 
		{
      SERIAL_ECHO_START;
      switch(code)
			{
        case 104:
          printf(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          printf(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          printf(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          printf(MSG_M218_INVALID_EXTRUDER);
          break;
      }
      printf("%d",tmp_extruder);
      return true;
    }
  }
  return false ;
}
