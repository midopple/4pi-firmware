/*
 G-Code Interpreter
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */


#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <systick/systick.h>
#include <utility/trace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "serial.h"
#include "samadc.h"
#include "com_interpreter.h"
#include "heaters.h"
#include "planner.h"


extern void motor_enaxis(unsigned char axis, unsigned char en);


volatile char uart_in_buffer[256];
volatile unsigned char uart_wr_pointer = 0;
volatile unsigned char uart_rd_pointer = 0;


char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
unsigned char fromsd[BUFSIZE];


volatile unsigned char bufindr = 0;
volatile unsigned char bufindw = 0;
volatile unsigned char buflen = 0;
unsigned char serial_char;
volatile int serial_count = 0;
unsigned char comment_mode = 0;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
long gcode_N, gcode_LastN;

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = 0;

unsigned char relative_mode = 0;
volatile int feedmultiply=100; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
int saved_feedmultiply = 0;
volatile char feedmultiplychanged=0;
volatile int extrudemultiply=100; //100->1 200->2

unsigned char active_extruder = 0;		//0 --> Exteruder 1 / 1 --> Extruder 2
unsigned char tmp_extruder = 0;

extern volatile unsigned long timestamp;

//extern int bed_temp_celsius;


void usb_characterhandler(unsigned char c){ 
    //every time the USB receives a new character, this function is called
	uart_in_buffer[uart_wr_pointer++] = c;
	if(uart_wr_pointer >= UART_BUFFER_SIZE)
		uart_wr_pointer = 0;
}


unsigned char get_byte_from_UART(unsigned char *zeichen)
{
	if(uart_rd_pointer == uart_wr_pointer)
		return(0);
		
	*zeichen = uart_in_buffer[uart_rd_pointer++];
	if(uart_rd_pointer >= UART_BUFFER_SIZE)
		uart_rd_pointer = 0;
		
	return(1);	
}


void ClearToSend()
{
	previous_millis_cmd = timestamp;
	usb_printf("ok\r\n");
}

void FlushSerialRequestResend()
{
	uart_rd_pointer = uart_wr_pointer;
	usb_printf("Resend:%u ok\r\n",gcode_LastN + 1);
}

void get_command() 
{ 
  while( get_byte_from_UART(&serial_char) != 0 && buflen < BUFSIZE)
  {
    if(serial_char == '\n' || serial_char == '\r' || (serial_char == ':' && comment_mode == 0) || serial_count >= (MAX_CMD_SIZE - 1) ) 
    {
      if(!serial_count) { //if empty line
        comment_mode = 0; // for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string

        fromsd[bufindw] = 0;
        if(strstr(cmdbuffer[bufindw], "N") != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) )
          {
            usb_printf("Serial Error: Line Number is not Last Line Number+1, Last Line:%u",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
    
          if(strstr(cmdbuffer[bufindw], "*") != NULL)
          {
            unsigned char checksum = 0;
            unsigned char count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');
  
            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
            {
              usb_printf("Error: checksum mismatch, Last Line:%u",gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else 
          {
            usb_printf("Error: No Checksum with line number, Last Line:%u",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
    
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strstr(cmdbuffer[bufindw], "*") != NULL))
          {
            usb_printf("Error: No Line Number with checksum, Last Line:%u",gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        
				if((strstr(cmdbuffer[bufindw], "G") != NULL))
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
          {
            case 0:
            case 1:
            #ifdef USE_ARC_FUNCTION
            case 2:  //G2
            case 3:  //G3 arc func
            #endif
              #ifdef SDSUPPORT
              if(savetosd)
                break;
              #endif
              usb_printf("ok\r\n");
            break;
            
            default:
            break;
          }
        }
        //Removed modulo (%) operator, which uses an expensive divide and multiplication
        //bufindw = (bufindw + 1) % BUFSIZE;
        bufindw++;
        if(bufindw == BUFSIZE) bufindw = 0;
        buflen += 1;

      comment_mode = 0; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = 1;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

}

float code_value() { return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); }
long code_value_long() { return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); }
unsigned char code_seen_str(char code_string[]) { return (strstr(cmdbuffer[bufindr], code_string) != NULL); }  //Return True if the string was found

unsigned char code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

//------------------------------------------------
// CHECK COMMAND AND CONVERT VALUES
//------------------------------------------------
void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
  unsigned char cnt_c = 0;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        get_coordinates(); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = timestamp;
        return;
        //break;
      case 2: // G2  - CW ARC
        get_arc_coordinates();
        prepare_arc_move(1);
        previous_millis_cmd = timestamp;
        return;
      case 3: // G3  - CCW ARC
        get_arc_coordinates();
        prepare_arc_move(0);
        previous_millis_cmd = timestamp;
        return;  
      case 4: // G4 dwell
        break;
      case 28: //G28 Home all Axis one at a time
        break;
      case 90: // G90
		relative_mode = 0;
        break;
      case 91: // G91
		relative_mode = 1;
        break;
      case 92: // G92
        break;
      default:
        usb_printf("Unknown G-COM: %s \r\n",cmdbuffer[bufindr]);
      break;
    }
  }

  else if(code_seen('M'))
  {
    
    switch( (int)code_value() ) 
    {

      case 42: //M42 -Change pin status via gcode
        if (code_seen('S'))
        {

        }
        break;
      case 104: // M104
		if (code_seen('S'))
		{
			if(tmp_extruder < MAX_EXTRUDER)
				heaters[tmp_extruder].target_temp = code_value();
		}
        break;
      case 140: // M140 set bed temp
		if (code_seen('S')) bed_heater.target_temp = code_value();
        break;
      case 105: // M105
		  	if(tmp_extruder < MAX_EXTRUDER)
				usb_printf("ok T:%u @%u B:%u",heaters[tmp_extruder].akt_temp,heaters[tmp_extruder].pwm,bed_heater.akt_temp);
			else
				usb_printf("ok T:%u @%u B:%u",heaters[0].akt_temp,heaters[0].pwm,bed_heater.akt_temp);

        return;
        //break;
      case 109: // M109 - Wait for extruder heater to reach target.

		break;
		
      case 190: // M190 - Wait for bed heater to reach target temperature.
		break;

      case 106: //M106 Fan On

        break;
		
      case 107: //M107 Fan Off

        break;
      case 82:
        axis_relative_modes[3] = 0;
        break;
      case 83:
        axis_relative_modes[3] = 1;
        break;
      case 84:
        st_synchronize(); // wait for all movements to finish
        if(code_seen('S'))
        {
          stepper_inactive_time = code_value() * 1000; 
        }
        else if(code_seen('T'))
        {
          enable_x(); 
          enable_y(); 
          enable_z(); 
          enable_e(); 
        }
        else
        { 
          disable_x(); 
          disable_y(); 
          disable_z(); 
          disable_e(); 
        }
        break;
      case 85: // M85
        code_seen('S');
        max_inactive_time = code_value() * 1000; 
        break;
	  case 92: // M92
        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c])) 
          {
            axis_steps_per_unit[cnt_c] = code_value();
            axis_steps_per_sqr_second[cnt_c] = max_acceleration_units_per_sq_second[cnt_c] * axis_steps_per_unit[cnt_c];
          }
        }
        break;
      case 93: // M93 show current axis steps.
		//usb_printf("ok X:%g Y:%g Z:%g E:%g",axis_steps_per_unit[0],axis_steps_per_unit[1],axis_steps_per_unit[2],axis_steps_per_unit[3]);
		printf("ok X:%d Y:%d Z:%d E:%d",(int)axis_steps_per_unit[0],(int)axis_steps_per_unit[1],(int)axis_steps_per_unit[2],(int)axis_steps_per_unit[3]);
        break;
	  case 114: // M114 Display current position
		usb_printf("X:%d Y:%d Z:%d E:%d",(int)current_position[0],(int)current_position[1],(int)current_position[2],(int)current_position[3]);
        break;
      case 115: // M115
        usb_printf("FIRMWARE_NAME: Sprinter 4pi PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa EXTRUDER_COUNT:1\r\n");
        break;
	  case 119: // M119 show endstop state
		break;
	  case 201: // M201  Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)

        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c]))
          {
            max_acceleration_units_per_sq_second[cnt_c] = code_value();
            axis_steps_per_sqr_second[cnt_c] = code_value() * axis_steps_per_unit[cnt_c];
          }
        }
        break;
      case 202: // M202 max feedrate mm/sec
        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c])) max_feedrate[cnt_c] = code_value();
        }
      break;
      case 203: // M203 Temperature monitor
          //if(code_seen('S')) manage_monitor = code_value();
          //if(manage_monitor==100) manage_monitor=1; // Set 100 to heated bed
      break;
      case 204: // M204 acceleration S normal moves T filmanent only moves
          if(code_seen('S')) move_acceleration = code_value() ;
          if(code_seen('T')) retract_acceleration = code_value() ;
      break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E= max E jerk
        if(code_seen('S')) minimumfeedrate = code_value();
        if(code_seen('T')) mintravelfeedrate = code_value();
      //if(code_seen('B')) minsegmenttime = code_value() ;
        if(code_seen('X')) max_xy_jerk = code_value() ;
        if(code_seen('Z')) max_z_jerk = code_value() ;
        if(code_seen('E')) max_e_jerk = code_value() ;
      break;
      case 206: // M206 additional homing offset
        if(code_seen('D'))
        {
          usb_printf("Addhome X:%g Y:%g Z:%g",add_homing[0],add_homing[1],add_homing[2]);
        }

        for(cnt_c=0; cnt_c < 3; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c])) add_homing[cnt_c] = code_value();
        }
      break;  	
	  case 220: // M220 S<factor in percent>- set speed factor override percentage
      {
        if(code_seen('S')) 
        {
          feedmultiply = code_value() ;
          feedmultiply = constrain(feedmultiply, 20, 200);
          feedmultiplychanged=1;
        }
      }
      break;
      case 221: // M221 S<factor in percent>- set extrude factor override percentage
      {
        if(code_seen('S')) 
        {
          extrudemultiply = code_value() ;
          extrudemultiply = constrain(extrudemultiply, 40, 200);
        }
      }
      break;
      case 301: // M301
      {
        if(tmp_extruder < MAX_EXTRUDER)
		{
			if(code_seen('P')) heaters[tmp_extruder].PID_Kp = code_value();
			if(code_seen('I')) heaters[tmp_extruder].PID_I = code_value();
			if(code_seen('D')) heaters[tmp_extruder].PID_Kd = code_value();
			heaters[tmp_extruder].temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (int)heaters[tmp_extruder].PID_I;
			heaters[tmp_extruder].temp_iState_min = heaters[tmp_extruder].temp_iState_max * (-1);
		}
      }
      break;
      case 303: // M303 PID autotune
      {
        //float help_temp = 150.0;
        //if (code_seen('S')) help_temp=code_value();
        //PID_autotune(help_temp);
      }
      break;
      case 400: // M400 finish all moves
      {
      	st_synchronize();	
      }
      break;	  

      default:
		usb_printf("Unknown M-COM: %s \r\n",cmdbuffer[bufindr]);
      break;

    }
    
  }
  else if(code_seen('T')) 
  {
    tmp_extruder = code_value();
    if(tmp_extruder >= MAX_EXTRUDER) 
	{
		//No more extruder
		usb_printf("Only 2 Extruder possible\r\n");
    }
    else 
	{
		active_extruder = tmp_extruder;
    }
  }
  else{
  
      usb_printf("Unknown command: %s \r\n",cmdbuffer[bufindr]);
  }
  
  ClearToSend();
      
}