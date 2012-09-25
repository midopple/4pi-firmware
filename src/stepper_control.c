/*
 Stepper Control
 Load Data from Plannerbuffer 
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */
 
 
 
 
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


#include "com_interpreter.h"
#include "arc_func.h"
#include "planner.h"
#include "stepper_control.h"


//Motor opts
extern void motor_enaxis(unsigned char axis, unsigned char en);
extern void motor_setdir(unsigned char axis, unsigned char dir);
extern void motor_step(unsigned char axis);

const unsigned char INVERT_X_DIR = 0;
const unsigned char INVERT_Y_DIR = 0;
const unsigned char INVERT_Z_DIR = 1;
const unsigned char INVERT_E_DIR = 0;


// Stepper

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
//#define MultiU16X8toH16(intRes, charIn1, intIn2)


// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
// #define MultiU24X24toH16(intRes, longIn1, longIn2)

// Some useful constants

//#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
//#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)

#ifdef ENDSTOPS_ONLY_FOR_HOMING
  #define CHECK_ENDSTOPS  if(check_endstops)
#else
  #define CHECK_ENDSTOPS
#endif

volatile block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y, 
            counter_z,       
            counter_e;
static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef ADVANCE
  static long advance_rate, advance, final_advance = 0;
  static short old_advance = 0;
#endif
#ifdef ADVANCE
	static short e_steps;
#endif
static unsigned char busy = 0; // ture when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate; // needed for deceleration start point
static char step_loops;
static unsigned short OCR1A_nominal;

static volatile unsigned char endstop_x_hit=0;
static volatile unsigned char endstop_y_hit=0;
static volatile unsigned char endstop_z_hit=0;

unsigned char old_x_min_endstop=0;
unsigned char old_x_max_endstop=0;
unsigned char old_y_min_endstop=0;
unsigned char old_y_max_endstop=0;
unsigned char old_z_min_endstop=0;
unsigned char old_z_max_endstop=0;



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
//  The trapezoid is the shape of the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.


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
  
  if(step_rate < 40) step_rate = 40;
  step_rate -= 40; // Correct for minimal speed
  
  if(step_rate >= (8*256)) // higher step rate 
  { // higher step rate 
    //unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    //unsigned char tmp_step_rate = (step_rate & 0x00ff);
    //unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    //timer = (unsigned short)((tmp_step_rate * gain) >> 16);
    //timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else 
  { // lower step rates
    //unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
    //table_address += ((step_rate)>>1) & 0xfffc;
    //timer = (unsigned short)pgm_read_word_near(table_address);
    //timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; }//(20kHz this should never happen)
  return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
void trapezoid_generator_reset()
{
  #ifdef ADVANCE
    advance = current_block->initial_advance;
    final_advance = current_block->final_advance;
    // Do E steps + advance steps
    e_steps += ((advance >>8) - old_advance);
    old_advance = advance >>8;  
  #endif
  deceleration_time = 0;
  
  
  // step_rate to timer interval
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  //OCR1A = acceleration_time;
  //OCR1A_nominal = calc_timer(current_block->nominal_rate);
    
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
//ISR(TIMER1_COMPA_vect)
void stepper_timer(void)
{        
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
	  printf("get block\n\r");
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;
      #ifdef ADVANCE
		e_steps = 0;
      #endif
    } 
    else {
        //OCR1A=2000; // 1kHz.
    }    
  } 

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction and check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
	  motor_setdir(X_AXIS, INVERT_X_DIR);
      CHECK_ENDSTOPS
      {
        #if X_MIN_PIN > -1
          unsigned char x_min_endstop=0;	//read IO
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            if(!is_homing)
              endstop_x_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_x_hit=0;
          }
          old_x_min_endstop = x_min_endstop;
        #else
          endstop_x_hit=0;
        #endif
      }
    }
    else { // +direction 
      motor_setdir(X_AXIS, !INVERT_X_DIR);
      CHECK_ENDSTOPS 
      {
        #if X_MAX_PIN > -1
          unsigned char x_max_endstop=0;	//read IO
          if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)){
            if(!is_homing)
              endstop_x_hit=1;
            else    
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_x_hit=0;
          }
          old_x_max_endstop = x_max_endstop;
        #else
          endstop_x_hit=0;
        #endif
      }
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
	  motor_setdir(Y_AXIS, INVERT_Y_DIR);
      CHECK_ENDSTOPS
      {
        #if Y_MIN_PIN > -1
          unsigned char y_min_endstop=0;	//read IO
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            if(!is_homing)
              endstop_y_hit=1;
            else
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_y_hit=0;
          }
          old_y_min_endstop = y_min_endstop;
        #else
          endstop_y_hit=0;  
        #endif
      }
    }
    else { // +direction
      motor_setdir(Y_AXIS, !INVERT_Y_DIR);
      CHECK_ENDSTOPS
      {
        #if Y_MAX_PIN > -1
          unsigned char y_max_endstop=0;	//read IO
          if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)){
            if(!is_homing)
              endstop_y_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_y_hit=0;
          }
          old_y_max_endstop = y_max_endstop;
        #else
          endstop_y_hit=0;  
        #endif
      }
    }

    if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
      motor_setdir(Z_AXIS, INVERT_Z_DIR);
      CHECK_ENDSTOPS
      {
        #if Z_MIN_PIN > -1
          unsigned char z_min_endstop=0;	//read IO
          if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
            if(!is_homing)  
              endstop_z_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_z_hit=0;
          }
          old_z_min_endstop = z_min_endstop;
        #else
          endstop_z_hit=0;  
        #endif
      }
    }
    else { // +direction
      motor_setdir(Z_AXIS, !INVERT_Z_DIR);
      CHECK_ENDSTOPS
      {
        #if Z_MAX_PIN > -1
          unsigned char z_max_endstop=0;	//read IO
          if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
            if(!is_homing)
              endstop_z_hit=1;
            else  
              step_events_completed = current_block->step_event_count;
          }
          else
          {
            endstop_z_hit=0;
          }
          old_z_max_endstop = z_max_endstop;
        #else
          endstop_z_hit=0;  
        #endif
      }
    }

    #ifndef ADVANCE
      if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
        motor_setdir(E_AXIS, INVERT_E_DIR);
      }
      else { // +direction
        motor_setdir(E_AXIS, !INVERT_E_DIR);
      }
    #endif //!ADVANCE
    

    unsigned char cnt_c = 0;
    for(cnt_c=0; cnt_c < step_loops; cnt_c++) { // Take multiple steps per interrupt (For high speed moves) 
      
      #ifdef ADVANCE
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        counter_e -= current_block->step_event_count;
        if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
          e_steps--;
        }
        else {
          e_steps++;
        }
      }    
      #endif //ADVANCE


      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        if(!endstop_x_hit)
        {
          if(virtual_steps_x)
            virtual_steps_x--;
          else
            motor_step(X_AXIS);
        }
        else
          virtual_steps_x++;
          
        counter_x -= current_block->step_event_count;
        //WRITE(X_STEP_PIN, LOW);
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        if(!endstop_y_hit)
        {
          if(virtual_steps_y)
            virtual_steps_y--;
          else
            motor_step(Y_AXIS);
        }
        else
          virtual_steps_y++;
            
        counter_y -= current_block->step_event_count;
        //WRITE(Y_STEP_PIN, LOW);
      }

      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        if(!endstop_z_hit)
        {
          if(virtual_steps_z)
            virtual_steps_z--;
          else
            motor_step(Z_AXIS);
        }
        else
          virtual_steps_z++;
          
        counter_z -= current_block->step_event_count;
        //WRITE(Z_STEP_PIN, LOW);
      }

      #ifndef ADVANCE
        counter_e += current_block->steps_e;
        if (counter_e > 0) {
          motor_step(E_AXIS);
          counter_e -= current_block->step_event_count;
          //WRITE(E_STEP_PIN, LOW);
        }
      #endif //!ADVANCE

      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
      
    }
    // Calculare new timer value
    unsigned short timer;
    unsigned short step_rate;
    if (step_events_completed <= (unsigned long)current_block->accelerate_until) {
      
      acc_step_rate = (unsigned short)((acceleration_time * current_block->acceleration_rate) >> 24);
      acc_step_rate += current_block->initial_rate;
      
      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      //OCR1A = timer;
      acceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t cnt_c=0; cnt_c < step_loops; cnt_c++) {
          advance += advance_rate;
        }
        //if(advance > current_block->advance) advance = current_block->advance;
        // Do E steps + advance steps
        e_steps += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
        
      #endif
    } 
    else if (step_events_completed > (unsigned long)current_block->decelerate_after) {   
      step_rate = (unsigned short)((deceleration_time * current_block->acceleration_rate) >> 24);
      
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
      //OCR1A = timer;
      deceleration_time += timer;
      #ifdef ADVANCE
        for(int8_t cnt_c=0; cnt_c < step_loops; cnt_c++) {
          advance -= advance_rate;
        }
        if(advance < final_advance) advance = final_advance;
        // Do E steps + advance steps
        e_steps += ((advance >>8) - old_advance);
        old_advance = advance >>8;  
      #endif //ADVANCE
    }
    else {
      //OCR1A = OCR1A_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) {
      current_block = NULL;
      plan_discard_current_block();
    }   
  } 
}
