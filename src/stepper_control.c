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
volatile unsigned char out_bits;        // The next stepping-bits to be output
volatile long 	counter_x,       // Counter variables for the bresenham line tracer
				counter_y, 
				counter_z,       
				counter_e;
volatile unsigned long step_events_completed; // The number of step events executed in the current block

#ifdef ADVANCE
	volatile long advance_rate, advance, final_advance = 0;
	volatile short old_advance = 0;
#endif
#ifdef ADVANCE
	volatile short e_steps;
#endif

volatile unsigned char busy = 0; // ture when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
volatile long acceleration_time, deceleration_time;
volatile unsigned short acc_step_rate; // needed for deceleration start point
volatile unsigned short OCR1A_nominal;

volatile volatile unsigned char endstop_x_hit=0;
volatile volatile unsigned char endstop_y_hit=0;
volatile volatile unsigned char endstop_z_hit=0;

volatile unsigned char old_x_min_endstop=0;
volatile unsigned char old_x_max_endstop=0;
volatile unsigned char old_y_min_endstop=0;
volatile unsigned char old_y_max_endstop=0;
volatile unsigned char old_z_min_endstop=0;
volatile unsigned char old_z_max_endstop=0;


//------------------------------------------------------------------------------
/// Interrupt handler for TC0 interrupt --> Stepper.
//------------------------------------------------------------------------------
void TC0_IrqHandler(void)
{
    volatile unsigned int dummy;
    // Clear status bit to acknowledge interrupt
    
    dummy = AT91C_BASE_TC0->TC_SR;
    if(dummy & AT91C_TC_CPCS)
	{
		//Function need 3 �s up to 5�s
		stepper_timer();
		motor_unstep();
    }
    if(dummy & AT91C_TC_CPBS){
        //This line is not called ???
		motor_unstep();
    }
 }

void ConfigureTc(void)
{

    // Enable peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;
    unsigned int freq=1000; 
    //TC_FindMckDivisor(freq, BOARD_MCK, &div, &tcclks);
    TC_Configure(AT91C_BASE_TC0, 3 | AT91C_TC_CPCTRG);
    AT91C_BASE_TC0->TC_RB = 3; //6*((BOARD_MCK / div)/1000000); //6 uSec per step pulse 
    AT91C_BASE_TC0->TC_RC = (BOARD_MCK / 128) / freq; // timerFreq / desiredFreq

    // Configure and enable interrupt on RC compare
    IRQ_ConfigureIT(AT91C_ID_TC0, 0, TC0_IrqHandler);
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS|AT91C_TC_CPBS;
    IRQ_EnableIT(AT91C_ID_TC0);

    // Start the counter if LED is enabled.
    TC_Start(AT91C_BASE_TC0);
    
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
//  The trapezoid is the shape of the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.


unsigned short calc_timer(unsigned short step_rate)
{
	unsigned short timer;
	if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;


	if(step_rate < 50) step_rate = 50;
	//step_rate -= 49; // Correct for minimal speed

	timer = (unsigned short)((BOARD_MCK / 128) / step_rate);

	if(timer < 10) { timer = 10; }//(40kHz this should never happen)
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
	AT91C_BASE_TC0->TC_RC = acceleration_time;
	OCR1A_nominal = calc_timer(current_block->nominal_rate);

}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
//ISR(TIMER1_COMPA_vect)
void stepper_timer(void)
{        
	// If there is no current block, attempt to pop one from the buffer
	if (current_block == NULL)
	{
		// Anything in the buffer?
		current_block = plan_get_current_block();
		if (current_block != NULL)
		{
			//printf("get block\n\r");
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
		else
		{
			AT91C_BASE_TC0->TC_RC=500; // ~1kHz.
		}    
	} 

	if (current_block != NULL)
	{
		// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
		out_bits = current_block->direction_bits;

		// Set direction and check limit switches
		if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
			motor_setdir(X_AXIS, INVERT_X_DIR);
			CHECK_ENDSTOPS
			{
				#if X_MIN_PIN > -1
				unsigned char x_min_endstop=0;	//read IO
				if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) 
				{
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
				if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0))
				{
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
				if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0))
				{
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
				if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0))
				{
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
				if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0))
				{
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
				if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0))
				{
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
		  

		// Calculare new timer value
		unsigned short timer;
		unsigned short step_rate;
		if (step_events_completed <= (unsigned long)current_block->accelerate_until)
		{
			acc_step_rate = (unsigned short)(((long long)acceleration_time * (long long)current_block->acceleration_rate) >> 24);
			acc_step_rate += current_block->initial_rate;

			// upper limit
			if(acc_step_rate > current_block->nominal_rate)
				acc_step_rate = current_block->nominal_rate;

			// step_rate to timer interval
			timer = calc_timer(acc_step_rate);
			AT91C_BASE_TC0->TC_RC = timer;
			acceleration_time += timer;
			#ifdef ADVANCE
			//if(advance > current_block->advance) advance = current_block->advance;
			// Do E steps + advance steps
			e_steps += ((advance >>8) - old_advance);
			old_advance = advance >>8;  
			#endif
		} 
		else if (step_events_completed > (unsigned long)current_block->decelerate_after)
		{   
			step_rate = (unsigned short)(((long long)deceleration_time * (long long)current_block->acceleration_rate) >> 24);

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
			AT91C_BASE_TC0->TC_RC = timer;
			deceleration_time += timer;
			#ifdef ADVANCE
			if(advance < final_advance) advance = final_advance;
			// Do E steps + advance steps
			e_steps += ((advance >>8) - old_advance);
			old_advance = advance >>8;  
			#endif //ADVANCE
		}
		else 
		{
			AT91C_BASE_TC0->TC_RC = OCR1A_nominal;
		}

		// If current block is finished, reset pointer 
		if (step_events_completed >= current_block->step_event_count) 
		{
			current_block = NULL;
			plan_discard_current_block();
		}   
	} 
}
