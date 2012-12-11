/*
 LCD Display Control (for 4x20 with HD44780)
  
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "parameters.h"
#include "init_configuration.h"
#include "LCD_4x20.h"
#include "util.h"

//--------------------------------------------
//DISPLAY VALUES
//--------------------------------------------
#define DISP_LINES		4
#define CHAR_PER_LINE	20

//--------------------------------------------
// LCD Commands
//--------------------------------------------
#define CMD_CLEAR_DISPLAY 	0x01
#define CMD_CURSOR_HOME   	0x02

#define CMD_FUCTION_SET		0x20
#define OPT_INTERFACE_8BIT	0x10
#define OPT_TWO_LINES		0x08
#define OPT_5x11_PIXEL		0x04

#define CMD_DISPLAY			0x08
#define OPT_DISP_ON			0x04
#define OPT_CURSOR_ON		0x02
#define OPT_BLINK_CURSOR	0x01

#define CMD_DISPLAY_SHIFT	0x10
#define OPT_SHIFT_DISP		0x08	
#define OPT_SHIFT_LEFT		0x04

#define CMD_ENTRY_MODE		0x04
#define OPT_CURSOR_RIGHT	0x02	
#define OPT_SHIFT_ON		0x01

#define CMD_DD_RAM_ADR		0x80


//-------------------------------------------- 
// define Pins for LCD
//--------------------------------------------
const Pin LCD_RS={1 <<  9, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin LCD_RW={1 <<  14, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin LCD_EN={1 <<  17, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};

const Pin LCD_DB4={1 <<  19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin LCD_DB5={1 <<  21, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin LCD_DB6={1 <<  22, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin LCD_DB7={1 <<  24, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};


//--------------------------------------------
//setup the IO's
//--------------------------------------------
void lcd_io_setup(void)
{
	Pin LCD_pins[]={LCD_EN,LCD_RS,LCD_RW,LCD_DB4,LCD_DB5,LCD_DB6,LCD_DB7};
	PIO_Configure(LCD_pins,7);
	
	PIO_Clear(&LCD_EN);
	PIO_Clear(&LCD_RS);
	PIO_Clear(&LCD_RW);
	
}

//--------------------------------------------
//send data to display
//--------------------------------------------
void lcd_data(unsigned char data_val)
{
   
	unsigned char cnt_c = 0;

	
	Pin LCD_data_pins[]={LCD_DB4,LCD_DB5,LCD_DB6,LCD_DB7};
	
	PIO_Set(&LCD_RS);

	for(cnt_c=0;cnt_c<4;cnt_c++)
	{
		if(data_val & (0x01 << (cnt_c+4)))
			PIO_Set(&LCD_data_pins[cnt_c]);
		else
			PIO_Clear(&LCD_data_pins[cnt_c]);
	}
	
	lcd_enable();
	
	
	for(cnt_c=0;cnt_c<4;cnt_c++)
	{
		if(data_val & (0x01 << cnt_c))
			PIO_Set(&LCD_data_pins[cnt_c]);
		else
			PIO_Clear(&LCD_data_pins[cnt_c]);
	}	
   
	lcd_enable();
	

}

//-------------------------------------------- 
// send command to display
//--------------------------------------------
void lcd_command(unsigned char cmd_val)
{
	
	unsigned char cnt_c = 0;
	
	Pin LCD_data_pins[]={LCD_DB4,LCD_DB5,LCD_DB6,LCD_DB7};
	
	PIO_Clear(&LCD_RS);
	
	for(cnt_c=0;cnt_c<4;cnt_c++)
	{
		if(cmd_val & (0x01 << (cnt_c+4)))
			PIO_Set(&LCD_data_pins[cnt_c]);
		else
			PIO_Clear(&LCD_data_pins[cnt_c]);
	}
	
	lcd_enable();
	
	
	for(cnt_c=0;cnt_c<4;cnt_c++)
	{
		if(cmd_val & (0x01 << cnt_c))
			PIO_Set(&LCD_data_pins[cnt_c]);
		else
			PIO_Clear(&LCD_data_pins[cnt_c]);
	}	
   
	lcd_enable();
	
 
}

//-------------------------------------------- 
//make enable signal
//--------------------------------------------
void lcd_enable(void)
{
    volatile unsigned int uDummy;
	
	PIO_Set(&LCD_EN);	
	for (uDummy=0; uDummy<200; ++uDummy);
	PIO_Clear(&LCD_EN);
	for (uDummy=0; uDummy<20; ++uDummy);
}

//-------------------------------------------- 
// Init Display
//--------------------------------------------
void lcd_init(void)
{
 	//toto: make value for char per line
 
	lcd_io_setup();
 
	delay_ms(15);
	
	PIO_Clear(&LCD_DB7);
	PIO_Clear(&LCD_DB6);
	PIO_Set(&LCD_DB5);
	PIO_Set(&LCD_DB4);
	
	PIO_Clear(&LCD_RS);
	lcd_enable();

	delay_ms(5);
	lcd_enable();

	delay_ms(2);
	lcd_enable();

	// 4 Bit Modus 
	PIO_Clear(&LCD_DB7);
	PIO_Clear(&LCD_DB6);
	PIO_Set(&LCD_DB5);
	PIO_Clear(&LCD_DB4);
	
	lcd_enable();

	#if DISP_LINES == 1
	// 4Bit / 1 Line / 5x8 points
	lcd_command(CMD_FUCTION_SET);
	#else
	// 4Bit / 2 Line / 5x8 points
	lcd_command(CMD_FUCTION_SET+OPT_TWO_LINES);
	#endif
	
	// Display on / Cursor off 
	lcd_command(CMD_DISPLAY+OPT_DISP_ON); 

	// move cursor right
	lcd_command(CMD_ENTRY_MODE+OPT_CURSOR_RIGHT);

	lcd_clear();


	
	//Test
	unsigned char help_uc = 1;
	LCD_printf("Hello 4pi %d",help_uc++);
	set_cursor(0,2);
	LCD_printf("Hello 4pi %d",help_uc++);
	set_cursor(0,3);
	LCD_printf("Hello 4pi %d",help_uc++);
	set_cursor(0,4);
	LCD_printf("Hello 4pi %d",help_uc++);	
	
}
 
//-------------------------------------------- 
// Send clear command
//--------------------------------------------
void lcd_clear(void)
{
	lcd_command(CMD_CLEAR_DISPLAY);
	delay_ms(3);
}
 
//-------------------------------------------- 
// move to home position
//--------------------------------------------
void lcd_home(void)
{
	lcd_command(CMD_CURSOR_HOME);
	delay_ms(3);
}

//--------------------------------------------
// move cursor to x,y pos
//--------------------------------------------
void set_cursor(unsigned char x, unsigned char y)
{
	if(x > CHAR_PER_LINE)
		x = CHAR_PER_LINE;
	
	switch (y) 
	{

		case 1: lcd_command(CMD_DD_RAM_ADR+0x00+x); break;    // 1. Zeile
		#if (DISP_LINES > 1)
		case 2: lcd_command(CMD_DD_RAM_ADR+0x40+x); break;    // 2. Zeile
		#endif
		#if (DISP_LINES > 2)
		case 3: lcd_command(CMD_DD_RAM_ADR+CHAR_PER_LINE+x); break;    // 3. Zeile
		#endif
		#if (DISP_LINES > 3)
		case 4: lcd_command(CMD_DD_RAM_ADR+0x40+CHAR_PER_LINE+x); break;    // 4. Zeile
		#endif
	}
}

//--------------------------------------------
// Print a String to Display
//--------------------------------------------
void LCD_printf(const char * format, ...)
{
	
	unsigned int str_len = 0;
	unsigned char cnt_uc = 0;
	char printBuffer[CHAR_PER_LINE+1];
	
	va_list args;
	va_start (args, format);
	str_len = vsnprintf(printBuffer,CHAR_PER_LINE+1,format, args);
	va_end (args);

	
	for(cnt_uc = 0;cnt_uc < str_len;cnt_uc++)
	{
		lcd_data(printBuffer[cnt_uc]);
	}
	
}
