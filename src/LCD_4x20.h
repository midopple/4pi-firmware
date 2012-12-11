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

//Global functions
void lcd_init(void);
void LCD_printf(const char * format, ...);

//Local functions
void lcd_io_setup(void);
void lcd_data(unsigned char temp1);
void lcd_command(unsigned char temp1);
void lcd_enable(void);
void lcd_init(void);
void lcd_home(void);
void set_cursor(uint8_t x, uint8_t y);
void lcd_clear(void);