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
 
 
 
 #define MAX_STEP_FREQUENCY 40000
 
#define X_MIN_PIN           1
#define X_MAX_PIN          -1

#define Y_MIN_PIN           2
#define Y_MAX_PIN          -1

#define Z_MIN_PIN           3
#define Z_MAX_PIN          -1
 
 
 
 void stepper_timer(void);