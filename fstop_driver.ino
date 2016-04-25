/*
    Copyright (C) 2011-2015 William Brodie-Tyrrell
    william@brodie-tyrrell.org
  
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of   
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

/*
 * This sketch implements an f/stop enlarger timer; it requires 
 * an ATmega328-based Arduino; it will not fit in a -168.
 */

#ifndef _FSTOP_DRIVER
#define _FSTOP_DRIVER

// set this if using the new board layout, otherwise comment out for Rev.A
#define REVISION_B 1

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <RotaryEncoder.h>
#include <FreqMeasure.h>
#include <FstopTimer.h>

/**
 * Set this according to the vagaries of your rotary encoder & keypad cable
 */
#define ROTARY_REVERSE 1
#define ROTARY_DBLSTEP 1
#define KEYPAD_REVERSE 1

#if REVISION_B
/**
 * IO pin assignments for PCB Rev.B
 */

#define EXPOSE 13      // high=on
#define EXPOSEBTN A4   // low=depressed (internal pullup)
#define BEEP 5        // requires PWM
#define BACKLIGHT 6   // requires PWM; high=on

// keypad is 4x4: 3x3 numeric upper left with *0# below that and A-D down the right
// for physical keypad pinout, consider LHS (viewed from front) as pin 1
// 1-4 = rows starting at top
// 5-8 = columns starting at left

// output pins shared with LCD via resistors
#define SCANCOL0 9    // left column 147*
#define SCANCOL1 10
#define SCANCOL2 11
#define SCANCOL3 12    // right column ABCD

#define SCANROW0 A3    // top row 123A
#define SCANROW1 A2
#define SCANROW2 A1
#define SCANROW3 A0    // bottom row *0#D

// pins for HD44780 in 4-bit mode; RW grounded.
#define LCDD7 12
#define LCDD6 11
#define LCDD5 10
#define LCDD4 9
#define LCDEN 4
#define LCDRS 7
 
// light-sensor pins
#define TSLQ 5      // actually wired to both D5 and D8, to measure freq or period
#define TSLCK A5    // clock line for 74164/TSL230 control bits
#define TSLD 9      // data line for 74164/TSL230 control bits

#else
/**
 * IO pin assignments for PCB Rev.A
 */

#define EXPOSE 13      // high=on
#define EXPOSEBTN 12   // low=depressed (internal pullup)
#define BEEP 11        // requires PWM
#define BACKLIGHT 10   // requires PWM; high=on

// keypad is 4x4: 3x3 numeric upper left with *0# below that and A-D down the right
// for physical keypad pinout, consider LHS (viewed from front) as pin 1
// 1-4 = rows starting at top
// 5-8 = columns starting at left

// output pins shared with LCD via resistors
#define SCANCOL0 4    // left column 147*
#define SCANCOL1 5
#define SCANCOL2 6
#define SCANCOL3 7     // right column ABCD

#define SCANROW0 A5    // top row 123A
#define SCANROW1 A4
#define SCANROW2 A3
#define SCANROW3 A2    // bottom row *0#D

// pins for HD44780 in 4-bit mode; RW grounded.
#define LCDD7 7
#define LCDD6 6
#define LCDD5 5
#define LCDD4 4
#define LCDEN 9
#define LCDRS 8

#endif // pinout/revision

// 2 and 3 are taken by the rotary encoder - fixed assignment as they
// require interrupts

/**
 * Instances of static interface objects
 */
LiquidCrystal disp(LCDRS, LCDEN, LCDD4, LCDD5, LCDD6, LCDD7);
SMSKeypad keys(SCANCOL0, SCANCOL1, SCANCOL2, SCANCOL3, SCANROW0, SCANROW1, SCANROW2, SCANROW3, KEYPAD_REVERSE);
ButtonDebounce expbtn(EXPOSEBTN);
RotaryEncoder rotary(ROTARY_REVERSE, ROTARY_DBLSTEP);
// all the guts are in this object
FstopTimer fst(disp, keys, rotary, expbtn, EXPOSE, BEEP, BACKLIGHT);

/**
 * Arduino boot/init function
 */
void setup()
{
   // init raw IO for LCD
   pinMode(LCDRS, OUTPUT);
   pinMode(LCDEN, OUTPUT);
   pinMode(LCDD4, OUTPUT);
   pinMode(LCDD5, OUTPUT);
   pinMode(LCDD6, OUTPUT);
   pinMode(LCDD7, OUTPUT);
   
   disp.begin(16, 2);
   keys.begin();
   rotary.begin();
   fst.begin();
}

/**
 * Arduino main-program loop.
 * Processes the current state, looks for transitions to other states.
 */
void loop()
{
  fst.poll();
}

#endif
