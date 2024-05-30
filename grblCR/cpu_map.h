/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h


  #ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.

    // Define serial port pins and interrupt vectors.
    #define SERIAL_RX     USART_RX_vect
    #define SERIAL_UDRE   USART_UDRE_vect

    // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
    #define STEP_DDR        DDRD
    #define STEP_PORT       PORTD
    #define X_STEP_BIT      2  // Uno Digital Pin 2
    #define Y_STEP_BIT      3  // Uno Digital Pin 3
    #define Z_STEP_BIT      4  // Uno Digital Pin 4
    #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

    // Define step direction output pins. NOTE: All direction pins must be on the same port.
    #define DIRECTION_DDR     DDRD
    #define DIRECTION_PORT    PORTD
    #define X_DIRECTION_BIT   5  // Uno Digital Pin 5
    #define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
    #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
    #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

    // Define stepper driver enable/disable output pin.
    #define STEPPERS_DISABLE_DDR    DDRB
    #define STEPPERS_DISABLE_PORT   PORTB
    #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
    #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

    // Define stepper X1 sleep/wake output pin
    #define STEPPERS_X1_SLEEP_DDR  DDRC
    #define STEPPERS_X1_SLEEP_PIN  PINC //1: awake, 0: sleeping
    #define STEPPERS_X1_SLEEP_PORT PORTC
    #define STEPPERS_X1_SLEEP_BIT  0 // Uno Pin A0
    #define STEPPERS_X1_SLEEP_MASK (1<<STEPPERS_X1_SLEEP_BIT)

    //Define Stepper Power Level (Uno pin A1)
    #define STEPPERS_POWER_DDR  DDRC
    #define STEPPERS_POWER_PIN  PINC
    #define STEPPERS_POWER_PORT PORTC
    #define STEPPERS_POWER_BIT  1 // Uno Pin A1
    #define STEPPERS_POWER_MASK (1<<STEPPER_POWER_BIT)

    // Define homing/hard limit switch input pins and limit interrupt vectors.
    // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
    #define LIMIT_DDR        DDRB
    #define LIMIT_PIN        PINB
    #define LIMIT_PORT       PORTB
    #define X_LIMIT_BIT      1  // Uno Digital Pin 9
    #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
    #define Z_LIMIT_BIT	     4  // Uno Digital Pin 12
    #define LIMIT_MASK     ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

    #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
    #define LIMIT_INT_vect   PCINT0_vect
    #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

    // Performs two separate tasks:
    //When spindle not rotating, detects X Table's second limit switch status during auto-level ('$L') // This limit switch doesn't cause interrupts
    //When spindle     rotating, used to detect spindle actualRPM status (along with SPINDLE_RPM_STATUS pin)
    #define LIMIT_X1_DDR     DDRC   
    #define LIMIT_X1_PIN     PINC
    #define LIMIT_X1_PORT    PORTC
    #define LIMIT_X1_BIT     2 // Uno Pin A2
    #define LIMIT_X1_MASK    (1<<LIMIT_X1_BIT)

    //Define spindle RPM status pin (from 32M1)
    #define SPINDLE_RPM_STATUS_DDR  DDRC
    #define SPINDLE_RPM_STATUS_PIN  PINC
    #define SPINDLE_RPM_STATUS_PORT PORTC
    #define SPINDLE_RPM_STATUS_BIT  4 //Uno Pin A4
    #define SPINDLE_RPM_STATUS_MASK (1<<SPINDLE_RPM_STATUS_BIT)
    
    // Define probe switch input pin.
    #define PROBE_DDR       DDRC
    #define PROBE_PIN       PINC
    #define PROBE_PORT      PORTC
    #define PROBE_BIT       5  // Uno Analog Pin 5
    #define PROBE_MASK      (1<<PROBE_BIT)

    // Define spindle enable pin
    #define SPINDLE_ENABLE_DDR    DDRB
    #define SPINDLE_ENABLE_PORT   PORTB
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11

    // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
    #define SPINDLE_PWM_DDR   DDRB
    #define SPINDLE_PWM_PORT  PORTB
    #define SPINDLE_PWM_BIT   3    // Uno Digital Pin 11
    #define SPINDLE_PWM_MASK  (1<<SPINDLE_PWM_BIT)

    // Define spindle direction pin
    #define SPINDLE_DIRECTION_DDR   DDRB
    #define SPINDLE_DIRECTION_PORT  PORTB
    #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
    #define SPINDLE_DIRECTION_MASK (1<<SPINDLE_DIRECTION_BIT)

    #define SPINDLE_HALL_DISABLE_DDR   DDRC
    #define SPINDLE_HALL_DISABLE_PIN   PINC
    #define SPINDLE_HALL_DISABLE_PORT  PORTC
    #define SPINDLE_HALL_DISABLE_BIT   3 // Uno Analog Pin 3
    #define SPINDLE_HALL_DISABLE_MASK  (1<<SPINDLE_HALL_DISABLE_BIT)

    // Variable spindle configuration below. Do not change unless you know what you are doing.
    // NOTE: Only used when variable spindle is enabled.
    #define SPINDLE_PWM_MAX_VALUE     255 // Don't change. 328p fast PWM mode fixes top value as 255.
    #ifndef SPINDLE_PWM_MIN_VALUE
      #define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
    #endif
    #define SPINDLE_PWM_OFF_VALUE     0
    #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
    #define SPINDLE_TCCRA_REGISTER    TCCR2A
    #define SPINDLE_TCCRB_REGISTER    TCCR2B
    #define SPINDLE_OCR_REGISTER      OCR2A
    #define SPINDLE_COMB_BIT          COM2A1

    // Prescaled, 8-bit Fast PWM mode.
    #define SPINDLE_TCCRA_INIT_MASK   ((1<<WGM20) | (1<<WGM21))  // Configures fast PWM mode.
    // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)               // Disable prescaler -> 62.5kHz
    #define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)               // 1/8 prescaler -> 7.8kHz (Used in v0.9) //enabled
    // #define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21) | (1<<CS20)) // 1/32 prescaler -> 1.96kHz
    //#define SPINDLE_TCCRB_INIT_MASK      (1<<CS22)               // 1/64 prescaler -> 0.98kHz

  #endif

#endif
