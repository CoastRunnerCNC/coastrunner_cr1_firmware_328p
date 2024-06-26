/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "grbl.h"

static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.

void spindle_init()
{
  // Configure variable spindle PWM and enable pin, if required. On the Uno, PWM and enable are
  // combined unless configured otherwise.
  SPINDLE_PWM_DDR |= SPINDLE_PWM_MASK; // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  SPINDLE_DIRECTION_DDR |= SPINDLE_DIRECTION_MASK; // Configure spindle direction pin as output

  SPINDLE_HALL_DISABLE_PORT &= ~(SPINDLE_HALL_DISABLE_MASK); //Hall_C PORT value (never changes)

  SPINDLE_RPM_STATUS_DDR &= ~(SPINDLE_RPM_STATUS_MASK); //set as input

  spindle_stop();
}


uint8_t spindle_get_state()
{
  if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
    if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
    else { return(SPINDLE_STATE_CW); }
  }
  return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
    //create interrupt on 32M1
    SPINDLE_DIRECTION_PORT |=  (1<<SPINDLE_DIRECTION_BIT); //set direction pin high
    SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT); //set direction pin low

    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.

    //When the spindle is stopped, we also need to pull Hall_C sensor low.
    //This ensures that at lower PWM values - when PWM(328p)->LPF->ADC(VFD) result is close to zero - 
    //the spindle does not randomly start and stop.  This lets us safely use lower PWM values.
    //Pulling Hall_C low causes invalid LUT state (b000) when Hall_A & Hall_B are also both low.
    //When (A XOR B), Hall_C still corrupts data because VFD chooses wrong quadrant (good).
    //End result is pulling Hall_C low keeps spindle from spinning when noise exists on PWM pin.
    SPINDLE_HALL_DISABLE_DDR |= SPINDLE_HALL_DISABLE_MASK;
}

// Sets spindle speed PWM output. Called by spindle_set_state()
// and stepper ISR. Keep routine small and efficient.
void spindle_set_speed(uint8_t pwm_value)
{
  SPINDLE_HALL_DISABLE_DDR &= ~(SPINDLE_HALL_DISABLE_MASK); //set Hall_C sensor pin high impedance.
  SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
      SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
    } else {
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
    }
}

// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
{
  uint8_t pwm_value;
  rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
  // Calculate PWM register value based on rpm max/min settings and programmed rpm.
  if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
    // No PWM range possible. Set simple on/off spindle control pin state.
    sys.spindle_speed = settings.rpm_max;
    pwm_value = SPINDLE_PWM_MAX_VALUE;
  } else if (rpm <= settings.rpm_min) {
    if (rpm == 0.0) { // S0 disables spindle
      sys.spindle_speed = 0.0;
      pwm_value = SPINDLE_PWM_OFF_VALUE;
    } else { // Set minimum PWM output
      sys.spindle_speed = settings.rpm_min;
      pwm_value = SPINDLE_PWM_MIN_VALUE;
    }
  } else { 
    // Compute intermediate PWM value with linear spindle speed model.
    // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
    sys.spindle_speed = rpm;
    pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
  }
  return(pwm_value);
}
    

// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), g-code program end, sleep, and spindle stop override.
void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // Block during abort.

  //create interrupt on 32M1
  SPINDLE_DIRECTION_PORT |=  (1<<SPINDLE_DIRECTION_BIT); //set direction pin high
  SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT); //set direction pin low

  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
    sys.spindle_speed = 0.0;
    spindle_stop();
  } 
  else
  { 
    if (state == SPINDLE_ENABLE_CW) {SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);} 
    else {SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);}

    spindle_set_speed(spindle_compute_pwm_value(rpm));
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
void spindle_sync(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state,rpm);
}


// Determine spindle actualRPM status
// The spindle CPU (32M1) sets two pins to indicate actual RPM status:
  //'0k': Spindle actualRPM within 0000:0999 of goalRPM //unoPinA2_low  //unoPinA4_low
  //'1k': Spindle actualRPM within 1000:1999 of goalRPM //unoPinA2_low  //unoPinA4_high
  //'2k': Spindle actualRPM within 2000:2999 of goalRPM //unoPinA2_high //unoPinA4_low
  //'3k': Spindle actualRPM beyond 3000      of goalRPM //unoPinA2_high //unoPinA4_high
uint8_t spindle_get_actual_RPM_status(void)
{
  uint8_t actualRPM_pinStates = 0;

  if(LIMIT_X1_PIN & LIMIT_X1_MASK)                     { actualRPM_pinStates  = (1<<SPINDLE_ACTUAL_RPM_BIT1); } //set bit1 if pin A2 is high
  if(SPINDLE_RPM_STATUS_PIN & SPINDLE_RPM_STATUS_MASK) { actualRPM_pinStates |= (1<<SPINDLE_ACTUAL_RPM_BIT0); } //set bit0 if pin A4 is high

  return actualRPM_pinStates;
}