/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "ESCSpindleControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"
#include "utils.h"

#include "libs/Pin.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"
#include "us_ticker_api.h"

#define spindle_checksum                    CHECKSUM("spindle")
#define spindle_pwm_pin_checksum            CHECKSUM("pwm_pin")        //has to be a hw pwm pin
#define spindle_pwm_period_checksum         CHECKSUM("pwm_period_ms")  //default: 20ms = 50Hz
#define spindle_control_smoothing_checksum  CHECKSUM("control_smoothing") //low pass filter time constant in seconds
// scaling rpm to pwm: pwm = scale_a * rpm + scale_b
#define spindle_pwm_scale_a_checksum        CHECKSUM("pwm_scale_a")    
#define spindle_pwm_scale_b_checksum        CHECKSUM("pwm_scale_b")
// initial rpm
#define spindle_rpm_default_checksum        CHECKSUM("rpm_default")
// rpm ramp (increment in rpm per second)
#define spindle_rpm_ramp_checksum           CHECKSUM("rpm_ramp")


// slow ticker in Hz
#define UPDATE_FREQ 10

ESCSpindleControl::ESCSpindleControl()
{
    THEKERNEL->streams->printf("ESC Spindle: created\r\n");
}

void ESCSpindleControl::on_module_loaded()
{
    current_rpm = 0;
    current_pwm_value = 0;

    manual_pwm_control_active = false;
    spindle_on = false;
    waiting = false;
    
    pwm_scale_a = THEKERNEL->config->value(spindle_checksum, spindle_pwm_scale_a_checksum)->by_default(1.0f)->as_number();
    pwm_scale_b = THEKERNEL->config->value(spindle_checksum, spindle_pwm_scale_b_checksum)->by_default(1.0f)->as_number();

    target_rpm = THEKERNEL->config->value(spindle_checksum, spindle_rpm_default_checksum)->by_default(5000.0f)->as_number();
    
    // ramp time (as increment rpm per second)
    ramp_rpm   = THEKERNEL->config->value(spindle_checksum, spindle_rpm_ramp_checksum)->by_default(100.0f)->as_number();
    // convert to increments per loop
    ramp_rpm   = ramp_rpm / UPDATE_FREQ;

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, spindle_pwm_pin_checksum)->by_default("nc")->as_string());
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    
    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("ESC Spindle: spindle PWM pin must be P2.0-2.5 or other PWM pin\r\n");
        delete this;
        return;
    }

    pwm_period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->by_default(20)->as_int();
    pwm_pin->period_us(pwm_period * 1000);
    pwm_pin->write(output_inverted ? 1 : 0);
    
    this->register_for_event(ON_SECOND_TICK);

    set_pwm(0);

    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &ESCSpindleControl::on_update_speed);
    THEKERNEL->streams->printf("ESC Spindle: module loaded\r\n");
}


void ESCSpindleControl::on_second_tick(void *argument)
{
    // If waiting for a speed to be reached, display current speed
    if (waiting){
        report_speed();
    }
    //THEKERNEL->streams->printf("ESC Spindle: tick!\n");
}

uint32_t ESCSpindleControl::on_update_speed(uint32_t dummy)
{
    float target;

    if (manual_pwm_control_active){
        return 0;
    }
    
    if (spindle_on) {
        target = target_rpm;
    }else{
        target = 0.0;
    }

    // do ramp up/down:
    float new_rpm = current_rpm;
    
    if (new_rpm == target){
	// do nothing
    }else if (new_rpm < target){
        // increment rpm!
        new_rpm += ramp_rpm;
        if (new_rpm > target){
            new_rpm = target;
        }
    } else {
        // decrement rpm!
        new_rpm -= ramp_rpm;
        if (new_rpm < target){
            new_rpm = target;
        }
    }
    
    // convert to pwm signal:
    float pwm = pwm_scale_a * new_rpm + pwm_scale_b;
 
    if (new_rpm == 0.0){
	    pwm = 0;
    }

    // update pwm output
    set_pwm(pwm);
    
    // store old value
    current_rpm = new_rpm;

    return 0;
}

void ESCSpindleControl::set_pwm(float pwm){
    // limit to valid pwm values:
    // 5-10% of 20ms period -> 1ms -2ms = servo signal
    // rescale to percentage of pwm
    current_pwm_value = confine(pwm, 5.0f, 10.0f) / 100.0;
    
    //THEKERNEL->streams->printf("ESC Spindle: pwm = %f\r\n", current_pwm_value);
    
    // set to output:
    if (output_inverted)
        pwm_pin->write(1.0f - current_pwm_value);
    else
        pwm_pin->write(current_pwm_value);
    
}

void ESCSpindleControl::turn_on() {
    spindle_on = true;
}

void ESCSpindleControl::turn_off() {
    spindle_on = false;
    set_speed(0);
}


void ESCSpindleControl::set_speed(int rpm) {
    manual_pwm_control_active = false;
    target_rpm = rpm;
    
    // we need to sleep until we reach the target speed
    this->waiting = true; 
    THEKERNEL->streams->printf("ESC Spindle: waiting for spindle to reach speed...\r\n");
    
    float tdiff;
    do {
        THEKERNEL->call_event(ON_IDLE, this);
        
        // check if ON_HALT was called (usually by kill button)
        if(THEKERNEL->is_halted()) {
            THEKERNEL->streams->printf("ESC Spindle: wait aborted by kill\r\n");
            break;
        }
        
        // calc speed diff
        tdiff =  fabs(current_rpm - target_rpm);
    } while (tdiff >= 200);
    
    this->waiting = false;
    
    report_speed();
    THEKERNEL->streams->printf("ESC Spindle: speed reached!\r\n");
}


void ESCSpindleControl::report_speed() {
    THEKERNEL->streams->printf("[S%f]\r\n", current_rpm);
    
    THEKERNEL->streams->printf("ESC Spindle: target speed %5f, current speed %5f [PWM = %5.6f]\r\n", 
                               target_rpm, current_rpm, current_pwm_value);
}

// misuse pid settings to configure the scaling
void ESCSpindleControl::set_p_term(float p) {
    pwm_scale_a = p;
}


void ESCSpindleControl::set_i_term(float i) {
    pwm_scale_b = i;
}


// use this for debugging and initial set up to find pwm_scale_*
void ESCSpindleControl::set_d_term(float d) {
    // activate manual speed control
    if (d > 0.0){
        manual_pwm_control_active = true;
    }else{
        manual_pwm_control_active = false;
    }
    
    //set pwm value. note: make sure to pass a valid value (5.0... 10.0)
    set_pwm(d);
    THEKERNEL->streams->printf("ESC Spindle: pwm set to %f\r\n", d);
}


void ESCSpindleControl::report_settings() {
    THEKERNEL->streams->printf("ESC Spindle: pwm(f) = %0.6f * f + %0.6f [current pwm period: %0.2f, pwm=%5.6f]\r\n",
                               pwm_scale_a, pwm_scale_b, pwm_period, current_pwm_value);
}

