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
#define spindle_default_rpm_checksum        CHECKSUM("default_rpm")

#define UPDATE_FREQ 10

PWMSpindleControl::PWMSpindleControl()
{
}

void PWMSpindleControl::on_module_loaded()
{
    current_rpm = 0;
    current_pwm_value = 0;
    
    spindle_on = false;
    waiting = false;
    
    pwm_scale_a = THEKERNEL->config->value(spindle_checksum, spindle_pwm_scale_a_checksum)->by_default(1.0f)->as_number();
    pwm_scale_b = THEKERNEL->config->value(spindle_checksum, spindle_pwm_scale_b_checksum)->by_default(1.0f)->as_number();

    target_rpm = THEKERNEL->config->value(spindle_checksum, spindle_default_rpm_checksum)->by_default(5000.0f)->as_number();

    // Smoothing value is low pass filter time constant in seconds.
    float smoothing_time = THEKERNEL->config->value(spindle_checksum, spindle_control_smoothing_checksum)->by_default(0.1f)->as_number();
    if (smoothing_time * UPDATE_FREQ < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (UPDATE_FREQ * smoothing_time);

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
        THEKERNEL->streams->printf("ESC Spindle: spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    pwm_period = THEKERNEL->config->value(spindle_checksum, spindle_pwm_period_checksum)->by_default(20)->as_int();
    pwm_pin->period_us(pwm_period * 1000);
    pwm_pin->write(output_inverted ? 1 : 0);

    THEKERNEL->slow_ticker->attach(UPDATE_FREQ, this, &PWMSpindleControl::on_update_speed);
}


void PWMSpindleControl::on_second_tick(void *argument)
{
    // If waiting for a speed to be reached, display current speed
    if (waiting){
        report_speed();
    }
}

uint32_t PWMSpindleControl::on_update_speed(uint32_t dummy)
{
    // Calculate target RPM
    float new_rpm = smoothing_decay * target_rpm + (1.0f - smoothing_decay) * current_rpm;
    current_rpm = new_rpm;
    
    float target;

    if (spindle_on) {
        target = target_rpm;
    }else{
        target = 0.0;
    }

    // do ramp up/down:
    float new_rpm = smoothing_decay * target + (1.0f - smoothing_decay) * current_rpm;
    
    // convert to pwm signal:
    float pwm = pwm_factor_a * new_rpm + pwm_factor_b;
    
    // limit to valid pwm values:
    // 5-10% of 20ms period -> 1ms -2ms = servo signal
    current_pwm_value = confine(pwm, 5.0f, 10.0f);

    // set to output:
    if (output_inverted)
        pwm_pin->write(1.0f - current_pwm_value);
    else
        pwm_pin->write(current_pwm_value);
    
    // store old value
    current_rpm = new_rpm;

    return 0;
}

void PWMSpindleControl::turn_on() {
    spindle_on = true;
}

void PWMSpindleControl::turn_off() {
    spindle_on = false;
}


void PWMSpindleControl::set_speed(int rpm) {
    target_rpm = rpm;
    
    // we need to sleep until we reach the target speed
    this->waiting = true; 
    THEKERNEL->streams->printf("ESC Spindle: waiting for spindle to reach speed...\n");
    
    float tdiff;
    do {
        THEKERNEL->call_event(ON_IDLE, this);
        
        // check if ON_HALT was called (usually by kill button)
        if(THEKERNEL->is_halted()) {
            THEKERNEL->streams->printf("ESC Spindle: wait aborted by kill\n");
            break;
        }
        
        // calc speed diff
        tdiff =  fabs(current_rpm - target_rpm);
    } while (tdiff > 200);
    
    this->waiting = false;
    THEKERNEL->streams->printf("ESC Spindle: speed reached!\n");
}


void PWMSpindleControl::report_speed() {
    THEKERNEL->streams->printf("ESC Spindle: target speed %5f, current speed %5f [PWM = %5.3f]\n", 
                               target_rpm, current_rpm, current_pwm_value);
}

/*
void PWMSpindleControl::set_p_term(float p) {
}


void PWMSpindleControl::set_i_term(float i) {
}


void PWMSpindleControl::set_d_term(float d) {
}
*/

void PWMSpindleControl::report_settings() {
    THEKERNEL->streams->printf("ESC Spindle: scale_a: %0.6f scale_b: %0.6f period: %0.2f",
                               pwm_factor_a, pwm_factor_b, pwm_period);
}

