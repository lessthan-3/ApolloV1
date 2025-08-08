/*
 * File:   main.c
 * Author: niklas
 *
 * Created on June 8, 2025, 12:09 PM
 */


#include "lcd.h"
#include "config.h"






void init_system(void) {
    cli();
    
	lcd_init();
    configure_ports();
    configure_timers();
    
    //ADC
    ADMUX = (1 << REFS0);  // AVCC as reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // INT0 for zero crossing detection

    EICRA |= (1 << ISC01) | (1 << ISC00);  // Rising edge trigger
    EIMSK |= (1 << INT0);  // Enable INT0
    
    sei();
}

void configure_ports(void) {
    // Port B configuration
    DDRB = 0b10000111;
    // PB2 = Triac gate pulse (output)
    // PB7 = Heartbeat LED (output)
    
    // Port C configuration  
    DDRC = 0b00000100;
    // PC2 = Beeper (output)
    // PC3-PC5 = Switch inputs with pullups
    PORTC |= (1 << PC3) | (1 << PC4) | (1 << PC5);
    
    // Port D configuration
    DDRD = 0b11111010;
    // PD2 = INT0 zero crossing input
}

void configure_timers(void) {
    // Timer1: Used for triac firing delay after zero crossing
    // CTC mode, NO prescaler set initially (timer stopped)
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);  // CTC mode, NO clock source (timer stopped)
    TIMSK1 = (1 << OCIE1A); // Enable compare match interrupt
    TCNT1 = 0;              // Clear counter
}

uint16_t read_adc(uint8_t channel) {
    uint32_t sum = 0;
    
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    
    for (uint8_t i = 0; i < 10; i++) {
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));
        sum += ADC;
    }
    
    return (uint16_t)(sum / 10);
}

// Set motor speed (0-100%) - converts to delay value for triac

#define MAXDELAY 255
#define MINDELAY 35


void set_motor_speed(void) {
    uint16_t motor_speed_calc;
    
    if (motor_speed == 0) {
        motor_pwm = MAXDELAY;
    } else {

        if (motor_speed > 100) motor_speed = 100;
        
        if (FREQUENCY == 60) {  // 60Hz
            
            motor_speed_calc = motor_speed * MAXDELAY / 100;

            motor_pwm = MAXDELAY - motor_speed_calc; 
            if (motor_pwm < MINDELAY) motor_pwm = MINDELAY;
        }
        else{ // Need to adjust for 
                        
            motor_speed_calc = motor_speed * MAXDELAY / 100;

            motor_pwm = MAXDELAY - motor_speed_calc; 
            if (motor_pwm < MINDELAY) motor_pwm = MINDELAY;

        }
    }
}





void fire_triac(void) {

    PORTB |= (1 << PB2);    
    _delay_us(100);
    PORTB &= ~(1 << PB2); 
    
}

// Zero crossing interrupt - start timing for triac firing
ISR(INT0_vect) {
    // Zero crossing detected - start delay timer for triac firing
    
    if (motor_pwm < MAXDELAY) {  
        OCR1A = motor_pwm;  // Set delay time
        TCNT1 = 0;          // Reset timer

        //Starts timer for Triac Fire
        TCCR1B = (1 << WGM12) | (1 << CS12);  // CTC mode + prescaler 256

    }
    
}

// Timer1 Compare Match - time to fire triac
ISR(TIMER1_COMPA_vect) {
	fire_triac();
    // Stop timer until next zero crossing
     TCCR1B = (1 << WGM12);
}








void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->pid_scale = 1.0f;
    pid->pid_offset = 0.0f;
    pid->output_min = PID_OUTPUT_MIN;
    pid->output_max = PID_OUTPUT_MAX;
    pid->integral_max = PID_INTEGRAL_MAX;
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

//FIXED VERSION
float pid_calculate(pid_controller_t *pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;

    float p_scale = (pid->setpoint) * (0.0019) - 0.245;
    float i_scale = (pid->setpoint) * (0.005) - .68;
    float d_scale = (pid->setpoint ) * (0.0012) + 0.22;
    
    // Deadband - don't adjust if error is very small
    if (error < OUTPUT_DEADBAND && error > -OUTPUT_DEADBAND) {
        return pid->prev_output;
    }
    
    // Proportional term
    float proportional = pid->kp * error;
    
    // Calculate raw output BEFORE integral to check if we're rate limited
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = pid->kd * (error - pid->prev_error) / dt;
    }
    
    // Calculate what the output would be without integral
    float raw_output_no_integral = proportional + derivative;
    
    // Apply output limits to non-integral terms
    if (raw_output_no_integral > pid->output_max) {
        raw_output_no_integral = pid->output_max;
    } else if (raw_output_no_integral < pid->output_min) {
        raw_output_no_integral = pid->output_min;
    }
    
    // Check if we would be rate limited
    float output_change_no_integral = raw_output_no_integral - pid->prev_output;
    bool will_be_rate_limited = false;
    float potential_output = raw_output_no_integral;
    
    //rate limit 
    // if (output_change_no_integral > MAX_OUTPUT_CHANGE ) {
    //     potential_output = pid->prev_output + MAX_OUTPUT_CHANGE;
    //     will_be_rate_limited = true;
    // } else if (output_change_no_integral < -MAX_OUTPUT_CHANGE) {
    //     potential_output = pid->prev_output - MAX_OUTPUT_CHANGE;
    //     will_be_rate_limited = true;
    // }
    
    // Only accumulate integral if we're NOT rate limited or if error and integral have opposite signs
    if (!will_be_rate_limited || (error * pid->integral < 0)) {
        pid->integral += error * dt;
        
        // Apply integral limits
        if (pid->integral > pid->integral_max) {
            pid->integral = pid->integral_max;
        } else if (pid->integral < -pid->integral_max) {
            pid->integral = -pid->integral_max;
        }
    }
    
    // Calculate integral term
    float integral = pid->ki * pid->integral;
    
    // Update previous error for next derivative calculation
    pid->prev_error = error;
    
    // Calculate final raw output
    float raw_output = proportional + integral  + derivative;

    // Apply output limits
    if (raw_output > pid->output_max) {
        raw_output = pid->output_max;
    } else if (raw_output < pid->output_min) {
        raw_output = pid->output_min;
    }
    
    pid->prev_output = raw_output;
    return raw_output;
}

// PID setup function  (Uncomment later code to enable setting PID during startup)
void pid_setup(pid_controller_t *pid) {
    uint16_t adc_value;
    char buf[9];




        // Setup KP
    lcd_print("        ");
    lcd_print("SLEEP LIM");
    _delay_ms(1000);
    
    uint16_t while_ticks = 0;
	uint8_t delay = 0;
    while (while_ticks < MAX_WHILE_TICKS) {
		while_ticks++;
		
        adc_value = read_adc(7);
        
        delay = adc_value / 14;
        lcd_print("        ");
        // Display KP value with 1 decimal place
        snprintf(buf, 9, "P:%2d", delay);
        lcd_print(buf);
        _delay_ms(100);
    }
    sleep_deviation = delay;
    


    while_ticks = 0;
	uint16_t idlecount = 0;
    while (while_ticks < MAX_WHILE_TICKS) {
		while_ticks++;
		
        adc_value = read_adc(7);
        
        idlecount = adc_value / 14;
        lcd_print("        ");
        // Display KP value with 1 decimal place
        snprintf(buf, 9, "#:%2d", idlecount);
        lcd_print(buf);
        _delay_ms(100);
    }
    idle_decrease = idlecount;


    // // Setup KP
    // lcd_print("        ");
    // lcd_print("SET  KP ");
    // _delay_ms(1000);
    
    // while_ticks = 0;
    // while (while_ticks < MAX_WHILE_TICKS) {
	// 	while_ticks++;
		
    //     adc_value = read_adc(7);
    //     float kp_value = (float)adc_value / 1278.0f * PID_SCALE_FACTOR;
        
    //     lcd_print("        ");
    //     // Display KP value with 1 decimal place
    //     snprintf(buf, 9, "KP%2d.%02d", (int)kp_value, (int)((kp_value - (int)kp_value) * 100));
    //     lcd_print(buf);
    //     _delay_ms(100);
    // }
    // adc_value = read_adc(7);
    // pid->kp = (float)adc_value / 1278.0f * PID_SCALE_FACTOR;
    
    // // Setup KI
    // lcd_print("        ");
    // lcd_print("SET  KI ");
    // _delay_ms(1000);
	
	// while_ticks = 0;
    
    // while (while_ticks < MAX_WHILE_TICKS) {
    //     adc_value = read_adc(7);
	// 	while_ticks++;
    //     float ki_value = (float)adc_value / 2557.5f * PID_SCALE_FACTOR; 
        
    //     lcd_print("        ");
    //     snprintf(buf, 9, "KI .%2d%02d", (int)ki_value, (int)((ki_value - (int)ki_value) * 100));
    //     lcd_print(buf);
    //     _delay_ms(100);
		
    // }
    // adc_value = read_adc(7);
    // pid->ki = (float)adc_value / 2557.5f * PID_SCALE_FACTOR / 10;
    
    // // Setup KD
    // lcd_print("        ");
    // lcd_print("SET  KD ");
    // _delay_ms(1000);
    
    // while_ticks = 0;
    // while (while_ticks < MAX_WHILE_TICKS) {
    //     adc_value = read_adc(7);
    //     float kd_value = (float)adc_value / 2557.5f * PID_SCALE_FACTOR;
        
    //     lcd_print("        ");
    //     snprintf(buf, 9, "KD %2d.%02d", (int)kd_value, (int)((kd_value - (int)kd_value) * 100));
    //     lcd_print(buf);
    //     _delay_ms(100);
	// 	while_ticks++;
    // }
    // adc_value = read_adc(7);
    // pid->kd = (float)adc_value / 2557.5f * PID_SCALE_FACTOR;

}


void overtemp_check(float temp_sense){
    if (temp_sense > OVERTEMP_SETPOINT) {
        if (over_temp_counter <= 50) {
            over_temp_counter++;
        } else {
            display_overtemp();
            motor_speed = 0;
            set_motor_speed();
            pid_reset(&pressure_pid); // Reset PID when shutting down
            over_temp_flag = true;
            return;
        }
    } else {
        if (over_temp_counter > 0) over_temp_counter--;
    }
}



void motor_control_loop(void) {
    uint16_t pot_setting, pressure, adc_value;
    float temp_sense;
    static uint32_t last_time = 0;
    uint8_t sleep_deviation_scaled = 0;
    
    // Read temperature
    adc_value = read_adc(0);
    temp_sense = (adc_value - TEMP_OFFSET) * TEMP_MULT;
    
    overtemp_check(temp_sense);
    
    // Read pressure
    adc_value = read_adc(6);
    if (adc_value < PRESS_OFFSET) {
        pressure = 0;
    } else {
        pressure = ((adc_value - PRESS_OFFSET) * PRESS_MULTIPLIER) / PRESS_DIVISOR;
    }

    // Read pot setting
    adc_value = read_adc(7);
    pot_setting = adc_value;
    
    if (pot_setting >= 15) {
        pot_setting = ((pot_setting + 4) * 28) /45 + 200;
    } else {
        pot_setting = 0;
    }

    sleep_deviation_scaled = sleep_deviation;
    
    // Motor off if pot at zero
    if (pot_setting == 0) {
        lcd_print("        ");
        motor_speed = 0;
        idle_timer = 0;
        set_motor_speed();
        pid_reset(&pressure_pid); // Reset PID when turning off
        lcd_print("Off");
        idle_mode = false;
        return;
    }


    if (idle_mode) {
        if (pressure < IDLE_PRESSURE_THRESHOLD) {
            // Exit idle if pressure dips below threshold
			inside_count = 0;
            idle_mode = false;
            lcd_print("        ");
            lcd_print("RESUME");
            _delay_ms(200);
        } else {
            // Stay in idle mode: hold motor at 30%
            motor_speed = IDLE_MOTOR_SPEED;
            set_motor_speed();
            pid_reset(&pressure_pid);
            lcd_print("        ");
            lcd_print("IDLE");
            _delay_ms(100);
            return;
        }
    }
    // Update motor speed from PID
    pressure_pid.setpoint = (float)pot_setting;

    float pid_output = pid_calculate(&pressure_pid, (float)pressure, .1f);
    
    motor_speed = (uint16_t)(pid_output + 0.5f); 

    //bounding speed
    if (motor_speed > 100) motor_speed = 100;
    if (motor_speed < 10) motor_speed = 10;
    last_motor_speed = motor_speed;


    //Display    
    
    //smoothing of display
    if(print_pressure + 10 < pressure){
        print_pressure = pressure;
    } else if(print_pressure - 10 > pressure){
        print_pressure = pressure;
    }  else print_pressure = pot_setting;
    if (display_count == 0){
        seconds++;
        lcd_print("        ");
        char buf[9];


        snprintf(buf, 9, "%2u>%2u", inside_count, pot_setting);
        lcd_print(buf);

        // if(pot_setting > 800){
        //     snprintf(buf, 9, "%2u>MAX", print_pressure);
        //     lcd_print(buf);
        //     motor_speed = 100;
        // } else{
        //     snprintf(buf, 9, "%2u>%2u", print_pressure, pot_setting);
        //     lcd_print(buf);
        // }
    }
    

    
    if (!idle_mode) {


        if ((pressure > (pot_setting - sleep_deviation_scaled)) &&
            (pressure < (pot_setting + sleep_deviation_scaled))) {
                    inside_count++;
        } else if(inside_count >= idle_decrease) inside_count = inside_count - idle_decrease;


        if (inside_count >= IDLE_OUTSIDE_THRESHOLD) {
            inside_count = 0;
            motor_speed = IDLE_MOTOR_SPEED;
            set_motor_speed();
            pid_reset(&pressure_pid);
            idle_mode = true;
            lcd_print("        ");
            lcd_print("IDLE");
            seconds = 0;
            _delay_ms(2000);
            last_pot_setting = pot_setting;
            return;
        }

    }

    display_count++;
    display_count = display_count % 20;
    
    
    set_motor_speed();
    
}






// --- LCD Display ---

void display_idle(void) {
    lcd_print("        ");
    lcd_print("PWRPAUSE");
}

void display_shutdown(void) {
    lcd_print("        ");
    lcd_print("SHUTDOWN");
}

void display_overtemp(void) {
    lcd_print("        ");
    lcd_print("OVERHEAT");
}


void display_start(void) {
    lcd_print("        ");
    lcd_print("APOLLO");
    _delay_ms(500);
    lcd_print("        ");
    lcd_print("START");
    _delay_ms(500);
}



int main(void) {
    
    

    init_system();
    display_start();

    pid_init(&pressure_pid, PID_KP, PID_KI, PID_KD);
    pid_setup(&pressure_pid);
	
	
	
    while (1) {
        motor_control_loop();
        _delay_ms(MOTOR_LOOP_DELAY);
    }
    
    return 0;
}


