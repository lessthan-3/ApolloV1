/*
 * File:   main.c
 * Author: niklas
 *
 * Created on June 8, 2025, 12:09 PM
 */


#include "lcd.h"
#include "config.h"





// Call this in init_system() after configuring timers
uint8_t detect_ac_frequency(void) {
    // Start Timer1 with prescaler 256
    TCCR1B = (1 << WGM12) | (1 << CS12);
    TCNT1 = 0;
    zc_ready = false;

    // Wait for two zero crossings
    while (!zc_ready);

    uint16_t period = zc_time2 - zc_time1; // Timer ticks between zero crossings

    // Stop Timer1
    TCCR1B = (1 << WGM12); 

    // 60Hz half-cycle: ~8333us / 32us = ~260 ticks
    // 50Hz half-cycle: ~10000us / 32us = ~313 ticks

    char buf[9];
    lcd_print("        ");
        // Display KP value with 1 decimal place
    snprintf(buf, 9, "T:%2u", period);
    lcd_print(buf);
    _delay_ms(5000);
    if (period > 290) {
        maxdelay = MAXDELAY50;
    } else if (period > 200) {
        maxdelay = MAXDELAY;
    } else {
        return 0; // Unknown/error
    }
    
}

void init_system(void) {
    cli();
    
	lcd_init();
    configure_ports();
    configure_timers();

    display_start();
    
    //ADC
    ADMUX = (1 << REFS0);  // AVCC as reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // INT0 for zero crossing detection

    EICRA |= (1 << ISC01) | (1 << ISC00);  // Rising edge trigger
    EIMSK |= (1 << INT0);  // Enable INT0

    // Initialize beeper
    beeper_state = 0;
    beeper_counter = 0;
    beeper_off_func();  // Ensure beeper starts off
    
    sei();
    //detect_ac_frequency();
    maxdelay = MAXDELAY50;
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




void set_motor_speed(void) {
    uint16_t motor_speed_calc;
    
    if (motor_speed == 0) {
        motor_pwm = maxdelay;
    } else {

        if (motor_speed > 100) motor_speed = 100;
        
            
        motor_speed_calc = motor_speed * maxdelay / 100;

        motor_pwm = maxdelay - motor_speed_calc; 
        if (motor_pwm < MINDELAY) motor_pwm = MINDELAY;
    }
}





void fire_triac(void) {

    PORTB |= (1 << PB2);    
    _delay_us(100);
    PORTB &= ~(1 << PB2); 
    
}

// Zero crossing interrupt - start timing for triac firing
ISR(INT0_vect) {

    if(!zc_ready){
    if (zc_count == 0) {
        zc_time1 = TCNT1;
        zc_count++;
    } else if (zc_count == 1) {
        zc_time2 = TCNT1;
        zc_count++;
        zc_ready = true;
    } else {
        zc_ready = true;
    } 
    }


    // Zero crossing detected - start delay timer for triac firing
    
    else if (motor_pwm < maxdelay) {  
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
    lcd_print("SETUP");
    _delay_ms(1000);
    
    uint16_t while_ticks = 0;
	uint8_t delay = 0;
    while (while_ticks < MAX_WHILE_TICKS) {
		while_ticks++;
		adc_value = read_adc(7);
        
        delay = adc_value / 10;
        lcd_print("        ");
        // Display KP value with 1 decimal place
        snprintf(buf, 9, "P:%2d", delay);
        lcd_print(buf);
        _delay_ms(100);
    }
    idle_range = delay;
    
}

// Hour meter functions

// Initialize hour meter - read from EEPROM
void hour_meter_init(void) {
    // Read the stored hour meter value from EEPROM (stored as uint32_t)
    hour_meter_tenths = eeprom_read_dword((uint32_t*)EEPROM_HOUR_METER_ADDR);
    
    // Sanity check - if EEPROM is uninitialized (0xFFFFFFFF), reset to 0
    if (hour_meter_tenths == 0xFFFFFFFF) {
        hour_meter_tenths = 0;
        hour_meter_save();
    }
    
    hour_meter_counter = 0;
    motor_running = false;
}

// Update hour meter counter (called every motor control loop iteration)
void hour_meter_update(void) {
    // Only count when motor is actually running (not off, not in idle mode with very low speed)
    if (motor_running) {
        hour_meter_counter++;
        
        // Check if we've reached 0.1 hour of operation
        // MOTOR_LOOP_DELAY is in ms, so we need to calculate cycles for 360 seconds (0.1 hour)
        // 360 seconds = 360000 ms / MOTOR_LOOP_DELAY ms per cycle
        if (hour_meter_counter >= (360000 / MOTOR_LOOP_DELAY)) {
            hour_meter_tenths++;
            hour_meter_counter = 0;
            hour_meter_save();  // Save to EEPROM every 0.1 hour
        }
    }
}

// Save hour meter to EEPROM
void hour_meter_save(void) {
    eeprom_update_dword((uint32_t*)EEPROM_HOUR_METER_ADDR, hour_meter_tenths);
}

// Display hour meter on LCD
void display_hour_meter(void) {
    char buf[9];
    uint32_t total_hours = hour_meter_tenths / 10;
    uint8_t tenths = hour_meter_tenths % 10;
    
    //lcd_print("        ");
    
    // Display format: "000.0 Hrs" for hours with one decimal place
    if (total_hours > 999) {
        // If over 999 hours, just show "999.9Hrs"
        snprintf(buf, 9, "999.%uHrs", tenths);
    } else {
        snprintf(buf, 9, "%03lu.%uHrs", (unsigned long)total_hours, tenths);
    }
    
    lcd_print(buf);

    _delay_ms(400);
}

// Beeper functions

// Turn beeper on
void beeper_on_func(void) {
    PORTC |= (1 << PC2);  // Set PC2 high to turn on beeper
    beeper_on = true;
}

// Turn beeper off
void beeper_off_func(void) {
    PORTC &= ~(1 << PC2);  // Set PC2 low to turn off beeper
    beeper_on = false;
}

// Update beeper based on idle state and timing
void beeper_update(void) {
    beeper_counter++;
    
    switch(beeper_state) {
        case 0:  // Beeper off
            beeper_off_func();
            beeper_counter = 0;
            break;
            
        case 1:  // Intermittent beep (10 seconds before idle)
            // Beep pattern: 0.5s on, 1.5s off (2 second cycle)
            // At 8ms loop delay: 0.5s = 62.5 loops, 1.5s = 187.5 loops
            if (beeper_counter < 63) {  // ~0.5 seconds on
                beeper_on_func();
            } else if (beeper_counter < 250) {  // ~1.5 seconds off (total 2s cycle)
                beeper_off_func();
            } else {
                beeper_counter = 0;  // Reset for next cycle
            }
            break;
            
        case 2:  // Solid beep (5 seconds before idle)
            beeper_on_func();
            break;
            
        default:
            beeper_off_func();
            beeper_state = 0;
            break;
    }
}

void overtemp_check(float temp_sense){
    if (temp_sense > FILTER_TMP){
        check_filter = true;
    } else {
        check_filter = false;
    }

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
    if (temp_sense <= OVERTEMP_EXIT && over_temp_flag == true ){
        over_temp_flag = false;
        lcd_print("        ");
        lcd_print("RESTART");
        _delay_ms(1000);
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

    if(over_temp_flag){
        return;
    }
    
    // Check if system is in shutdown mode (after 15 min PowerPause timeout)
    if(shutdown_flag){
        motor_speed = 0;
        set_motor_speed();
        display_shutdown();
        motor_running = false;
        return;
    }
    
    // Read pressure
    adc_value = read_adc(6);
    if (adc_value < PRESS_OFFSET) {
        pressure = 0;
    } else {
        pressure = ((adc_value - PRESS_OFFSET) * PRESS_MULTIPLIER) / PRESS_DIVISOR;
    }
    
    pressure = (pressure + pressure + last_pressure)/3;

    last_pressure = pressure; 

    // Read pot setting
    adc_value = read_adc(7);
    pot_setting = adc_value;
    
    if (pot_setting >= 15) {
        pot_setting = ((pot_setting + 4) * 28) /45 + 200;
    } else {
        pot_setting = 0;
    }

    

    sleep_deviation_scaled = sleep_deviation;
    
    // Motor off if pot at zero - display hour meter
    if (pot_setting == 0) {
        lcd_print("        ");
        motor_speed = 0;
        idle_timer = 0;
        set_motor_speed();
        pid_reset(&pressure_pid); // Reset PID when turning off
        motor_running = false;  // Motor is not running
        display_hour_meter();  // Display hour meter when motor is off
        idle_mode = false;
        return;
    }

    // Motor is running for hour meter purposes
    motor_running = true;
    
    // Update hour meter
    hour_meter_update();


    if (idle_mode) {
        // Increment PowerPause timer (motor_control_loop is called every MOTOR_LOOP_DELAY ms)
        // Calculate seconds: each loop is MOTOR_LOOP_DELAY ms, so increment every (1000/MOTOR_LOOP_DELAY) loops
        static uint16_t powerpause_loop_counter = 0;
        powerpause_loop_counter++;
        if (powerpause_loop_counter >= (24000 / MOTOR_LOOP_DELAY)) {
            powerpause_timer++;
            powerpause_loop_counter = 0;
            
            // Check if 15 minutes have elapsed
            if (powerpause_timer >= POWERPAUSE_TIMEOUT_SEC) {
                shutdown_flag = true;
                motor_speed = 0;
                set_motor_speed();
                motor_running = false;
                display_shutdown();
                return;
            }
        }
        
        if (pressure < idle_pressure_threshold) {
            // Exit power pause if pressure dips below threshold
			inside_count = 0;
            idle_mode = false;
            powerpause_timer = 0;  // Reset timer when exiting PowerPause
            beeper_state = 0;  // Turn off beeper when exiting PowerPause
            beeper_off_func();
            lcd_print("        ");
            lcd_print("RESUME");
            _delay_ms(200);
        } else if (pot_setting > last_pot_setting + 3 || pot_setting < last_pot_setting - 3 ){
            inside_count = 0;
            idle_mode = false;
            powerpause_timer = 0;  // Reset timer when exiting PowerPause
            beeper_state = 0;  // Turn off beeper when exiting PowerPause
            beeper_off_func();
            lcd_print("        ");
            lcd_print("RESUME");
            _delay_ms(200);
        }
        else  {
            // Stay in idle mode: hold motor at 30%
            motor_speed = IDLE_MOTOR_SPEED;
            set_motor_speed();
            pid_reset(&pressure_pid);
            //lcd_print("        ");
            //lcd_print("IDLE");
            //_delay_ms(100);
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

        if(check_filter){
            lcd_print("CHK FLTR");
        } else if(idle_state == 3){
            lcd_print("IDLE NOW");  // 5 seconds before idle - solid beep
        } else if(idle_state == 2){
            lcd_print("IDLESOON");  // 10 seconds before idle - intermittent beep
        }  else {
            snprintf(buf, 9, "%2u.%u PSI", print_pressure / 100, print_pressure % 100);
            lcd_print(buf);

        }
       
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
        else inside_count = 0;

        // Update idle state and beeper warnings
        if(inside_count >= IDLE_OUTSIDE_THRESHOLD_SECOND_WARN){
            idle_state = 3;  // 5 seconds before idle - solid beep
            beeper_state = 2;
        } else if(inside_count >= IDLE_OUTSIDE_THRESHOLD_FIRST_WARN){
            idle_state = 2;  // 10 seconds before idle - intermittent beep
            beeper_state = 1;
        } else {
            idle_state = 0;  // Normal operation - no beep
            beeper_state = 0;
        }
        
        // Update beeper based on state
        beeper_update();
        
        if (inside_count >= IDLE_OUTSIDE_THRESHOLD) {
            inside_count = 0;
            motor_speed = IDLE_MOTOR_SPEED;
            set_motor_speed();
            pid_reset(&pressure_pid);
            idle_mode = true;
            powerpause_timer = 0;  // Reset PowerPause timer when entering idle mode
            beeper_state = 0;  // Turn off beeper when entering idle mode
            beeper_off_func();
            lcd_print("        ");
            lcd_print("IDLE");
            seconds = 0;
            _delay_ms(6000);
            adc_value = read_adc(6);
            if (adc_value < PRESS_OFFSET) {
                pressure = 0;
            } else {
                pressure = ((adc_value - PRESS_OFFSET) * PRESS_MULTIPLIER) / PRESS_DIVISOR;
            }
            lcd_print("        ");
            lcd_print("PWRPAUSE");

            last_pot_setting = pot_setting;

            idle_pressure_threshold = pressure - idle_range;
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
    _delay_ms(1000);
    lcd_print("        ");
    lcd_print("SPRAYERS");
    _delay_ms(1000);
    lcd_print("        ");
    lcd_print("  HVLP");
    _delay_ms(1000);
    lcd_print("        ");
    lcd_print("FW 1.0");
    _delay_ms(2000);
}



int main(void) {
    
    

    init_system();


    pid_init(&pressure_pid, PID_KP, PID_KI, PID_KD);
    hour_meter_init();  // Initialize hour meter from EEPROM
    maxdelay = MAXDELAY;
    //detect_ac_frequency();
    // if(maxdelay== MAXDELAY50){
    //     lcd_print("        ");
    //     lcd_print("50Hz AC");
    //     _delay_ms(500);
    // }
    // else if(maxdelay == MAXDELAY){
    //     lcd_print("        ");
    //     lcd_print("60Hz AC");
    //     _delay_ms(500);
    // }
    else{
        lcd_print("        ");
        lcd_print("Restart");
        _delay_ms(500);
        return 0;
    }


    //pid_setup(&pressure_pid);
	
	
	
    while (1) {
        motor_control_loop();
        _delay_ms(MOTOR_LOOP_DELAY);
    }
    
    return 0;
}


