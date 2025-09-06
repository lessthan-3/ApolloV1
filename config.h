/* 
 * File:   config.h
 * Author: niklas
 *
 * Created on June 8, 2025, 8:37 PM
 */

#ifndef CONFIG_H
#define	CONFIG_H

#define F_CPU 8000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


#define BAUD 54720
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// EEPROM access
#include <avr/eeprom.h>


//AC Power Frequency

#define FREQUENCY 60


// Constants 
#define OVERTEMP_SETPOINT 130
#define FILTER_TEMP_SETPOINT 115
#define POWER_DOWN 900
#define PRESS_MULTIPLIER 39
#define PRESS_DIVISOR 25
#define PRESS_OFFSET 46
#define TEMP_MULT -0.87
#define TEMP_OFFSET 210

#define OVERTEMP_THRESHOLD 130
#define IDLE_TIMEOUT_SEC   60
#define POWER_DOWN_SEC     900


// PID Controller Default Constants
#define PID_KP  11.0f    // Proportional gain
#define PID_KI  0.5f    // Integral gain  
#define PID_KD  1.5f    // Derivative gain

// PID limits
#define PID_OUTPUT_MIN  0.0f    // Minimum PID output
#define PID_OUTPUT_MAX  100.0f  // Maximum PID output
#define PID_INTEGRAL_MAX 1000.0f // Anti-windup limit for integral term

// PID tuning settings
#define MAX_WHILE_TICKS 45  // Time to set each parameter (3 seconds)
#define PID_SCALE_FACTOR   15.0f // Scale pot reading (0-1023) to reasonable PID values

#define MAX_OUTPUT_CHANGE  4    // Maximum change in output per loop (prevents jumping)
#define OUTPUT_DEADBAND   2.0f   // Don't change output if error is very small


#define MOTOR_LOOP_DELAY 8
#define IDLE_MOTOR_SPEED 30
#define IDLE_PRESSURE_THRESHOLD 187

volatile uint8_t over_temp_counter = 0;




#define SLEEP_CHECK_INTERVAL 50  // each worth 160 ms 
#define SLEEP_TIME 8 //ms

static uint16_t motor_speed_history[SLEEP_CHECK_INTERVAL] = {0};
static uint8_t history_index = 0;
static bool sleep_mode = false;
static uint16_t last_pot_setting = 0;
uint16_t last_motor_speed = 0;

uint16_t print_pressure = 0;
static bool over_temp_flag = false;
uint16_t display_count = 0;
uint16_t IDLE_OUTSIDE_THRESHOLD = 4000;


uint8_t sleep_deviation = 0;
float temp_sense = 0.0;
uint8_t display_timer = 0;
uint8_t idle_timeout = 60;
uint8_t idle_decrease = 0;

float pid_integral = 0;
float pid_last_error = 0;

uint16_t inside_count = 0;
uint16_t total_count = 0;

uint16_t power_timer = 0;

uint16_t last_pressure1 =0;
uint16_t last_pressure2 =0;
uint16_t last_pressure3 =0;

// PID structure to hold state
typedef struct {
    float kp, ki, kd;           // PID gains
    float setpoint;             // Desired value
    float integral;             // Integral accumulator
    float prev_error;           // Previous error for derivative
    float prev_output;          // Previous output for rate limiting
    float output_min, output_max; // Output limits
    float integral_max;         // Integral windup limit
    float pid_scale;
    float pid_offset;
} pid_controller_t;



// Global PID controller instance
static pid_controller_t pressure_pid;


// Global variables
volatile uint16_t time_counter = 0;
volatile uint16_t seconds = 0;
volatile uint16_t idle_timer = 0;
volatile bool timer_update_flag = false;
volatile uint32_t hour_counter = 0;

volatile uint16_t printtemp = 0;
uint16_t ptemp = 0;

uint16_t motor_speed = 0;
uint16_t motor_pwm = 225; 

uint8_t frequency = 120;  

uint8_t motor_incr_max = 25;
uint8_t motor_incr_max_down = 10;
static bool idle_mode = false;



// Function prototypes
void init_system(void);
void configure_ports(void);
void configure_timers(void);
uint16_t read_adc(uint8_t channel);
void set_motor_speed(void);
void fire_triac(void);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);
void pid_reset(pid_controller_t *pid);
float pid_calculate(pid_controller_t *pid, float current_value, float dt);
void pid_setup(pid_controller_t *pid);
void motor_control_loop(void);
void display_idle(void);
void display_shutdown(void);
void display_start(void);



void display_overtemp(void);



#endif	/* CONFIG_H */

