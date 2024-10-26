#ifndef X_DRIVE_ESP_H
#define X_DRIVE_ESP_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"
#include "ps5.h"
#include <inttypes.h>
#include <math.h>

static const char *TAG = "Log";

#define MOTOR_1_PWM GPIO_NUM_19 // Under timer0
#define MOTOR_2_PWM GPIO_NUM_18 // Under timer0
#define MOTOR_3_PWM GPIO_NUM_16 // Under timer1
#define MOTOR_4_PWM GPIO_NUM_17 // Under timer1

#define MOTOR_1_DIR GPIO_NUM_5
#define MOTOR_2_DIR GPIO_NUM_15
#define MOTOR_3_DIR GPIO_NUM_13
#define MOTOR_4_DIR GPIO_NUM_12

#define MOTOR_DIR_BIT_MASK ((1ULL << MOTOR_1_DIR) | (1ULL << MOTOR_2_DIR) | (1ULL << MOTOR_3_DIR) | (1ULL << MOTOR_4_DIR))

#define ENCODER1_PIN_A GPIO_NUM_36
#define ENCODER1_PIN_B GPIO_NUM_39
#define ENCODER2_PIN_A GPIO_NUM_35
#define ENCODER2_PIN_B GPIO_NUM_34
#define ENCODER3_PIN_A GPIO_NUM_32
#define ENCODER3_PIN_B GPIO_NUM_27
#define ENCODER4_PIN_A GPIO_NUM_26
#define ENCODER4_PIN_B GPIO_NUM_25

// PWM Cofiguration
#define TIMER_RESOLUTION 80000000 // 80Mhz which is half of the 160Mhz source used
#define COUNTER_PERIOD 4000 // 4000 ticks for 20kHz PWM used for Cytron MD10C motor driver

#define MCPWM_COMPRATOR_MAX 1999 // Max value for comparator

// PCNT Configuration
#define PCNT_HIGH_LIMIT 32767 // Max for signed 16 bit counter
#define PCNT_LOW_LIMIT -32768 // Min for signed 16 bit counter

#define INIT_TARGET_SPEED_M1 0 // Initial speed in pulse per us
#define INIT_TARGET_SPEED_M2 0
#define INIT_TARGET_SPEED_M3 0
#define INIT_TARGET_SPEED_M4 0

// PID Constant, needs to be adjusted of the sampling rate changes
#define KP_M1 10000
#define KI_M1 1000
#define KD_M1 0

#define KP_M2 10000
#define KI_M2 1000
#define KD_M2 0

#define KP_M3 10000
#define KI_M3 1000
#define KD_M3 0

#define KP_M4 1000
#define KI_M4 100
#define KD_M4 0

#define INTEGRAL_LIMIT_MAX 2000
#define INTEGRAL_LIMIT_MIN -2000

// Periodic timer period, will determine sampling rate of the PID loop
// The PID constants needs to be modified if the sampling rate is changed
#define TIMER_PERIOD 1000 

typedef struct {
    mcpwm_cmpr_handle_t motor;
    pcnt_unit_handle_t encoder;
    float target_speed;
    float kp;
    float ki;
    float kd;
    float integral_limit_max;
    float integral_limit_min;
    // The items need not to be configured during initialization
    float integral;
    float prev_err;
    float pTerm;
    float iTerm; 
    float dTerm; 
    float curr_speed; 
    float err; 
    float output;

}pid_controller_t;

typedef struct {
    int prev_count;
    int curr_count;
    int delta_count;
}encoder_pulses_t;

typedef struct{
    int64_t prev_time;
    int64_t curr_time;
    int64_t delta_time;
}time_t;

// Global constants for GPIO pins
extern const gpio_num_t encoder_array[4][2];
extern const gpio_num_t motor_dir_array[4];

// Global variables
extern encoder_pulses_t encoder_pulses[4];
extern time_t time_encoder;
extern int64_t prev_time, curr_time, delta_time;   

// Public functions
void initialize_peripherals(pid_controller_t *pid_params);
void print_debug(void *arg);

// Private Functions
void initialize_pid_controller(pid_controller_t *pid_params, mcpwm_cmpr_handle_t motor, pcnt_unit_handle_t encoder, float target_speed, float kp, float ki, float kd, float integral_limit_max, float integral_limit_min);
void update_PID_output(void *arg);
void cb (void *arg, ps5_t ps5, ps5_event_t event);

#endif