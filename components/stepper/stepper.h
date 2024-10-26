#ifndef STEPPER_H
#define STEPPER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#define STEP_PIN GPIO_NUM_4
#define STEPPER_DIR_PIN GPIO_NUM_15

#define STEPPER_BIT_MASK ((1ULL << STEPPER_DIR_PIN))

// PWM Cofiguration
#define STEPPER_TIMER_RESOLUTION 4000000 // 8Mhz
#define STEPPER_COUNTER_PERIOD 4000 // Value chosen after testing

#define STEPPER_MCPWM_COMPRATOR_MAX 3999 // Max value for comparator
#define STEPPER_MCPWM_COMPRATOR_MIDDLE 2000 // Max value for comparator

#define STEPPER_TIMER_GROUP 0

// static const char *STEPPER = "Stepper";

// This struct should be decalared as global variable in the main program
// Struct to pass arguments to ISR
typedef struct{
    mcpwm_cmpr_handle_t stepper_comparator;
    uint32_t stepper_comparator_value;
    int executed_pulses;
    int target_pulses;
}stepper_isr_args_t;

// Public Functions
void init_stepper(stepper_isr_args_t *stepper_isr_args);
void stepper_add_pulses(stepper_isr_args_t *stepper_isr_args, int pulses);
void stepper_stop_pulses(void *args);

// Private Functions
// ISR for stepper motor, should not be called directly
void stepper_ISR (mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t *edata, void *args);


#endif
