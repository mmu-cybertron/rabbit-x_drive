#include "stepper.h"

void stepper_add_pulses(stepper_isr_args_t *stepper_isr_args, int pulses){
    stepper_isr_args->target_pulses += pulses; // Add the additional pulses
    stepper_isr_args->stepper_comparator_value = STEPPER_MCPWM_COMPRATOR_MIDDLE; // This variable is set to so that the ISR know the stepper is running
    mcpwm_comparator_set_compare_value(stepper_isr_args->stepper_comparator, STEPPER_MCPWM_COMPRATOR_MIDDLE); // Start the stepper
}

// This function is a ISR
// Should only be called during setup
void stepper_stop_pulses(void *args){
    stepper_isr_args_t *stepper_isr_args = (stepper_isr_args_t*) args;
    mcpwm_comparator_set_compare_value(stepper_isr_args->stepper_comparator, 0); // Stop the stepper
    stepper_isr_args->stepper_comparator_value = 0; // Set the variable to 0 so that the ISR know the stepper is stopped
    stepper_isr_args->executed_pulses = 0; // Reset the executed pulses
    stepper_isr_args->target_pulses = 0; // Reset the target pulses
    gpio_isr_handler_remove(STEPPER_DIR_PIN); // Remove the ISR handler

}

// This function is a ISR
void stepper_ISR (mcpwm_cmpr_handle_t comparator, const mcpwm_compare_event_data_t *edata, void *args){
    
    stepper_isr_args_t *stepper_isr_args = (stepper_isr_args_t*) args;

    if (stepper_isr_args->stepper_comparator_value == 0){
        return;
    }

    if (stepper_isr_args->executed_pulses >= stepper_isr_args->target_pulses){
        // reduce the amount of times calling this function
        // Stop the stepper
        stepper_isr_args->stepper_comparator_value = 0;
        mcpwm_comparator_set_compare_value(comparator, 0);

    } else {
        // mcpwm_comparator_set_compare_value(comparator, STEPPER_MCPWM_COMPRATOR_MIDDLE);
        stepper_isr_args->stepper_comparator_value = STEPPER_MCPWM_COMPRATOR_MIDDLE;
        stepper_isr_args->executed_pulses++;
    }
    return; 
}

void init_stepper(stepper_isr_args_t *stepper_isr_args)
{
    gpio_set_direction(STEPPER_DIR_PIN, GPIO_MODE_OUTPUT);

    // ESP_LOGI(STEPPER, "Configuring Stepper GPIO");
    // gpio_config_t stepper_gpio_config = {
    //     .pin_bit_mask = STEPPER_BIT_MASK,
    //     .mode = GPIO_MODE_OUTPUT,
    // };
    // gpio_config(&stepper_gpio_config);


    // Configure mcpwm timer
    mcpwm_timer_handle_t stepper_timer = NULL;
    mcpwm_timer_config_t stepper_timer_config = {
        .group_id = STEPPER_TIMER_GROUP, // Use timer 3, as timer 1 & 2 is already used for motor on the base
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M, // 160Mhz default clock source
        .resolution_hz = STEPPER_TIMER_RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP, //Count up down for symetric waveform to reduce harmonics when driving DC motors
        .period_ticks = STEPPER_COUNTER_PERIOD
    };

    mcpwm_new_timer(&stepper_timer_config, &stepper_timer);

    // Configure mcpwm operator
    mcpwm_oper_handle_t stepper_operator = NULL;
    mcpwm_operator_config_t stepper_operator_config = {
        .group_id = STEPPER_TIMER_GROUP,
    };
    mcpwm_new_operator(&stepper_operator_config, &stepper_operator);
    mcpwm_operator_connect_timer(stepper_operator, stepper_timer);

    // Configure mcpwm comparator

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tep = true
    };
    mcpwm_new_comparator(stepper_operator, &comparator_config, &(stepper_isr_args->stepper_comparator));

    // Configure mcpwm generator
    mcpwm_gen_handle_t stepper_generator = NULL;
    mcpwm_generator_config_t stepper_generator_config = {.gen_gpio_num = STEP_PIN};
    mcpwm_new_generator(stepper_operator, &stepper_generator_config, &stepper_generator);

    /* 
    Configure the correct wave characteristics to interface with motor driver
    Dual Edge Symmetric Waveform - Active Low (Modified to active high)
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#dual-edge-symmetric-waveform-active-low
    */ 
    mcpwm_generator_set_action_on_timer_event(stepper_generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(stepper_generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, stepper_isr_args->stepper_comparator, MCPWM_GEN_ACTION_LOW));
    
    // Callback event for stepper motor ISR to count how many pulses are generated
    mcpwm_comparator_event_callbacks_t stepper_callback_event = {
        .on_reach = stepper_ISR
    };
    mcpwm_comparator_register_event_callbacks(stepper_isr_args->stepper_comparator, &stepper_callback_event, (void*)stepper_isr_args);

    // Set stepper to off initially
    mcpwm_comparator_set_compare_value(stepper_isr_args->stepper_comparator, 0);

    mcpwm_timer_enable(stepper_timer);
    mcpwm_timer_start_stop(stepper_timer, MCPWM_TIMER_START_NO_STOP);
    
    // mcpwm_comparator_set_compare_value(stepper, STEPPER_MCPWM_COMPRATOR_MIDDLE);
    // stepper_pulses.target_pulses = 1000;
}
