/* Exact speed control of DC motors using PID controller with ESP32
PCNT peripheral of ESP32 to read encoder pulses
Sampling rate of PID loop is 1ms, or 1000Hz
PID Loop Is constructed inside a periodic timer callback function 

MCPWM peripheral of ESP32 to drive 4 Cytron Motor Driver with 20kHz PWM
Use mcpwm_comparator_set_compare_value(motorX, DESIRED_LEVEL) to set the motor speed;

DESIRED_LEVEL max: COUNTER_PERIOD/2 - 1 = 1999
DESIRED_LEVEL min: 0

Documentation
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#dual-edge-symmetric-waveform-active-low
https://github.com/espressif/esp-idf/tree/v5.3/examples/peripherals/mcpwm/mcpwm_servo_control */

#include "x_drive_esp.h"

const gpio_num_t encoder_array[4][2] = {
    {ENCODER1_PIN_A, ENCODER1_PIN_B},
    {ENCODER2_PIN_A, ENCODER2_PIN_B},
    {ENCODER3_PIN_A, ENCODER3_PIN_B},
    {ENCODER4_PIN_A, ENCODER4_PIN_B}};

const gpio_num_t motor_dir_array[4] = {
    MOTOR_1_DIR,
    MOTOR_2_DIR,
    MOTOR_3_DIR,
    MOTOR_4_DIR};

// All the encoder pulses are stored in this array
// Global variable can be accessed by all functions
encoder_pulses_t encoder_pulses[4] = {0};
time_t time_encoder = {0};

void initialize_peripherals(pid_controller_t *pid_params){

    gpio_config_t motor_dir_config = {
        .pin_bit_mask = MOTOR_DIR_BIT_MASK,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&motor_dir_config);

    // Configure mcpwm timer
    mcpwm_timer_handle_t timer0 = NULL, timer1 = NULL;
    mcpwm_timer_config_t timer0_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M, // 160Mhz default clock source
        .resolution_hz = TIMER_RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, //Count up down for symetric waveform to reduce harmonics when driving DC motors
        .period_ticks = COUNTER_PERIOD
    };
    mcpwm_timer_config_t timer1_config = {
        .group_id = 1,
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M, // 160Mhz default clock source
        .resolution_hz = TIMER_RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, //Count up down for symetric waveform to reduce harmonics when driving DC motors
        .period_ticks = COUNTER_PERIOD
    };
    mcpwm_new_timer(&timer0_config, &timer0);
    mcpwm_new_timer(&timer1_config, &timer1);

    // Configure mcpwm operator
    mcpwm_oper_handle_t operator0 = NULL, operator1 = NULL;
    mcpwm_operator_config_t operator0_config = {
        .group_id = 0,
    };
    mcpwm_operator_config_t operator1_config = {
        .group_id = 1,
    };
    mcpwm_new_operator(&operator0_config, &operator0);
    mcpwm_new_operator(&operator1_config, &operator1);
    mcpwm_operator_connect_timer(operator0, timer0);
    mcpwm_operator_connect_timer(operator1, timer1);

    // Configure mcpwm comparator
    // motor1 & motor2 -> operator0
    // motor3 & motor4 -> operator1
    mcpwm_cmpr_handle_t motor1 = NULL, motor2 = NULL, motor3 = NULL, motor4 = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tep = true
    };
    mcpwm_new_comparator(operator0, &comparator_config, &motor1);
    mcpwm_new_comparator(operator0, &comparator_config, &motor2);
    mcpwm_new_comparator(operator1, &comparator_config, &motor3);
    mcpwm_new_comparator(operator1, &comparator_config, &motor4);

    mcpwm_gen_handle_t generator1 = NULL, generator2 = NULL, generator3 = NULL, generator4 = NULL;
    mcpwm_generator_config_t generator1_config = {.gen_gpio_num = MOTOR_1_PWM};
    mcpwm_generator_config_t generator2_config = {.gen_gpio_num = MOTOR_2_PWM};
    mcpwm_generator_config_t generator3_config = {.gen_gpio_num = MOTOR_3_PWM};
    mcpwm_generator_config_t generator4_config = {.gen_gpio_num = MOTOR_4_PWM};

    mcpwm_new_generator(operator0, &generator1_config, &generator1);
    mcpwm_new_generator(operator0, &generator2_config, &generator2);
    mcpwm_new_generator(operator1, &generator3_config, &generator3);
    mcpwm_new_generator(operator1, &generator4_config, &generator4);

    /* 
    Configure the correct wave characteristics to interface with motor driver
    Dual Edge Symmetric Waveform - Active Low (Modified to active high)
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#dual-edge-symmetric-waveform-active-low
    */ 
    mcpwm_generator_set_actions_on_compare_event(generator1,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor1, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor1, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(generator2,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor2, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor2, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(generator3,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor3, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor3, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(generator4,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor4, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, motor4, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    // Set each motor to off initially
    mcpwm_comparator_set_compare_value(motor1, 0);
    mcpwm_comparator_set_compare_value(motor2, 0);
    mcpwm_comparator_set_compare_value(motor3, 0);
    mcpwm_comparator_set_compare_value(motor4, 0);

    mcpwm_timer_enable(timer0);
    mcpwm_timer_enable(timer1);
    mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP);
    mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP);


    // // Setup pulse counter unit
    // pcnt_unit_config_t pcnt_config = {
    //     .low_limit = PCNT_LOW_LIMIT,
    //     .high_limit = PCNT_HIGH_LIMIT,
    //     .flags.accum_count = true
    // };

    // pcnt_unit_handle_t encoder1 = NULL, encoder2 = NULL, encoder3 = NULL, encoder4 = NULL;
    
    // pcnt_new_unit(&pcnt_config, &encoder1);
    // pcnt_new_unit(&pcnt_config, &encoder2);
    // pcnt_new_unit(&pcnt_config, &encoder3);
    // pcnt_new_unit(&pcnt_config, &encoder4);

    // pcnt_unit_handle_t encoder_handle_array[4] = {encoder1, encoder2, encoder3, encoder4};
    
    // // Used a for loop to reduce length
    // // Configure 4 encoder channels
    // for (int i = 0; i < 4; i++)
    // {
    //     // Setup pulse counter channel, each unit has 2 channels
    //     pcnt_chan_config_t channel_a_config = {
    //         .edge_gpio_num = encoder_array[i][0],
    //         .level_gpio_num = encoder_array[i][1]};
    //     pcnt_chan_config_t channel_b_config = {
    //         .edge_gpio_num = encoder_array[i][1],
    //         .level_gpio_num = encoder_array[i][0]};
    //     pcnt_channel_handle_t channel_a = NULL, channel_b = NULL;
        
    //     pcnt_new_channel(encoder_handle_array[i], &channel_a_config, &channel_a);
    //     pcnt_new_channel(encoder_handle_array[i], &channel_b_config, &channel_b);

    //     // Set edge and level actions for pulse counter channels
    //     pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    //     pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    //     pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    //     pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    // }

    // /* Add watch points, PCNT_HIGH_LIMIT and PCNT_LOW_LIMIT which is required to 
    // accumulate count when the counter overflow or underflow 
    // See: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/pcnt.html#compensate-overflow-loss
    // */
    // pcnt_unit_add_watch_point(encoder1, PCNT_HIGH_LIMIT);
    // pcnt_unit_add_watch_point(encoder1, PCNT_LOW_LIMIT);
    // pcnt_unit_add_watch_point(encoder2, PCNT_HIGH_LIMIT);
    // pcnt_unit_add_watch_point(encoder2, PCNT_LOW_LIMIT);
    // pcnt_unit_add_watch_point(encoder3, PCNT_HIGH_LIMIT);
    // pcnt_unit_add_watch_point(encoder3, PCNT_LOW_LIMIT);
    // pcnt_unit_add_watch_point(encoder4, PCNT_HIGH_LIMIT);
    // pcnt_unit_add_watch_point(encoder4, PCNT_LOW_LIMIT);

    // pcnt_unit_enable(encoder1);
    // pcnt_unit_enable(encoder2);
    // pcnt_unit_enable(encoder3);
    // pcnt_unit_enable(encoder4);

    // pcnt_unit_clear_count(encoder1);
    // pcnt_unit_clear_count(encoder2);
    // pcnt_unit_clear_count(encoder3);
    // pcnt_unit_clear_count(encoder4);
    
    // pcnt_unit_start(encoder1);
    // pcnt_unit_start(encoder2);
    // pcnt_unit_start(encoder3);
    // pcnt_unit_start(encoder4);

    // motor pwm comparator, encoder, target_speed, kp, ki, kd, integral_limit_max, integral_limit_min
    initialize_pid_controller(&pid_params[0], motor1, NULL, INIT_TARGET_SPEED_M1, KP_M1, KI_M1, KD_M1, INTEGRAL_LIMIT_MAX, INTEGRAL_LIMIT_MIN);
    initialize_pid_controller(&pid_params[1], motor2, NULL, INIT_TARGET_SPEED_M2, KP_M2, KI_M2, KD_M2, INTEGRAL_LIMIT_MAX, INTEGRAL_LIMIT_MIN);
    initialize_pid_controller(&pid_params[2], motor3, NULL, INIT_TARGET_SPEED_M3, KP_M3, KI_M3, KD_M3, INTEGRAL_LIMIT_MAX, INTEGRAL_LIMIT_MIN);
    initialize_pid_controller(&pid_params[3], motor4, NULL, INIT_TARGET_SPEED_M4, KP_M4, KI_M4, KD_M4, INTEGRAL_LIMIT_MAX, INTEGRAL_LIMIT_MIN);
}
 
void initialize_pid_controller(pid_controller_t *pid_params, mcpwm_cmpr_handle_t motor, pcnt_unit_handle_t encoder, float target_speed, float kp, float ki, float kd, float integral_limit_max, float integral_limit_min){
    pid_params->motor = motor;
    pid_params->encoder = encoder;
    pid_params->target_speed = target_speed;
    pid_params->kp = kp;
    pid_params->ki = ki;
    pid_params->kd = kd;
    pid_params->integral_limit_max = integral_limit_max;
    pid_params->integral_limit_min = integral_limit_min;
};

void print_debug(void*arg){
    pid_controller_t *pid_params = (pid_controller_t *)arg;
    int i = 2;
    while (1)
    {
        // ESP_LOGI(TAG, "%f %f %f %f", pid_params[0].target_speed, pid_params[1].target_speed, pid_params[2].target_speed, pid_params[3].target_speed);
        // vTaskDelay(pdMS_TO_TICKS(500));
        // pcnt_unit_get_count(pid_params[i].encoder, &encoder_pulses[i].curr_count);
        // ESP_LOGI(TAG, "%d", encoder_pulses[i].curr_count);
        // ESP_LOGI(TAG, "Delta Time: %lld", delta_time);
        // ESP_LOGI(TAG, "%d %d %d %d", encoder_pulses[0].curr_count, encoder_pulses[1].curr_count, encoder_pulses[2].curr_count, encoder_pulses[3].curr_count);
        // ESP_LOGI(TAG, "Output: %f, Error: %f, Target Speed: %f, Current Speed: %f, Delta Count: %d, Delta Time: %lld, P Term: %f, I Term: %f, D Term: %f", pid_params[i].output, pid_params[i].err, pid_params[i].target_speed, pid_params[i].curr_speed, encoder_pulses[i].delta_count, delta_time, pid_params[i].pTerm, pid_params[i].iTerm, pid_params[i].dTerm);
    }
    
}