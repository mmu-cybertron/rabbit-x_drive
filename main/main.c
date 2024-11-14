/**********************************************************************************************
 * Core 1 is reserved for bluetooth functionality
 * Bluetooth > Bluedroid Options > Core 1
 * Bluetooth > Bluetooth Controller > Core 1
 * Core 0 is running the app_main and used to setup peripherals.
 * In core 0, there is a pulse counting peripheral, with some interrupt footprint, however performance
 * does not seem to be an issue for now.
 * Configure component config > ESP System Settings  > Main Task core affinity to CPU0
 * PS5 Controller Callback function: ~1200ms between each run, 800Hz
 * PID Update function: 1ms between each run, 1kHz
 * MPU6050 Update function: 500us between each run, 2kHz
***********************************************************************************************/
#include "x_drive_esp.h"
#include "ps5.h"
#include "mpu6050.h"
#include "stepper.h"
#include <math.h>

#define PS5_RX_ROTATION_RATE 1500000.0
#define MPU6050_SAMPLING_US 500 // Sample at 500us, 2kHz

#define MAXIMUM_SPEED_COMPARATOR 1999.0 // Maximum speed of robot obtained after testing, in pulse per us
#define MAXIMUM_MAG_PS5 180.0 // Result of sqrt(x*x + y*y) when x and y are at maximum value of 127 or 128

const float comparator_per_ps5 = MAXIMUM_SPEED_COMPARATOR / MAXIMUM_MAG_PS5;

#define MPU_6050_KP 50 //50
#define MPU_6050_KI 0.01 //0.01
#define MPU_6050_KD 0
#define MPU_6050_INTEGRAL_LIMIT_MAX 500
#define MPU_6050_INTEGRAL_LIMIT_MIN -500
#define MPU_6050_TOLERANCE 1

#define PS5_DEADZONE 7

// Change the head of the robot, this value is for rabbit robot only
// DIfferent configuration with different robot
const float phase_shift = 3*3.14159/4;

#define EDGE_BUTTON_PIN GPIO_NUM_27

#define SHOOTER_MOTOR_UP_PWM GPIO_NUM_4
#define SHOOTER_MOTOR_DOWN_PWM GPIO_NUM_23

#define SHOOTER_TIMER_GROUP 1

#define SHOOTER_CLICK_STEP 50

#define PNEUMATIC_PIN GPIO_NUM_26
bool pneumatic_state = false;

int8_t lx;
int8_t ly;
int8_t rx;
int8_t ry;
time_t time_ps5 = {0}; // Used to integrate rx to get target angle
time_t time_mpu6050 = {0}; // For MPU6050 PID loop

// Enable/disable movement rotation
bool move_enabled = false;
bool rotate_enabled = false;
esp_timer_handle_t timer_update_PID_motor = NULL;
esp_timer_handle_t update_PID_mpu6050 = NULL;

// Shooter motor
int shooter_speed = 500; // Default speed of shooter is 500
bool shooter_running = false; // By default shooter is off
mcpwm_cmpr_handle_t shooter_up = NULL, shooter_down = NULL;

// Global variable to store the target angle
float target_angle_Z = 0, angleZ = 0;

// PID parameters for MPU6050
mpu6050_pid_t mpu6050_pid_params = {
    .kp = MPU_6050_KP,
    .ki = MPU_6050_KI,
    .integral_limit_max = MPU_6050_INTEGRAL_LIMIT_MAX,
    .integral_limit_min = MPU_6050_INTEGRAL_LIMIT_MIN,
    .tolerance = MPU_6050_TOLERANCE
};

// Stepper ISR arduments, must declare as global variable
stepper_isr_args_t stepper_isr_args = {0};

// PS5 Controller callback function function prototype
void cb (void *arg, ps5_t ps5, ps5_event_t event);
void connection_cb(uint8_t isConnected);


int64_t start_time = 0, end_time = 0;
TaskHandle_t blink_led_handle = NULL;


void print_debug_1(void*arg){
    pid_controller_t *pid_params = (pid_controller_t *)arg;
    // int i = 2;
    while (1)
    { 
        // ESP_LOGI(TAG, "PS5 CB Time: Delta Time: %lld",  end_time - start_time );
        // ESP_LOGI(TAG, "PS5 CB Time: Delta Time: %lld",  time_ps5.delta_time);
        ESP_LOGI(TAG, "angleZ: %f, target_angle_Z: %f mpu output: %f M1: %f M2: %f M3: %f M4: %f", angleZ, target_angle_Z, mpu6050_pid_params.output, pid_params[0].output, pid_params[1].output, pid_params[2].output, pid_params[3].output);
        // ESP_LOGI(TAG, "Delta Time: %lld, Target Angle: %f AngleZ: %f", pid_params., target_angle_Z, angleZ);
        // ESP_LOGI(TAG, "lx: %d, ly: %d, rx: %d, ry: %d target angle: %f\n", lx, ly, rx, ry, target_angle);
        // ESP_LOGI(TAG, "Motor 1 PID Output: %f, Motor 2 PID Output: %f, Motor 3 PID Output: %f, Motor 4 PID Output: %f", pid_params[0].output, pid_params[1].output, pid_params[2].output, pid_params[3].output); 
        // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to avoid flooding the log
    }
}

void setup_shooter(void){
    gpio_set_direction(EDGE_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(EDGE_BUTTON_PIN, GPIO_PULLUP_ONLY);

    if (gpio_get_level(EDGE_BUTTON_PIN) == 0){
        ESP_LOGI(TAG, "Shooter is in initial position");
        return;
    }

    // If shooter is not in position, setup an interrupt
    // Interrupt callback is from stepper.c
    gpio_set_intr_type(EDGE_BUTTON_PIN, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(EDGE_BUTTON_PIN);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(EDGE_BUTTON_PIN, stepper_stop_pulses, &stepper_isr_args);
    stepper_add_pulses(&stepper_isr_args, 4000);
}

void setup_shooter_motor(mcpwm_cmpr_handle_t *shooter_up_arg, mcpwm_cmpr_handle_t *shooter_down_arg){
    gpio_set_direction(STEPPER_DIR_PIN, GPIO_MODE_OUTPUT);

    // Configure mcpwm timer
    mcpwm_timer_handle_t shooter_timer = NULL;
    mcpwm_timer_config_t shooter_timer_config = {
        .group_id = SHOOTER_TIMER_GROUP, // Use timer 3, as timer 1 & 2 is already used for motor on the base
        .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M, // 160Mhz default clock source
        .resolution_hz = TIMER_RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, //Count up down for symetric waveform to reduce harmonics when driving DC motors
        .period_ticks = COUNTER_PERIOD
    };

    mcpwm_new_timer(&shooter_timer_config, &shooter_timer);

    // Configure mcpwm operator
    mcpwm_oper_handle_t shooter_operator = NULL;
    mcpwm_operator_config_t shooter_operator_config = {
        .group_id = SHOOTER_TIMER_GROUP,
    };
    mcpwm_new_operator(&shooter_operator_config, &shooter_operator);
    mcpwm_operator_connect_timer(shooter_operator, shooter_timer);

    // Configure mcpwm comparator
    mcpwm_cmpr_handle_t shooter_up = NULL, shooter_down = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tep = true
    };
    mcpwm_new_comparator(shooter_operator, &comparator_config, &shooter_up);
    mcpwm_new_comparator(shooter_operator, &comparator_config, &shooter_down);

    // Configure mcpwm generator
    mcpwm_gen_handle_t shooter_down_generator = NULL, shooter_up_generator = NULL; 
    
    mcpwm_generator_config_t shooter_generator_up_config = {.gen_gpio_num = SHOOTER_MOTOR_UP_PWM};
    mcpwm_generator_config_t shooter_generator_down_config = {.gen_gpio_num = SHOOTER_MOTOR_DOWN_PWM};
    
    mcpwm_new_generator(shooter_operator, &shooter_generator_up_config, &shooter_up_generator);
    mcpwm_new_generator(shooter_operator, &shooter_generator_down_config, &shooter_down_generator);

    /* 
    Configure the correct wave characteristics to interface with motor driver
    Dual Edge Symmetric Waveform - Active Low (Modified to active high)
    https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#dual-edge-symmetric-waveform-active-low
    */ 
    mcpwm_generator_set_actions_on_compare_event(shooter_up_generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, shooter_up, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, shooter_up, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());
    mcpwm_generator_set_actions_on_compare_event(shooter_down_generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, shooter_down, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, shooter_down, MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    // Set stepper to off initially
    mcpwm_comparator_set_compare_value(shooter_up, 0);
    mcpwm_comparator_set_compare_value(shooter_down, 0);

    mcpwm_timer_enable(shooter_timer);
    mcpwm_timer_start_stop(shooter_timer, MCPWM_TIMER_START_NO_STOP);

    *shooter_up_arg = shooter_up;
    *shooter_down_arg = shooter_down;
    // mcpwm_comparator_set_compare_value(stepper, STEPPER_MCPWM_COMPRATOR_MIDDLE);
    // stepper_pulses.target_pulses = 1000;
}

void setup_pneumatic_gpio(void){
    gpio_set_direction(PNEUMATIC_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PNEUMATIC_PIN, 0);
}

void blink_led(void *arg){
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    while(1){
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void setup_PS5(void *pid_params){
    // Initialize PS5 controller
    // 24:A6:FA:50:5D:D4
    // uint8_t new_mac[8] = {0x24,0xA6,0xFA,0x3B,0x93,0x0A};
    // uint8_t new_mac[8] = {0x24,0xA6,0xFA,0x50,0x5D,0xD4};
    // ps5SetBluetoothMacAddress(new_mac);
    
    ps5Init();
    while (!ps5IsConnected()) {
        ESP_LOGI("PS5", "Waiting for PS5 controller to connect...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI("PS5", "PS5 controller connected!");
    ps5Enable();
    ps5SetEventObjectCallback(pid_params, cb);

    // Start Printing Debug Messages
    xTaskCreatePinnedToCore(&print_debug_1, "print_debug", 2048, (void*)pid_params, 1, NULL, 1);
    
    // Stop blinking and turn off led
    vTaskDelete(blink_led_handle);
    gpio_set_level(GPIO_NUM_2, 0);

    vTaskDelete(NULL);
}

void app_main(void)
{
    // Declare the struct array, which contains the pwm and pcnt handler
    pid_controller_t pid_params[4] = {0};

    // Timer with 0.5ms periodic task for MPU6050 PID update
    esp_timer_create_args_t update_PID_mpu6050_config ={
        .arg = (void*)&mpu6050_pid_params,
        .callback = &update_PID_mpu6060
    };
    esp_timer_create(&update_PID_mpu6050_config, &update_PID_mpu6050);

    // // Setup timer with a 1ms periodic task to update PID motor speed
    esp_timer_create_args_t timer_update_PID_motor_config ={
        .arg = (void*)pid_params,
        .callback = &update_PID_output
    };
    esp_timer_create(&timer_update_PID_motor_config, &timer_update_PID_motor);

    // Initialize 4 motors using MCPWM, no encoders version
    initialize_peripherals(pid_params); 

    // Start blinking led
    xTaskCreate(&blink_led, "blink_led", 2048, NULL, 1, &blink_led_handle);
    
    // Start PS5 connection in parallel
    xTaskCreatePinnedToCore(&setup_PS5, "setup_PS5", 4096, (void*)pid_params, 2, NULL, 1);

    // Initialize Shooter Motor on GPIO 4 and 23
    setup_shooter_motor(&shooter_up, &shooter_down);

    // Initialize Stepper motors MCPWM and ISR
    init_stepper(&stepper_isr_args);

    // Move the shooter to initial position
    setup_shooter();

    // Setup pneumatic pin
    setup_pneumatic_gpio();

    // If MPU6050 failed to initialize, the function will block further operations
    mpu6050_init();

    vTaskDelay(pdMS_TO_TICKS(1000));
    while(1){
        // mcpwm_comparator_set_compare_value(pid_params[0].motor, 1000);
        // mcpwm_comparator_set_compare_value(pid_params[1].motor, 1000);
        // mcpwm_comparator_set_compare_value(pid_params[2].motor, 1000);
        // mcpwm_comparator_set_compare_value(pid_params[3].motor, 1000);

        // ESP_LOGI(TAG, "Setting Motor DIR to Low");
        // gpio_set_level(MOTOR_1_DIR, 0);
        // gpio_set_level(MOTOR_2_DIR, 0);
        // gpio_set_level(MOTOR_3_DIR, 0);
        // gpio_set_level(MOTOR_4_DIR, 0);
        // vTaskDelay(pdMS_TO_TICKS(2000));

        // ESP_LOGI(TAG, "Setting Motor DIR to High");
        // gpio_set_level(MOTOR_1_DIR, 1);
        // gpio_set_level(MOTOR_2_DIR, 1);
        // gpio_set_level(MOTOR_3_DIR, 1);
        // gpio_set_level(MOTOR_4_DIR, 1);
        // vTaskDelay(pdMS_TO_TICKS(2000));

        // ESP_LOGI(TAG, "Current Speed: Positive");
        // pid_params[2].target_speed = 0.1;   
        // vTaskDelay(pdMS_TO_TICKS(2000));
        // ESP_LOGI(TAG, "Current Speed: Zero");
        // pid_params[2].target_speed = 0; 
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // ESP_LOGI(TAG, "Current Speed: Negative");   
        // pid_params[2].target_speed = -0.1; 
        // vTaskDelay(pdMS_TO_TICKS(2000));

        // mcpwm_comparator_set_compare_value(shooter_up, 500);
        // mcpwm_comparator_set_compare_value(shooter_down, 500);
        
        vTaskDelay(portMAX_DELAY);
    }

}   
// PS5 callback function
// Monitoring delta time shows that this function is running at 800Hz
// Running in core 1
void cb ( void* arg, ps5_t ps5, ps5_event_t event) {
    time_ps5.curr_time = esp_timer_get_time();
    time_ps5.delta_time = time_ps5.curr_time - time_ps5.prev_time;
    time_ps5.prev_time = time_ps5.curr_time;

    pid_controller_t *pid_params = (pid_controller_t *)arg;

    if (event.button_down.up){
        // Shooter angle go up
        // ESP_LOGI(TAG, "Up Button Pressed");
        gpio_set_level(STEPPER_DIR_PIN, 1);
        stepper_add_pulses(&stepper_isr_args, STEPPER_CLICK_STEP);
    }
    if (event.button_down.down){
        // Shooter angle go down
        // ESP_LOGI(TAG, "Up Button Pressed");
        gpio_set_level(STEPPER_DIR_PIN, 0);
        stepper_add_pulses(&stepper_isr_args, STEPPER_CLICK_STEP);
    }
    if (event.button_down.square){
        // Start or stop shooter motor
        // ESP_LOGI(TAG, "Square Button Pressed");
        shooter_running = !shooter_running;
        if (shooter_running){
            mcpwm_comparator_set_compare_value(shooter_up, shooter_speed);
            mcpwm_comparator_set_compare_value(shooter_down, shooter_speed);
        }else{
            mcpwm_comparator_set_compare_value(shooter_up, 0);
            mcpwm_comparator_set_compare_value(shooter_down, 0);
        }
    }
    if (event.button_down.left && shooter_running){
        // Decrease shooter speed
        // ESP_LOGI(TAG, "Left Button Pressed");
        shooter_speed -= SHOOTER_CLICK_STEP ;
        if (shooter_speed < 0) shooter_speed = 0;
        mcpwm_comparator_set_compare_value(shooter_up, shooter_speed);
        mcpwm_comparator_set_compare_value(shooter_down, shooter_speed);
    }
    if (event.button_down.right && shooter_running){
        // Increase shooter speed
        // ESP_LOGI(TAG, "Right Button Pressed");
        shooter_speed += SHOOTER_CLICK_STEP ;
        if (shooter_speed > 1999) shooter_speed = 1999;
        mcpwm_comparator_set_compare_value(shooter_up, shooter_speed);
        mcpwm_comparator_set_compare_value(shooter_down, shooter_speed);
    }
    if (event.button_down.cross){
        // toggle the pneumatic
        // ESP_LOGI(TAG, "Cross Button Pressed");
        if (pneumatic_state){
            // ESP_LOGI(TAG, "Cross Button Low");
            gpio_set_level(PNEUMATIC_PIN, 0);
            pneumatic_state = false;
        }else{
            // ESP_LOGI(TAG, "Cross Button High");
            gpio_set_level(PNEUMATIC_PIN, 1);
            pneumatic_state = true;
        }
    }
    if (event.button_down.l3){
        // Disable/enable robot movement and rotation
        move_enabled = !move_enabled;
        if (move_enabled){
            // Start the timer for motor control
            esp_timer_start_periodic(timer_update_PID_motor, TIMER_PERIOD); // 1ms as sampling time
            gpio_set_level(GPIO_NUM_2, 1);
        }else{
            esp_timer_stop(timer_update_PID_motor);
            esp_timer_stop(update_PID_mpu6050);
            mcpwm_comparator_set_compare_value(pid_params[0].motor, 0);
            mcpwm_comparator_set_compare_value(pid_params[1].motor, 0);
            mcpwm_comparator_set_compare_value(pid_params[2].motor, 0);
            mcpwm_comparator_set_compare_value(pid_params[3].motor, 0);
            gpio_set_level(GPIO_NUM_2, 0);
            pid_params[0].output = 0; 
            pid_params[1].output = 0; 
            pid_params[2].output = 0; 
            pid_params[3].output = 0; 
            target_angle_Z = 0;
            angleZ = 0;
            mpu6050_pid_params.integral = 0;
            mpu6050_pid_params.prev_err = 0;
            mpu6050_pid_params.pTerm = 0;
            mpu6050_pid_params.iTerm = 0;
            mpu6050_pid_params.dTerm = 0;
            mpu6050_pid_params.err = 0;
            mpu6050_pid_params.output = 0;
            mpu6050_pid_params.inPosition = 0;
        }
    }
    if (event.button_down.r3){
        // Disable/enable robot rotation only
        rotate_enabled = !rotate_enabled;
        if (rotate_enabled){
            // Start the timer for MPU6050 PID control
            esp_timer_start_periodic(update_PID_mpu6050, MPU6050_SAMPLING_US);
        }else{
            // Reset target angle and PID parameters  
            esp_timer_stop(update_PID_mpu6050);
            // mcpwm_comparator_set_compare_value(pid_params[0].motor, 0);
            // mcpwm_comparator_set_compare_value(pid_params[1].motor, 0);
            // mcpwm_comparator_set_compare_value(pid_params[2].motor, 0);
            // mcpwm_comparator_set_compare_value(pid_params[3].motor, 0);
            target_angle_Z = 0;
            angleZ = 0;
            mpu6050_pid_params.integral = 0;
            mpu6050_pid_params.prev_err = 0;
            mpu6050_pid_params.pTerm = 0;
            mpu6050_pid_params.iTerm = 0;
            mpu6050_pid_params.dTerm = 0;
            mpu6050_pid_params.err = 0;
            mpu6050_pid_params.output = 0;
            mpu6050_pid_params.inPosition = 0;

        }
    }

    lx = ps5.analog.stick.lx;
    ly = ps5.analog.stick.ly;
    rx = ps5.analog.stick.rx;
    ry = ps5.analog.stick.ry;

    // Remove deadzone of the controller
    if (lx <= PS5_DEADZONE && lx >= -PS5_DEADZONE) lx = 0;
    if (ly <= PS5_DEADZONE && ly >= -PS5_DEADZONE) ly = 0;
    if (rx <= PS5_DEADZONE && rx >= -PS5_DEADZONE) rx = 0;
    if (ry <= PS5_DEADZONE && ry >= -PS5_DEADZONE) ry = 0;
    
    end_time = esp_timer_get_time();

    // Integrate the right stick to get the target angle
    // This line is computational expensive, need to optimize
    if (rotate_enabled && move_enabled){
        target_angle_Z += (float)-rx * (float)time_ps5.delta_time / PS5_RX_ROTATION_RATE;
    }
    
    float theta =  atan2f((float)-ly, (float)-lx);
    float mag = sqrtf((float)(lx*lx + ly*ly));

    float M1 = mag * cosf(theta - phase_shift);
    float M2 = mag * -sinf(theta - phase_shift);
    float M3 = mag * -cosf(theta - phase_shift);
    float M4 = mag * sinf(theta - phase_shift);
    
    M1 = M1 * comparator_per_ps5;
    M2 = M2 * comparator_per_ps5;
    M3 = M3 * comparator_per_ps5;
    M4 = M4 * comparator_per_ps5;

    pid_params[0].target_speed = M1;
    pid_params[1].target_speed = M2;
    pid_params[2].target_speed = M3;
    pid_params[3].target_speed = M4;


    // This function will run on core 1
    // int x = esp_cpu_get_core_id();
    // ESP_LOGI(TAG, "Core ID: %d", x);    

}

void update_PID_mpu6060(void *arg){
    mpu6050_pid_t *mpu6050_pid_params = (mpu6050_pid_t *)arg;
    time_mpu6050.curr_time = esp_timer_get_time();
    time_mpu6050.delta_time = time_mpu6050.curr_time - time_mpu6050.prev_time;
    time_mpu6050.prev_time = time_mpu6050.curr_time;

    mpu6050_updateZ(&angleZ, time_mpu6050.delta_time);

    mpu6050_pid_params->err = target_angle_Z - angleZ;

    // Calculate P term
    mpu6050_pid_params->pTerm = mpu6050_pid_params->kp * mpu6050_pid_params->err;

    // Calculate I term
    mpu6050_pid_params->integral += mpu6050_pid_params->err;
    mpu6050_pid_params->iTerm = mpu6050_pid_params->ki * mpu6050_pid_params->integral;
    if (mpu6050_pid_params->iTerm > mpu6050_pid_params->integral_limit_max)
    {
        mpu6050_pid_params->iTerm = mpu6050_pid_params->integral_limit_max;
    }
    else if (mpu6050_pid_params->iTerm < mpu6050_pid_params->integral_limit_min)
    {
        mpu6050_pid_params->iTerm = mpu6050_pid_params->integral_limit_min;
    }

    // Derivative not implemented
    
    // Calculate output and if it is within tolerance, set output to 0
    if (fabs(mpu6050_pid_params->err) < mpu6050_pid_params->tolerance){
        mpu6050_pid_params->output = 0;
    } else {
        mpu6050_pid_params->output = mpu6050_pid_params->pTerm + mpu6050_pid_params->iTerm;
    }

    // ESP_LOGI(TAG, "AngleZ: %f, Delta Time: %lld", angleZ, time_mpu6050.delta_time);

}

// Periodic timer callback to update PID output
void update_PID_output(void *arg){

    pid_controller_t *pid_params = (pid_controller_t *)arg;  // Cast void* to pid_controller_t*

    pid_params[0].output = pid_params[0].target_speed + mpu6050_pid_params.output;
    pid_params[1].output = pid_params[1].target_speed + mpu6050_pid_params.output;
    pid_params[2].output = pid_params[2].target_speed + mpu6050_pid_params.output;
    pid_params[3].output = pid_params[3].target_speed + mpu6050_pid_params.output;

    float largest_speed = fabs(pid_params[0].output);
    for (int i = 1; i < 4; i++) {
        if (fabs(pid_params[i].output) > largest_speed) {
            largest_speed = fabs(pid_params[i].output);
        }
    }
    // Normalize the speed to the maximum speed
    if (largest_speed > MAXIMUM_SPEED_COMPARATOR) {
        for (int i = 0; i < 4; i++) {
            pid_params[i].output = pid_params[i].output / largest_speed * MAXIMUM_SPEED_COMPARATOR;
        }
    }
    // ESP_LOGI(TAG, "Triggered");
    for(int i=0; i<4; i++){

        // Limit the ouput to a range within valid comparator value
        if (pid_params[i].output > MCPWM_COMPRATOR_MAX) {
            ESP_LOGI(TAG, "Triggered: %f", pid_params[i].output);
            pid_params[i].output = MCPWM_COMPRATOR_MAX;
        } else if (pid_params[i].output < -MCPWM_COMPRATOR_MAX) {
            ESP_LOGI(TAG, "Triggered: %f", pid_params[i].output);
            pid_params[i].output = -MCPWM_COMPRATOR_MAX;
        }
        // Set direction of motor
        if (pid_params[i].output>0) gpio_set_level(motor_dir_array[i], 0);
        else gpio_set_level(motor_dir_array[i], 1);

        // Update new PWM duty cycle
        uint32_t comp_val = fabs(pid_params[i].output);
        mcpwm_comparator_set_compare_value(pid_params[i].motor, comp_val);
    }

    // Print output, error and current speed
    // ESP_LOGI(TAG, "Output: %d, Error: %d, Current Speed: %d", output, err, curr_speed);
    // Print print delta count, delta time
    // ESP_LOGI(TAG, "Delta Count: %d, Delta Time: %lld", delta_count, delta_time);
}