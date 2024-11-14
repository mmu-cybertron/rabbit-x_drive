#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c_master.h"
#include <driver/gpio.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define I2C_MASTER_FREQ_HZ 400000
#define GYRO_LSB_SENSITIVITY 131000000.0 // 131 (from datasheet) * 1000000 (microsend per second)
#define MPU6050_CALIBRATION_SAMPLE_SIZE 2000
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22

#define MPU6050_ADDR 0x68

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_1_VAL 0x00

#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_CONFIG_VAL 0x00

#define MPU6050_SMPRT_DIV 0X19
#define MPU6050_SMPRT_DIV_VAL 0x00

#define MPU6050_CONFIG 0x1A
#define MPU6050_CONFIG_VAL 0x00

#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_GYRO_ZOUT_H 0x47

typedef struct{
    float target_speed;
    float kp;
    float ki;
    float kd;
    float integral_limit_max;
    float integral_limit_min;
    float tolerance;
    // The items need not to be configured during initialization
    float integral;
    float prev_err;
    float pTerm;
    float iTerm; 
    float dTerm; 
    float err; 
    float output;
    bool inPosition;
}mpu6050_pid_t;

// Public Functions
void mpu6050_init();
void mpu6050_updateZ(float* angleZ, int64_t dt); // void pointer is used to make it compatible with esp_timer callback function

// Private Functions
void update_PID_mpu6060(void *arg);

#endif
