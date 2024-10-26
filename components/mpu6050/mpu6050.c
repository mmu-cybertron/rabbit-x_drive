#include "mpu6050.h"

const char *TAG = "MPU6050";
i2c_master_dev_handle_t mpu6050_handle;

int64_t dt = 0, currtime = 0, prevtime = 0;
float calibrationZ = 0; 
float temp = 0;


// List of registers and command to configure mpu6050
uint8_t i2c_data[] = {
    MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_VAL,
    MPU6050_GYRO_CONFIG, MPU6050_GYRO_CONFIG_VAL,
    MPU6050_SMPRT_DIV, MPU6050_SMPRT_DIV_VAL,
    MPU6050_CONFIG, MPU6050_CONFIG_VAL
};

int16_t get_GyroZ_raw(){
    uint8_t data = MPU6050_GYRO_ZOUT_H;
    uint8_t gyroZ_raw[2] = {0};
    i2c_master_transmit_receive(mpu6050_handle, &data, sizeof(data), gyroZ_raw, sizeof(gyroZ_raw), -1);
    int16_t gyroZ_signed = ((gyroZ_raw[0] << 8) | gyroZ_raw[1]);
    // float gyroZ = (float)gyroZ_signed / GYRO_LSB_SENSITIVITY;
    // ESP_LOGI(TAG, "%f", gyroZ);
    return gyroZ_signed;
}

float get_temperature(i2c_master_dev_handle_t mpu6050_handle){
    uint8_t data = MPU6050_TEMP_OUT_H;
    uint8_t temperature_raw[2] = {0};
    i2c_master_transmit_receive(mpu6050_handle, &data, sizeof(data), temperature_raw, sizeof(temperature_raw), -1);
    int16_t temperature_signed = ((temperature_raw[0]<<8) | temperature_raw[1]);
    float temperature_float = (float)temperature_signed / 340.0 + 36.53;
    // ESP_LOGI(TAG, "%f", temperature_float); 
    return temperature_float;
}

void print_debug_mpu6050(void *arg){
    // float* angleZ = (float*)arg;
    while(1){
        // ESP_LOGI(TAG, "%f",  *angleZ);
        // ESP_LOGI(TAG, "Delta Time: %lld", dt);
    }
}

// Used void pointer to make it compatible for esp timer callback function
void mpu6050_updateZ(float* angleZ, int64_t dt){
    int16_t gyroZ = get_GyroZ_raw(mpu6050_handle);
    temp = (((float)gyroZ/GYRO_LSB_SENSITIVITY) - calibrationZ)* (float)dt / 1000000.0;
    // if (temp < 0.07 && temp >- 0.07){
    //     temp = 0;
    // }
    *angleZ = *angleZ + temp;
    // ESP_LOGI(TAG, "%f", calibrationZ);
}

void mpu6050_init(){
    i2c_master_bus_config_t i2c_mst_config = {
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .i2c_port = 0,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false       
    };
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    }; 
    i2c_master_bus_handle_t i2c_mst_handle;
    i2c_new_master_bus(&i2c_mst_config, &i2c_mst_handle);
     
    i2c_master_bus_add_device(i2c_mst_handle, &device_config, &mpu6050_handle);

    // Send everything in i2c_data[]
    for (int i = 0; i < sizeof(i2c_data); i += 2)
    {
        uint8_t data[2] = {i2c_data[i], i2c_data[i + 1]};
        if (i2c_master_transmit(mpu6050_handle, data, sizeof(data), 500) != ESP_OK ){
            ESP_LOGW(TAG, "Failed to initialize MPU6050");
            while(1){
                vTaskDelay(portMAX_DELAY);
            }
        }
    }
    ESP_LOGI(TAG, "MPU6050 Initialized");
    
    ESP_LOGI(TAG, "Calibrating MPU6050, do not move the sensor");
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int i = 0; i<MPU6050_CALIBRATION_SAMPLE_SIZE; i++){
        int16_t gyroZ = get_GyroZ_raw(mpu6050_handle); 
        calibrationZ += (float)gyroZ;
    }
    calibrationZ = (calibrationZ/MPU6050_CALIBRATION_SAMPLE_SIZE)/GYRO_LSB_SENSITIVITY;    
    ESP_LOGI(TAG, "%f", calibrationZ);
    ESP_LOGI(TAG, "MPU6050 Calibration Done");
    prevtime = esp_timer_get_time();

    // xTaskCreatePinnedToCore(print_debug_mpu6050, "print_debug_mpu6050", 2048, NULL, 10, NULL, 1);

};