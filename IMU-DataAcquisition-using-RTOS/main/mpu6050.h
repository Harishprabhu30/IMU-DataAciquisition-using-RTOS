#pragma once

#include "esp_err.h"

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_raw_data_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_raw(mpu6050_raw_data_t *data);
