// imu_sampler.c

#include "imu_sampler.h"
#include "imu_bias.h"
#include "mpu6050.h"
#include "esp_timer.h" // For esp_timer_get_time()
#include "esp_log.h"

//#define ACCEL_SCALE_FACTOR (1.0f / 16384.0f)  // ±2g full scale, 1 LSB = 1/16384 g
//#define GYRO_SCALE_FACTOR  (1.0f / 131.0f)    // ±250 °/s full scale, 1 LSB = 1/131 °/s

static const char *TAG = "IMU_SAMPLER";

static imu_sample_t sample_buffer[MAX_SAMPLES];
static int sample_index = 0;

static void sampler_task(void *arg) {
    ESP_LOGI(TAG, "IMU sampling started for %d seconds", SAMPLE_DURATION_SEC);
    int64_t start_time = esp_timer_get_time();

    while (sample_index < MAX_SAMPLES) {
        mpu6050_raw_data_t raw;
        mpu6050_read_raw(&raw);  // Read raw int16 data

        // Convert raw readings to scaled floats using datasheet scale factors
        float ax = raw.accel_x;
        float ay = raw.accel_y;
        float az = raw.accel_z;

        float gx = raw.gyro_x;
        float gy = raw.gyro_y;
        float gz = raw.gyro_z;

        // Subtract calibrated bias (already scaled floats)
        imu_sample_t sample;
        sample.ax = ax; // - accel_bias[0];
        sample.ay = ay; // - accel_bias[1];
        sample.az = az; // - accel_bias[2];

        sample.gx = gx - gyro_bias[0];
        sample.gy = gy - gyro_bias[1];
        sample.gz = gz - gyro_bias[2];

        sample.timestamp_us = esp_timer_get_time();

        sample_buffer[sample_index++] = sample;

        // Debug serial print
        printf("[%d] AX: %.3f AY: %.3f AZ: %.3f | GX: %.3f GY: %.3f GZ: %.3f\n",
            sample_index,
            sample.ax, sample.ay, sample.az,
            sample.gx, sample.gy, sample.gz);

        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
    }

    ESP_LOGI(TAG, "Sampling done. %d samples collected.", sample_index);

    vTaskDelete(NULL);
}

void imu_sampler_start(void) {
    xTaskCreate(sampler_task, "imu_sampler", 4096, NULL, 5, NULL);
}
