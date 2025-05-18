#include "imu_sampler.h"
#include "imu_bias.h"
#include "mpu6050.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>

#define UART_PORT_NUM UART_NUM_0 // Match uart_task.c

static imu_sample_t sample_buffer[MAX_SAMPLES];
static int sample_index = 0;

static const char *TAG = "IMU_SAMPLER";

// Accessor functions
const imu_sample_t* imu_get_sample_buffer(void) {
    return sample_buffer;
}

int imu_get_sample_count(void) {
    return sample_index;
}

void imu_clear_samples(void) {
    sample_index = 0;
    memset(sample_buffer, 0, sizeof(sample_buffer));
}

static void sampler_task(void *arg) {
    (void)arg; // Unused
    ESP_LOGI(TAG, "Sampling started");
    sample_index = 0;

    while (sample_index < MAX_SAMPLES) {
        mpu6050_raw_data_t raw;
        if (mpu6050_read_raw(&raw) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MPU6050 data");
            vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
            continue;
        }

        imu_sample_t sample;
        sample.ax = raw.accel_x; // - accel_bias[0];
        sample.ay = raw.accel_y; // - accel_bias[1];
        sample.az = raw.accel_z; // - accel_bias[2];
        sample.gx = raw.gyro_x - gyro_bias[0];
        sample.gy = raw.gyro_y - gyro_bias[1];
        sample.gz = raw.gyro_z - gyro_bias[2];
        sample.timestamp_us = esp_timer_get_time();

        sample_buffer[sample_index++] = sample;

        // Debug print
        printf("[%d] AX: %.3f AY: %.3f AZ: %.3f | GX: %.3f GY: %.3f GZ: %.3f\n",
               sample_index, sample.ax, sample.ay, sample.az,
               sample.gx, sample.gy, sample.gz);

        // Send raw imu_sample_t (32 bytes) over UART
        int written = uart_write_bytes(UART_PORT_NUM, &sample, sizeof(imu_sample_t));
        if (written < 0) {
            ESP_LOGE(TAG, "Failed to send sample");
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Avoid UART buffer overflow

        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
    }

    ESP_LOGI(TAG, "Sampling done");
    vTaskDelete(NULL);
}

void imu_sampler_start(void) {
    xTaskCreate(sampler_task, "imu_sampler", 4096, NULL, 5, NULL);
}