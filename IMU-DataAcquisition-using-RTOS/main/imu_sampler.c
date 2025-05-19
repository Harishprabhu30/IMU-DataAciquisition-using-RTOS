#include "imu_sampler.h"
#include "imu_bias.h"
#include "mpu6050.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define TAG "IMU_SAMPLER"

static imu_sample_t sample_buffer[MAX_SAMPLES];
static int sample_index = 0;
static bool has_sampled = false;

const imu_sample_t* imu_get_sample_buffer(void) {
    return sample_buffer;
}

int imu_get_sample_count(void) {
    return sample_index;
}

// Note: Do NOT clear the buffer unless needed for development
void imu_clear_samples(void) {
    sample_index = 0;
    memset(sample_buffer, 0, sizeof(sample_buffer));
}

static void sampler_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Sampling started");
    sample_index = 0;

    for (int i = 0; i < MAX_SAMPLES; i++) {
        mpu6050_raw_data_t raw;
        if (mpu6050_read_raw(&raw) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MPU6050 data");
            vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
            continue;
        }

        imu_sample_t sample;
        sample.ax = raw.accel_x; // No bias correction
        sample.ay = raw.accel_y;
        sample.az = raw.accel_z;
        sample.gx = raw.gyro_x - gyro_bias[0];
        sample.gy = raw.gyro_y - gyro_bias[1];
        sample.gz = raw.gyro_z - gyro_bias[2];
        sample.timestamp_us = esp_timer_get_time();

        ESP_LOGI(TAG, "%d: %.3f %.3f %.3f %.3f %.3f %.3f %lld",
                 i, sample.ax, sample.ay, sample.az,
                 sample.gx, sample.gy, sample.gz, sample.timestamp_us);

        sample_buffer[sample_index++] = sample;
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_RATE));
    }

    ESP_LOGI(TAG, "Sampling done, starting transfer task");
    imu_transfer_start();
    vTaskDelete(NULL);
}

static void transfer_task(void *arg) {
    (void)arg;

    ESP_LOGI(TAG, "Transfer started, sending %d samples to UART", sample_index);

    // Optional: timestamp debug for buffer age
    if (sample_index > 0) {
        ESP_LOGI(TAG, "First timestamp: %lld", sample_buffer[0].timestamp_us);
        ESP_LOGI(TAG, "Last timestamp: %lld", sample_buffer[sample_index - 1].timestamp_us);
    }

    for (int i = 0; i < sample_index; i++) {
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lld\n",
               sample_buffer[i].ax, sample_buffer[i].ay, sample_buffer[i].az,
               sample_buffer[i].gx, sample_buffer[i].gy, sample_buffer[i].gz,
               sample_buffer[i].timestamp_us);
        vTaskDelay(pdMS_TO_TICKS(50)); // Prevent UART congestion
    }

    ESP_LOGI(TAG, "Transfer complete. Data preserved until reboot.");
    vTaskDelete(NULL);
}

void imu_sampler_start(void) {
    if (has_sampled) {
        ESP_LOGW(TAG, "Sampling already completed. Restart the device to sample again.");
        return;
    }
    has_sampled = true;
    xTaskCreate(sampler_task, "imu_sampler", 4096, NULL, 5, NULL);
}

void imu_transfer_start(void) {
    xTaskCreate(transfer_task, "imu_transfer", 4096, NULL, 4, NULL);
}
