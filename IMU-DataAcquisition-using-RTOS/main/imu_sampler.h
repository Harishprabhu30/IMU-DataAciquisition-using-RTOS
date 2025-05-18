#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Constants
#define SAMPLE_RATE 100 // in Hz
#define SAMPLE_DURATION_SEC 30 // in seconds
#define MAX_SAMPLES (SAMPLE_RATE * SAMPLE_DURATION_SEC) // 3000 samples

typedef struct {
    float ax, ay, az; // accelerometer data (in g)
    float gx, gy, gz; // gyroscope data (in deg/s)
    int64_t timestamp_us; // timestamp in microseconds
} imu_sample_t;

void imu_sampler_start(void); // Start IMU sampling
const imu_sample_t* imu_get_sample_buffer(void); // Get sample buffer pointer
int imu_get_sample_count(void); // Get sample count
void imu_clear_samples(void); // Clear sample buffer