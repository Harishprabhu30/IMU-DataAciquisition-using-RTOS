# pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// constants
#define SAMPLE_RATE 100 // in HZ
#define SAMPLE_DURATION_SEC 30 // in seconds
#define MAX_SAMPLES (SAMPLE_RATE * SAMPLE_DURATION_SEC) // max samples = 100 * 30 = 3000 smaples

typedef struct {
    float ax, ay, az; // accelerometer data
    float gx, gy, gz; // gyroscope data
    int64_t timestamp_us; // timestamp in microseconds
} imu_sample_t;

void imu_sampler_start(void); // Start the IMU sampling task