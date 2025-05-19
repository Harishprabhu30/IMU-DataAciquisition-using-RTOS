#ifndef IMU_SAMPLER_H
#define IMU_SAMPLER_H

#include <stdint.h>
#include "esp_err.h"

#define MAX_SAMPLES 3000
#define SAMPLE_RATE 100

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    int64_t timestamp_us;
} imu_sample_t;

const imu_sample_t* imu_get_sample_buffer(void);
int imu_get_sample_count(void);
void imu_clear_samples(void);
void imu_sampler_start(void);
void imu_transfer_start(void);

#endif