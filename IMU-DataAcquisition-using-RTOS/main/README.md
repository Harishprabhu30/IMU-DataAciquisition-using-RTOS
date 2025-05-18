code structure to be written in main app:

// 1. Blink LED 3 times on boot
// 2. Initialize NVS
// 3. Check if bias exists in NVS
//    a. If exists, load and print
//    b. If not, store default bias values you gave above
// 4. Use the loaded bias for calibration logic
// 5. If bias successfully applied → blink 3 times (success)
//    If failure → blink 5 times (failure)
// 6. Print all biases using printf for debugging

code structure for imu_sampler

├── imu_sampler.c      ← sampling task, bias correction, data storage
├── imu_sampler.h      ← header with config, structs, prototypes
├── imu_bias.h         ← externs for accel/gyro bias values

1. Creating imu_sampler.h and it contains
- Structs for storing samples

- Buffer definitions

- Function prototypes

2. Adding bias.h : loads the calibrated values
3. Add imu_sampler.c and it does:
- Reads IMU

- Applies bias correction

- Stores to buffer

- Optionally prints for debugging

# ADded wifi task (but not using for this project for now)

To be added in main.c
#include "wifi_task.h"

void app_main(void) {
    wifi_task_init();  // Blocks until connected
    // Now safe to start HTTP or other tasks that require internet
}
