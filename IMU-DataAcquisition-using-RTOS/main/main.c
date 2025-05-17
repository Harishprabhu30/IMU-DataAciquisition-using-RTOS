#include <stdio.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

// constants
#define LED_GPIO 10
#define NUM_AXIS 3

// Bias keys for each axis 
const char* accel_keys[NUM_AXIS] = {"accel_x", "accel_y", "accel_z"};
const char* gyro_keys[NUM_AXIS] = {"gyro_x", "gyro_y", "gyro_z"};

// Default Bias Values for each axis {pre calculated bias values}
float accel_bias_deafults[NUM_AXIS] = {0.020, 0.026, 1.031};
float gyro_bias_defaults[NUM_AXIS] = {2.015, 2.952, 0.256};

// these will hold loaded values
float accel_bias[NUM_AXIS];
float gyro_bias[NUM_AXIS];

void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(LED_GPIO, 1); // Turn on the LED
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay for the specified time
        gpio_set_level(LED_GPIO, 0); // Turn off the LED
        vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Delay for the specified time
    }
}

// Function to load or store bias values
esp_err_t load_or_store_bias()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS
    err = nvs_open("imu_storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    bool any_missing = false; // Flag to check if any bias value is missing

    // Load or store each accelerometer bias
    for (int i = 0; i < NUM_AXIS; i++) {
        size_t length = sizeof(float);
        err = nvs_get_blob(nvs_handle, accel_keys[i], &accel_bias[i], &length);
        if (err != ESP_OK) {
            accel_bias[i] = accel_bias_deafults[i]; // Use default value
            nvs_get_blob(nvs_handle, accel_keys[i], &accel_bias[i], &length);
            any_missing = true; // Marks if a value is missing in NVS
        }
    }

    // Load or store each gyroscope bias
    for (int i = 0; i < NUM_AXIS; i++) {
        size_t length = sizeof(float);
        err = nvs_get_blob(nvs_handle, gyro_keys[i], &gyro_bias[i], &length);
        if (err != ESP_OK) {
            gyro_bias[i] = gyro_bias_defaults[i]; // Use default value
            nvs_get_blob(nvs_handle, gyro_keys[i], &gyro_bias[i], &length);
            any_missing = true; // Mark that a value was missing
        }
    }

    // If any value was missing,
    if (any_missing) {
        err = nvs_commit(nvs_handle); // Commit changes
        if (err != ESP_OK) {
            nvs_close(nvs_handle);
            return err; // Return error if commit fails
        }
    }

    // Close NVS
    nvs_close(nvs_handle);
    return ESP_OK; // Return success
}

void app_main(void)
{
    // Configure LED
    gpio_reset_pin(LED_GPIO); // Reset the GPIO pin
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // Set the GPIO pin as output

    // blink 3 times to indicate start
    blink_led(3, 200);

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        printf("Failed to initialize NVS\n");
        blink_led(5, 100); // Blink 5 times to indicate failure
        return;
    }
    
    // Load and store calibration values
    err = load_or_store_bias(); // call the function to load or store bias values

    if (err == ESP_OK) {
        // Sucess
        printf("Bias values loaded successfully:\n");
        printf("IMU BIAS VALUES:\n");
        printf("Accel Bias: X=%.3f, Y=%.3f, Z=%.3f\n",
               accel_bias[0], accel_bias[1], accel_bias[2]);
        printf("Gyro  Bias: X=%.3f, Y=%.3f, Z=%.3f\n",
               gyro_bias[0], gyro_bias[1], gyro_bias[2]);

        // Blink 3 times to indicate success
        blink_led(3, 200);
    } else {
        printf("Error loading bias values: %s\n", esp_err_to_name(err));
        blink_led(5, 100); // Blink 5 times to indicate failure
    }
}

// improvements: Check if IMU is connected before loading bias values and calibrating. (TO BE ADDED in future)