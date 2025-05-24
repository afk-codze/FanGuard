#ifndef ANOMALY_DETECTION_H
#define ANOMALY_DETECTION_H

#include "esp_wifi.h"       // For Wi-Fi functionalities (if used for MQTT)
#include <Arduino.h>        // Standard Arduino core functions
#include <math.h>           // For mathematical operations (e.g., sqrt, abs)
#include "freertos/FreeRTOS.h" // FreeRTOS kernel definitions
#include "freertos/task.h"  // FreeRTOS task management
#include "driver/uart.h"    // UART driver (if used for debugging/comm)
#include "esp_sleep.h"      // ESP-IDF deep sleep functions
#include "shared-defs.h"    // Custom shared definitions (e.g., ina_data_t)
#include "ina-library.h"    // Library for INA219 sensor functions
#include "MPU6500-library.h" // Library for MPU6500 sensor functions

// --- Constants ---
// FFT sampling rate in Hz. This defines the number of samples collected
// for both motion and INA data within a specific period.
#define SAMPLE_RATE 1000

// --- Function Prototypes ---

/**
 * @brief Classifies if the collected sensor data indicates an anomaly.
 *
 * This function takes arrays of motion and INA data samples and
 * performs an analysis to determine if an anomaly is present based on
 * predefined criteria or machine learning models.
 *
 * @param motion_samples A pointer to an array of xyzFloat structures
 * containing motion sensor readings (e.g., accelerometer data).
 * @param ina_samples A pointer to an array of ina_data_t structures
 * containing INA sensor readings (e.g., power, current, voltage).
 * @return True if an anomaly is classified, false otherwise.
 */
bool classify_anomaly(xyzFloat *motion_samples, ina_data_t *ina_samples);

/**
 * @brief Sends anomaly notification via MQTT.
 *
 * This function publishes a message to an MQTT broker indicating whether
 * an anomaly was detected and can include the relevant sensor data.
 *
 * @param anomaly_class A boolean indicating the anomaly classification (true for anomaly).
 * @param motion_values A pointer to an array of float values representing motion data.
 * (Note: The original code passed float*, but motion_samples is xyzFloat*.
 * Ensure type consistency or adapt this parameter as needed.)
 * @param ina_samples A pointer to an array of float values representing INA data.
 * (Note: The original code passed float*, but ina_samples is ina_data_t*.
 * Ensure type consistency or adapt this parameter as needed.)
 */
void send_anomaly_mqtt(bool anomaly_class, xyzFloat *motion_samples, ina_data_t *ina_samples);

/**
 * @brief Performs high-frequency sampling of motion and INA data.
 *
 * This function collects `SAMPLE_RATE` number of readings from both
 * the MPU6500 motion sensor and the INA219 power sensor at a rate
 * determined by `SAMPLE_RATE`. After collection, it calls `classify_anomaly`
 * and, if an anomaly is detected, `send_anomaly_mqtt`.
 */
void high_freq_sampling(void *args);

#endif // ANOMALY_DETECTION_H