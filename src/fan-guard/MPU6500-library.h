#ifndef MPU6500_MOTION_H
#define MPU6500_MOTION_H
#include "esp_attr.h"
#include <MPU6500_WE.h>
#include "shared-defs.h"
#include "MPU6500-library.h"
#include "esp_sleep.h" // Required for esp_sleep_enable_ext0_wakeup

// Define the I2C address of the MPU6500.
// This is the default address when AD0 is connected to GND.
#define MPU6500_ADDR 0x68

// Sampling period in milliseconds for baseline calculation and general readings.
#define SAMPLING_PERIOD 10

// Number of samples to take for calculating the baseline acceleration and deviations.
#define BASELINE_SAMPLES 500

// Multiplier applied to the maximum observed baseline deviation to set the motion threshold.
// A higher value makes the motion detection less sensitive.
#define THRESHOLD_MULTIPLIER 2.0

// The GPIO pin connected to the MPU6500's interrupt (INT) pin.
#define INT_PIN 2

// External declaration for the MPU6500 sensor object.
// 'extern' indicates that the actual object is defined in a .cpp file.
extern MPU6500_WE my_mpu6500;

// Volatile boolean flag to indicate if a motion anomaly has been detected.
// 'volatile' ensures that the compiler doesn't optimize access to this variable,
// as it can be modified by an interrupt service routine (ISR).
extern volatile bool motion_anomaly;

// Global variables to store the calculated baseline acceleration values.
// These are external because they are calculated in one function and used potentially elsewhere.
extern float baseline_x, baseline_y, baseline_z;


/**
 * @brief Interrupt Service Routine (ISR) for motion detection.
 *
 * This function is called automatically when the MPU6500's interrupt pin (INT_PIN)
 * goes HIGH, indicating that a motion anomaly has been detected by the sensor.
 * It sets the `motion_anomaly` flag to true.
 *
 * @note IRAM_ATTR is used to place the function in IRAM, which is necessary for ISRs
 * on ESP32 to ensure quick and reliable execution.
 */
void IRAM_ATTR motion_isr();

/**
 * @brief Enables the Wake-on-Motion interrupt on the MPU6500.
 *
 * Configures the MPU6500 to generate an interrupt when motion exceeding
 * the specified threshold is detected. It also attaches the `motion_isr`
 * to the `INT_PIN`.
 *
 * @param threshold The motion detection threshold in MPU6500 units (4mg per unit).
 * Motion above this threshold will trigger an interrupt.
 */
void enable_motion_interrupt(uint8_t threshold);

/**
 * @brief Initializes the MPU6500 sensor.
 *
 * This function attempts to initialize communication with the MPU6500.
 * If successful, it configures various accelerometer settings, including
 * enabling the Digital Low Pass Filter (DLPF), setting the acceleration
 * range, and configuring the interrupt pin behavior for wake-up.
 * It also sets up ESP32's external wake-up source.
 */
void acc_init();

/**
 * @brief Builds the baseline acceleration profile for the MPU6500.
 *
 * This function performs a two-phase process to establish a baseline for motion detection:
 * Phase 1: Calculates the average baseline acceleration (including gravity component)
 * by taking `BASELINE_SAMPLES` readings.
 * Phase 2: Measures the maximum deviation from the calculated baseline during a
 * second set of `BASELINE_SAMPLES` readings.
 *
 * Based on the maximum observed deviation and `THRESHOLD_MULTIPLIER`, it calculates
 * an appropriate motion detection threshold and then calls `enable_motion_interrupt()`
 * to activate the motion interrupt with this threshold.
 * Serial output is provided to show the progress and results of the calibration.
 */
void build_baseline_mpu6500(void *args);

#endif // MPU6500_MOTION_H