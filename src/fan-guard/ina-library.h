#ifndef INA_ANOMALY_H
#define INA_ANOMALY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "ina-library.h"
#include "shared-defs.h"

// External declaration for the INA219 sensor object.
// 'extern' indicates that the actual object is defined in a .cpp file.
extern Adafruit_INA219 ina219;

// Filtering parameters
// Number of samples for averaging readings.
#define SAMPLE_COUNT 20
// Alpha value for the low-pass filter, determines smoothing strength.
#define ALPHA 0.5
// Delay between readings in milliseconds.
#define READING_DELAY 20
// Threshold for spike filtering, represents the maximum acceptable change between readings.
#define SPIKE_THRESHOLD 0.7

// Baseline and anomaly detection parameters
// Number of samples to establish the baseline for power consumption.
#define BASELINE_SAMPLES 100
// Standard deviations from the mean to define an anomaly.
#define ANOMALY_THRESHOLD 2.3

#define PERIODIC_CHECK_SAMPLES 50

struct INA_DATA{
  float fl_power;
  float fl_voltage;
  float fl_current;
};

typedef INA_DATA ina_data_t;
extern ina_data_t ina_samples[PERIODIC_CHECK_SAMPLES];

// Variables for filtering
// Filtered bus voltage in Volts.
extern float filtered_bus_voltage;
// Filtered shunt voltage in millivolts.
extern float filtered_shunt_voltage;
// Filtered current in milliamperes.
extern float filtered_current;
// Filtered power in milliwatts.
extern float filtered_power;
// Filtered load voltage in Volts.
extern float filtered_load_voltage;
// Previous bus voltage reading for spike filtering.
extern float prev_bus_voltage;
// Previous current reading for spike filtering.
extern float prev_current;

// Variables for baseline and anomaly detection
// Mean (average) power consumption during the baseline period.
extern RTC_DATA_ATTR float baseline_mean;
// Standard deviation of power consumption during the baseline period.
extern RTC_DATA_ATTR float baseline_std;
// Array to store power readings during baseline establishment.
extern float power_readings[BASELINE_SAMPLES];
// Total number of readings taken (can be used for debugging or dynamic baseline).
extern RTC_DATA_ATTR int total_readings;
// Flag to indicate if it's the first reading for filter initialization.
extern bool first_reading; // Assuming this is defined elsewhere or needs to be added

// Variable to store the maximum deviation observed during anomaly detection.
extern RTC_DATA_ATTR float max_deviation; // Assuming this is defined elsewhere or needs to be added

/**
 * @brief Initializes the INA219 power sensor.
 *
 * This function sets up serial communication, initializes the INA219 sensor,
 * and configures its calibration. It also takes initial readings to
 * prime the filtering variables.
 */
void ina219_init();
DataClassification classify_ina(ina_data_t *samples);

void send_ina_anomaly_mqtt(bool ina_check_classification,ina_data_t *ina_samples);
void ina_periodic_check(void *args);
/**
 * @brief Gets an averaged reading from a sensor function.
 *
 * This function repeatedly calls a provided sensor reading function
 * for a specified number of samples and returns their average.
 *
 * @param read_function A pointer to the function that returns a single sensor reading.
 * @param samples The number of samples to average.
 * @return The averaged sensor reading.
 */
float get_averaged_reading(float (*read_function)(), int samples);

/**
 * @brief Retrieves the bus voltage from the INA219.
 * @return The bus voltage in Volts.
 */
float get_bus_voltage();

/**
 * @brief Retrieves the shunt voltage from the INA219.
 * @return The shunt voltage in millivolts.
 */
float get_shunt_voltage();

/**
 * @brief Retrieves the current from the INA219.
 * @return The current in milliamperes.
 */
float get_current();

/**
 * @brief Retrieves the power from the INA219.
 * @return The power in milliwatts.
 */
float get_power();

/**
 * @brief Applies a spike filter to a new reading.
 *
 * If the `newValue` deviates from `prevValue` by more than `threshold`,
 * `prevValue` is returned to filter out sudden spikes. Otherwise, `newValue` is returned.
 *
 * @param new_value The current sensor reading.
 * @param prev_value The previous sensor reading.
 * @param threshold The maximum allowed deviation from the previous value.
 * @return The filtered value (either `new_value` or `prev_value`).
 */
float spike_filter(float new_value, float prev_value, float threshold);

/**
 * @brief Applies a low-pass filter to a new reading.
 *
 * This filter smooths out readings by combining the new value with the
 * previously filtered value, weighted by `alpha`.
 *
 * @param new_value The current raw sensor reading.
 * @param prev_filtered The previously filtered value.
 * @param alpha The smoothing factor (0.0 to 1.0). Higher alpha means less smoothing.
 * @return The new filtered value.
 */
float low_pass_filter(float new_value, float prev_filtered, float alpha);

/**
 * @brief Reads and filters the power consumption from the INA219.
 *
 * This function obtains raw readings, applies spike filtering, and then
 * applies a low-pass filter to provide a smoothed power reading.
 *
 * @return The filtered power reading in milliwatts.
 */
ina_data_t read_ina_filtered();

/**
 * @brief Calculates the mean (average) of an array of float values.
 *
 * @param array A pointer to the float array.
 * @param size The number of elements in the array.
 * @return The calculated mean.
 */
float calculate_mean(float* array, int size);

/**
 * @brief Calculates the standard deviation of an array of float values.
 *
 * @param array A pointer to the float array.
 * @param size The number of elements in the array.
 * @param mean The pre-calculated mean of the array.
 * @return The calculated standard deviation.
 */
float calculate_std_dev(float* array, int size, float mean);

/**
 * @brief Detects anomalies based on a threshold derived from baseline statistics.
 *
 * Checks if a given `reading` deviates significantly from the `baseline_mean`,
 * where "significantly" is defined by `ANOMALY_THRESHOLD` times the `baseline_std`.
 * It also tracks the maximum deviation observed.
 *
 * @param reading The current power reading to check for anomaly.
 * @return True if an anomaly is detected, false otherwise.
 */
bool detect_threshold_anomaly(float reading);

/**
 * @brief Builds the baseline profile for power consumption using the INA219.
 *
 * This function takes a series of power readings (`BASELINE_SAMPLES`),
 * calculates their mean and standard deviation, and then sets up the
 * parameters for subsequent anomaly detection. It also prints calibration progress
 * and final baseline statistics to the serial monitor.
 */
void build_baseline_ina(void *args);
#endif // INA_ANOMALY_H
