#include <a1_inferencing.h>
#include <Adafruit_Sensor.h>
#include "fft-and-adaptive-sampling.h"

// Anomaly flags
RTC_DATA_ATTR bool anomaly_detected = false;
RTC_DATA_ATTR bool anomaly_sent = false;
RTC_DATA_ATTR data_to_send_t anomaly;

// FFT initialization phase flag
volatile RTC_DATA_ATTR bool fft_init_complete = false;

// Current system sampling frequency (Hz)
RTC_DATA_ATTR int g_sampling_frequency = INIT_SAMPLE_RATE;

// FFT samples
float samples_real[INIT_SAMPLE_RATE];
float samples_imag[INIT_SAMPLE_RATE];

// window of RMS
RTC_DATA_ATTR int g_window_size = INIT_SAMPLE_RATE * SESSION_DURATION_SEC;
RTC_DATA_ATTR int num_of_samples = 0;

// counter for numeric series of squared samples
RTC_DATA_ATTR float session_sum_sq[3] = {0.0f, 0.0f, 0.0f};

// ArduinoFFT instance configured with buffers
ArduinoFFT<float> FFT = ArduinoFFT<float>(samples_real, samples_imag, INIT_SAMPLE_RATE, INIT_SAMPLE_RATE);

float features[3] = {0};

void send_data(data_to_send_t rms_data) {
  // Send to queue
  if(xQueue_data == NULL)
    Serial.print("rms is NULL");
  xQueueSend(xQueue_data, &rms_data, 0);
  Serial.printf("send data with timestamp: %ul",rms_data.time_stamp);
}

void send_anomaly_task(void *pvParameters){
  if(anomaly_detected && !anomaly_sent){
    send_data(anomaly);
    anomaly_sent = true;
  }
  vTaskDelete(NULL);
}

void read_sample(float* x, float* y, float* z) {
  sensors_event_t event;
  accel.getEvent(&event);
  // Convert m/s² → g (1 g = 9.80665 m/s²)
  *x = event.acceleration.x / 9.80665;
  *y = event.acceleration.y / 9.80665;
  *z = event.acceleration.z / 9.80665;
}

void light_sleep(int duration) {
  //uart_wait_tx_idle_polling((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM);
  esp_sleep_enable_timer_wakeup(1000*1000*duration);
  esp_light_sleep_start();
}

void deep_sleep(int duration) {
  //uart_wait_tx_idle_polling((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM);
  esp_sleep_enable_timer_wakeup(1000*1000*duration);
  esp_deep_sleep_start();
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = features[offset + i];
  }
  return 0;
}

bool anomaly_detection(float rms_x,float rms_y,float rms_z) {
  Serial.printf("[MODEL] RMS values for model input - X: %.3f, Y: %.3f, Z: %.3f\n", rms_x, rms_y, rms_z);

   // ----- Update Global Features Array -----
  features[0] = rms_x;
  features[1] = rms_y;
  features[2] = rms_z;

  // ----- Prepare the Signal Object for Inference -----
  // Following the documentation, the features are provided via a signal_t structure.
  signal_t features_signal;
  features_signal.total_length = 3;
  features_signal.get_data = &raw_feature_get_data;

  ei_impulse_result_t result = {0};

  // Run the classifier
  EI_IMPULSE_ERROR res = run_classifier(&features_signal,&result, false);

  ei_printf("Predictions:\r\n");
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
    ei_printf("%.5f\r\n", result.classification[i].value);
  }

  if (res != EI_IMPULSE_OK) {
    ei_printf("[ERROR] Failed to run Edge Impulse classifier");
    return false;
  }

  if (result.classification[0].value > 0.5) {
    return true;
  }

  return false;
}

/* FFT Processing Core ----------------------------------------------------- */
/**
 * @brief Execute complete FFT processing chain
 * @details Performs:
 * 1. Hamming window application
 * 2. Forward FFT computation
 * 3. Complex-to-magnitude conversion
 * @note Results stored in module buffers
 */
void fft_perform_analysis(void) {
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
}

/**
 * @brief Identify max frequency component
 * @return Frequency (Hz) of highest frequency
 * @pre Requires prior call to fft_perform_analysis()
 */
float fft_get_max_frequency(void) {
  float maxFrequency = -1;

  // Loop through all bins (skip DC at i=0)
  for (uint16_t i = 1; i < (INIT_SAMPLE_RATE >> 1); i++) {
    // Check if the current bin is a local maximum and above the noise floor
    if (samples_real[i] > samples_real[i-1] && samples_real[i] > samples_real[i+1] && samples_real[i] > NOISE_THRESHOLD) {
      float currentFreq = (i * g_sampling_frequency) / INIT_SAMPLE_RATE;
      // Update maxFrequency if this peak has a higher frequency
      if (currentFreq > maxFrequency) {
        maxFrequency = currentFreq;
      }
    }
  }
  return maxFrequency;
}

void fft_sample_signal() {
  float sample[3] = {0,0,0};
  // Sample the signal for initial FFT analysis
  for (int i = 0; i < INIT_SAMPLE_RATE; i++) {
    read_sample(&sample[0], &sample[1], &sample[2]);
    samples_real[i] = sample[0]+sample[1]+sample[2];
    light_sleep(1000 / g_sampling_frequency);
  }
}


/* System Configuration ---------------------------------------------------- */
/**
 * @brief Adapt sampling rate based on Nyquist-Shannon criteria
 * @param max_freq Max detected frequency component
 * @note Implements safety factor of 2.5× maximum frequency
 */
 void fft_adjust_sampling_rate(float max_freq) {
     int new_rate = (int)(NYQUIST_MULTIPLIER * max_freq);
     if (new_rate < 1) new_rate = INIT_SAMPLE_RATE / 2; // Fallback if no valid frequency detected
     g_sampling_frequency = (g_sampling_frequency > new_rate) ? new_rate : g_sampling_frequency;
 }

/**
 * @brief Initialize FFT processing module
 * @details Performs:
 * 1. Initial signal acquisition
 * 2. Frequency analysis
 * 3. Adaptive rate configuration
 * @note Must be called before starting sampling tasks
 */
void fft_init() {
  Serial.println("[FFT] Initializing FFT module");

  if (!fft_init_complete) {

    // Initial analysis with high frequency sampling
    g_sampling_frequency = INIT_SAMPLE_RATE;
    fft_sample_signal();

    fft_perform_analysis();

    float max_freq = fft_get_max_frequency();

    if (max_freq > 0) {
      Serial.printf("[FFT] Max frequency: %.2f Hz\n", max_freq);
    } else {
      //this might happen if the fan is turned off
      Serial.printf("[ERROR] No valid peaks detected\n");
      return;
    }

    // Adaptive rate adjustment
    fft_adjust_sampling_rate(max_freq);
    Serial.printf("[FFT] Optimal sampling rate: %d Hz\n", g_sampling_frequency);

    g_window_size = g_sampling_frequency * SESSION_DURATION_SEC;
    Serial.printf("[FFT] Window size (30 s): %d samples\n", g_window_size);

    // Mark fft initialization as complete
    fft_init_complete = true;
  }
}

/**
 * @brief Main sampling task handler
 * @param pvParameters FreeRTOS task parameters (unused)
 * @details
 *
 * @warning Depends on initialized queue (xQueue_temp and xQueue_temp_avg)
 */
void fft_sampling_task(void *pvParameters) {
  float sample[3] = {0,0,0};
  float rms_array[3] = {0,0,0};
  data_to_send_t data_to_send;

  unsigned long time_stamp = 0;

  if(!fft_init_complete)
    fft_init();
  Serial.printf("[FFT] Starting sampling at %d Hz\n", g_sampling_frequency);
  Serial.println("--------------------------------");

  while(1) {
    // Read sample from sensor
    read_sample(&sample[0], &sample[1], &sample[2]);
    unsigned long cycle_start_ms = millis();

    Serial.printf("[FFT] Sample %d = x: %.2f, y: %.2f, z: %.2f\n",num_of_samples, sample[0], sample[1], sample[2]);
    Serial.printf("[WINDOW] g_window_size %d", g_window_size);

    time_stamp = micros();

    //accumulate squared sum samples
    session_sum_sq[0] += sample[0] * sample[0];
    session_sum_sq[1] += sample[1] * sample[1];
    session_sum_sq[2] += sample[2] * sample[2];

    // set flag this_reboot_send_rms = true if next reboot we send data to mqtt
    if(((num_of_samples + 1) >= (g_window_size)) == 0)
      this_reboot_send_rms = true; //enables mqtt task 

    // Send RMS values periodically
    if (num_of_samples >= (g_window_size)) {
      
      float inv_window = 1.0f / g_window_size;

      rms_array[0] = sqrtf(session_sum_sq[0] * inv_window); // may use a separate function
      rms_array[1] = sqrtf(session_sum_sq[1] * inv_window);
      rms_array[2] = sqrtf(session_sum_sq[2] * inv_window);
      data_to_send.rms_array = rms_array;
      data_to_send.time_stamp = time_stamp;

      // Print RMS values
      Serial.printf("[FFT] RMS: x:%.2f y:%.2f z:%.2f\n", rms_array[0], rms_array[1], rms_array[2]);

      // Check for anomalies
      if (anomaly_detection(rms_array[0],rms_array[1],rms_array[2])) {
        anomaly_detected = true;
        data_to_send.anomaly = true;
        anomaly = data_to_send;
      }

      Serial.println("**********SEND************");
      send_data(data_to_send);

      // Reset session sum of squares
      session_sum_sq[0] = session_sum_sq[1] = session_sum_sq[2] = 0.0f;
      num_of_samples = 0;


      // notify from rms_send_task
       // To be defined:  max delay
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000)) == 0) {
        // Timeout occurred
        Serial.println("[ERROR] Notification timeout - MQTT task not responding");
      } else {
        Serial.println("[FFT] Received notification from MQTT task");
      }
    }

    unsigned long cycle_end_ms = micros();
    Serial.printf("\n[TIMING] Cycle duration: %lu microseconds\n", cycle_end_ms - cycle_start_ms);

    num_of_samples++;
    deep_sleep(1000/g_sampling_frequency);
  }
  vTaskDelete(NULL);
}