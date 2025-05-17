#include <a1_inferencing.h>

#include "fft-and-adaptive-sampling.h"

// Anomaly flags
RTC_DATA_ATTR bool anomaly_detected = false;
RTC_DATA_ATTR bool anomaly_sent = false;
RTC_DATA_ATTR anomaly_data_t anomaly;

// FFT initialization phase flag
volatile RTC_DATA_ATTR bool fft_init_complete = false;

// Current system sampling frequency (Hz)
RTC_DATA_ATTR int g_sampling_frequency = INIT_SAMPLE_RATE;

// FFT samples
float samples_real[INIT_SAMPLE_RATE];
float samples_imag[INIT_SAMPLE_RATE];

// window of RMS
RTC_DATA_ATTR float window_rms[3][WINDOW_SIZE];
int g_window_size = WINDOW_SIZE; // Window size for RMS calculation

RTC_DATA_ATTR int index_rms = 0;
RTC_DATA_ATTR int num_of_samples = 0;

// ArduinoFFT instance configured with buffers
ArduinoFFT<float> FFT = ArduinoFFT<float>(samples_real, samples_imag, INIT_SAMPLE_RATE, INIT_SAMPLE_RATE);

float features[3] = {0};

void send_rms(float* avg) {
  // Send to queue
}

void send_anomaly(unsigned long ts, float* rms_values) {
  // Trigger alert
  Serial.print("************* ANOMALY ************\n");
  Serial.printf("************* ts: %ul - rms: x = %.2f, y = %.2f, z = %.2f ************\n",ts,rms_values[0],rms_values[1],rms_values[2]);
}

void send_anomaly_task(void *pvParameters){
  if(anomaly_detected && !anomaly_sent){
    send_anomaly(anomaly.time_stamp, anomaly.rms_array);
    anomaly_sent = true;
  }else{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    send_anomaly(anomaly.time_stamp, anomaly.rms_array);
    anomaly_sent = true;
  }
  vTaskDelete(NULL);
}

void read_sample(float* x, float* y, float* z) {
  xyzFloat values = myMPU6500.getGValues();
  *x = values.x;
  *y = values.y;
  *z = values.z;
}

float calculateRMS(float* data, int size) {
  float sumSquares = 0.0;
  for (int i = 0; i < size; i++) {
    sumSquares += data[i] * data[i];
  }
  return sqrt(sumSquares / size);
}

void light_sleep(int duration) {
  uart_wait_tx_idle_polling((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM);
  esp_sleep_enable_timer_wakeup(1000*1000*duration);
  esp_light_sleep_start();
}

void deep_sleep(int duration) {
  uart_wait_tx_idle_polling((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM);
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


void add_to_window(float sample[], float window[][WINDOW_SIZE], int size) {
  window[0][index_rms] = sample[0];
  window[1][index_rms] = sample[1];
  window[2][index_rms] = sample[2];
  index_rms = (index_rms + 1) % size;
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

  unsigned long time_stamp = 0;
  
  if(!fft_init_complete)
    fft_init();
  Serial.printf("[FFT] Starting sampling at %d Hz\n", g_sampling_frequency);
  Serial.println("--------------------------------");

  while(1) {
    // Read sample from sensor
    read_sample(&sample[0], &sample[1], &sample[2]);
  
    
    Serial.printf("[FFT] Sample %d = x: %.2f, y: %.2f, z: %.2f\n",num_of_samples, sample[0], sample[1], sample[2]);

    time_stamp = millis();
    
    add_to_window(sample, window_rms, g_window_size);

    // Send RMS values periodically
    if ((num_of_samples % g_window_size) == 0) {
      rms_array[0] = calculateRMS(window_rms[0], g_window_size);
      rms_array[1] = calculateRMS(window_rms[1], g_window_size);
      rms_array[2] = calculateRMS(window_rms[2], g_window_size);
      send_rms(rms_array);
      // Print RMS values
      Serial.printf("[FFT] RMS: x:%.2f y:%.2f z:%.2f\n", rms_array[0], rms_array[1], rms_array[2]);
      
      // Check for anomalies
      if (anomaly_detection(rms_array[0],rms_array[1],rms_array[2])) {
        anomaly_detected = true;
        anomaly.time_stamp = time_stamp;
        anomaly.rms_array = rms_array;
        
        // notify anomaly task
        xTaskNotifyGive((TaskHandle_t)pvParameters);
      }
    }   
    num_of_samples++;
    deep_sleep(1000/g_sampling_frequency);
  }
  vTaskDelete(NULL);
}