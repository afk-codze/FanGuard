#include "esp_attr.h"
#include "ina-library.h"
#include "shared-defs.h"
Adafruit_INA219 ina219;

// Variables for filtering
float filtered_bus_voltage = 0;   
float filtered_shunt_voltage = 0; 
float filtered_current = 0;       
float filtered_power = 0;         
float filtered_load_voltage = 0;  
float prev_bus_voltage = 0;       
float prev_current = 0;          

RTC_DATA_ATTR float baseline_mean = 0;
RTC_DATA_ATTR float baseline_std = 0;
float power_readings[BASELINE_SAMPLES];
RTC_DATA_ATTR int total_readings = 0;
bool first_reading = true; 

RTC_DATA_ATTR float max_deviation = 0;
RTC_DATA_ATTR int ina_threshold = 0;
ina_data_t ina_samples[PERIODIC_CHECK_SAMPLES];

void ina219_init() {
  Serial.begin(115200);
  while (!Serial) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  Serial.println("Based on INA219 Power Monitor");

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { vTaskDelay(pdMS_TO_TICKS(10)); }
  }

  ina219.setCalibration_16V_400mA();

  // Initialize filters with first reading
  vTaskDelay(pdMS_TO_TICKS(1000));
  filtered_bus_voltage = ina219.getBusVoltage_V();
  filtered_shunt_voltage = ina219.getShuntVoltage_mV();
  filtered_current = ina219.getCurrent_mA();
  filtered_power = ina219.getPower_mW();
  filtered_load_voltage = filtered_bus_voltage + (filtered_shunt_voltage / 1000);

  prev_bus_voltage = filtered_bus_voltage;
  prev_current = filtered_current;

  Serial.println("INA219 initialized successfully");
}

float get_averaged_reading(float (*read_function)(), int samples) { 
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += read_function();
    //vTaskDelay(pdMS_TO_TICKS(2));
  }
  return sum / samples;
}

float get_bus_voltage() { return ina219.getBusVoltage_V(); }     
float get_shunt_voltage() { return ina219.getShuntVoltage_mV(); } 
float get_current() { return ina219.getCurrent_mA(); }          
float get_power() { return ina219.getPower_mW(); }               

float spike_filter(float new_value, float prev_value, float threshold) { 
  if (abs(new_value - prev_value) > threshold) {
    return prev_value;
  }
  return new_value;
}

float low_pass_filter(float new_value, float prev_filtered, float alpha) { 
  return alpha * new_value + (1 - alpha) * prev_filtered;
}

ina_data_t read_ina_filtered() {
  float raw_bus_voltage = get_averaged_reading(get_bus_voltage, SAMPLE_COUNT);       
  float raw_shunt_voltage = get_averaged_reading(get_shunt_voltage, SAMPLE_COUNT);   
  float raw_current = get_averaged_reading(get_current, SAMPLE_COUNT);               
  float raw_power = get_averaged_reading(get_power, SAMPLE_COUNT);                   
  float raw_load_voltage = raw_bus_voltage + (raw_shunt_voltage / 1000);             

  raw_bus_voltage = spike_filter(raw_bus_voltage, prev_bus_voltage, SPIKE_THRESHOLD); 
  raw_current = spike_filter(raw_current, prev_current, 50.0);                      

  if (first_reading) {
    filtered_bus_voltage = raw_bus_voltage;
    filtered_shunt_voltage = raw_shunt_voltage;
    filtered_current = raw_current;
    filtered_power = raw_power;
    filtered_load_voltage = raw_load_voltage;
    first_reading = false;
  } else {
    filtered_bus_voltage = low_pass_filter(raw_bus_voltage, filtered_bus_voltage, ALPHA);       
    filtered_shunt_voltage = low_pass_filter(raw_shunt_voltage, filtered_shunt_voltage, ALPHA); 
    filtered_current = low_pass_filter(raw_current, filtered_current, ALPHA);                   
    filtered_power = low_pass_filter(raw_power, filtered_power, ALPHA);                        
    filtered_load_voltage = low_pass_filter(raw_load_voltage, filtered_load_voltage, ALPHA);    
  }

  prev_bus_voltage = filtered_bus_voltage;
  prev_current = filtered_current;

  return {filtered_power,filtered_load_voltage,filtered_current};
}

float calculate_mean(float* array, int size) { 
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += array[i];
  }
  return sum / size;
}

float calculate_std_dev(float* array, int size, float mean) { 
  float sum_squared_diff = 0;
  for (int i = 0; i < size; i++) {
    float diff = array[i] - mean;
    sum_squared_diff += diff * diff;
  }
  return sqrt(sum_squared_diff / size);
}

bool detect_threshold_anomaly(float reading) {
  float deviation = abs(reading - baseline_mean);

  if (deviation > max_deviation) {
    max_deviation = deviation;
  }

  return deviation > ina_threshold;
}

void build_baseline_ina(void *args){
  for(int i=0; i < BASELINE_SAMPLES;i++){
    power_readings[i] = read_ina_filtered().fl_power;
    Serial.print("Calibrating... "); Serial.print(i);
    Serial.print("/"); Serial.print(BASELINE_SAMPLES);
    Serial.print(" | Power: "); Serial.print(power_readings[i], 2); Serial.println(" mW");
    vTaskDelay(pdMS_TO_TICKS(READING_DELAY));
  }

  baseline_mean = calculate_mean(power_readings, BASELINE_SAMPLES);   
  baseline_std = calculate_std_dev(power_readings, BASELINE_SAMPLES, baseline_mean); 

  // Ensure minimum standard deviation to avoid false positives
  if (baseline_std < 1.0) {
    baseline_std = 1.0;
  }

  Serial.println("\n=== BASELINE ESTABLISHED ===");
  Serial.print("Mean Power: "); Serial.print(baseline_mean, 2); Serial.println(" mW");
  Serial.print("Std Deviation: "); Serial.print(baseline_std, 2); Serial.println(" mW");
  Serial.print("Anomaly Threshold: "); Serial.print(ANOMALY_THRESHOLD * baseline_std, 2); Serial.println(" mW");
  Serial.print("Samples Used: "); Serial.println(BASELINE_SAMPLES);
  Serial.println("Starting anomaly detection...");
  Serial.println("===========================\n");
  ina_threshold = ANOMALY_THRESHOLD * baseline_std;

  xTaskNotifyGive((TaskHandle_t)args);
  vTaskDelete(NULL);
}

DataClassification classify_ina(ina_data_t *samples){
  Serial.printf("\n classify_ina \n");
  return CLASS_1;
}

void ina_periodic_check(void *args){
  Serial.printf("*** ina_periodic_check ***");
  unsigned long start = millis(),ts;
  int local_anomalies = 0;
  data_to_send_t data_to_send;
  bool ina_anomaly = false;
  for (int i =0; i<PERIODIC_CHECK_SAMPLES; i++) {
    ina_samples[i] = read_ina_filtered();
    
    if(detect_threshold_anomaly(ina_samples[i].fl_power))
      local_anomalies++;

    if(local_anomalies>=3){
      ina_anomaly = true;
      Serial.print("!!! ANOMALY THRESHOLD !!!");
      break;
    }
    vTaskDelay(pdTICKS_TO_MS(1000/PERIODIC_CHECK_SAMPLES));
  }
  ts = millis();
  Serial.print(ts-start);
  if(ina_anomaly){
    data_to_send.classification = classify_ina(ina_samples);
    data_to_send.type = TYPE_INA;
    data_to_send.time_stamp = ts;
    data_to_send.data = "data";
    send_anomaly_mqtt(data_to_send);
  }

  xTaskNotifyGive((TaskHandle_t)args);
  vTaskDelete(NULL);
}