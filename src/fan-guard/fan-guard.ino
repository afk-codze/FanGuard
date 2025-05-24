#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared-defs.h"
#include "driver/adc.h"
#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "ina-library.h"
#include "MPU6500-library.h"
#include "sampling.h"
#include "driver/uart.h"

#define BUZZER_PIN 23

#define DEEP_SLEEP_INA_SEC 10
#define DEEP_SLEEP_INA_US (DEEP_SLEEP_INA_SEC * 1000ULL * 1000ULL)

void deep_sleep() {
  esp_wifi_stop();
  Serial.flush();
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_INA_US);
  esp_deep_sleep_start();
}

void first_boot_init(){
  // Calc id device
  for (int i = 0; i < 17; i = i + 8) {
    id_device |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  xTaskCreatePinnedToCore(build_baseline_mpu6500, "build_basline_mpu", 4096, xTaskGetCurrentTaskHandle(), 1, NULL, 0);
  xTaskCreatePinnedToCore(build_baseline_ina, "build_baseline_ina", 4096, xTaskGetCurrentTaskHandle(), 1, NULL, 1);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(41,42);
  // Set up interrupt pin
  pinMode(INT_PIN, INPUT_PULLUP); // Set up interrupt pin with pull-up  
  ina219_init();
  acc_init();
  // Queues initialization
  init_shared_queues();

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up from deep sleep by MPU6500 interrupt!");
    motion_anomaly = true; // Set anomaly_motion flag if woke by interrupt
  } else if(wakeup_reason == ESP_SLEEP_WAKEUP_TIMER){
    Serial.println("Woke up from deep sleep by INA timer");
    xTaskCreate(ina_periodic_check, "ina_periodic_check", 8096, xTaskGetCurrentTaskHandle(), 1, NULL);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }else{
    Serial.println("Normal boot or other wakeup reason.");
  }

  if(is_first_boot){
    first_boot_init();
    is_first_boot = false;
  }

  Serial.printf("Device ID: %llX\n", id_device); // Print device ID
  
  if(motion_anomaly){
    motion_anomaly = false;
    Serial.print("*** Anomaly ***");
    xTaskCreate(high_freq_sampling, "high_freq_sampling", 8096, xTaskGetCurrentTaskHandle(), 1, NULL);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  

  // Enable the motion interrupt with our calculated threshold
  enable_motion_interrupt(calculated_threshold);
  
  deep_sleep();
}

void loop() {
  vTaskDelete(NULL);
}