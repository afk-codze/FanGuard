
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fft-and-adaptive-sampling.h"
#include <MPU6500_WE.h>
#include <Wire.h>
#include "shared-defs.h"
#define BUZZER_PIN 23

MPU6500_WE myMPU6500(MPU6500_ADDR);

RTC_DATA_ATTR bool initialized = false;

//Saving the accelerometer's calibration settings across deep sleep
RTC_DATA_ATTR float acc_off_x = 0.0;
RTC_DATA_ATTR float acc_off_y = 0.0;
RTC_DATA_ATTR float acc_off_z = 0.0;
RTC_DATA_ATTR float gyr_off_x = 0.0;
RTC_DATA_ATTR float gyr_off_y = 0.0;
RTC_DATA_ATTR float gyr_off_z = 0.0;

TaskHandle_t send_anomaly_task_handler  = NULL;

void buzzer_anomaly_task(void *args){
  if(anomaly_detected){
    while(1){
      digitalWrite(BUZZER_PIN, HIGH);
    }
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(BUZZER_PIN,OUTPUT);
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  
  if(!initialized){
    Serial.println("MPU6500 - calibrating...");
    delay(1000);
    myMPU6500.autoOffsets();
    
    // Get offsets as regular xyzFloat
    xyzFloat accOffsets = myMPU6500.getAccOffsets();
    xyzFloat gyrOffsets = myMPU6500.getGyrOffsets();
    
    // Store individual components to RTC memory
    acc_off_x = accOffsets.x;
    acc_off_y = accOffsets.y;
    acc_off_z = accOffsets.z;
    
    gyr_off_x = gyrOffsets.x;
    gyr_off_y = gyrOffsets.y;
    gyr_off_z = gyrOffsets.z;
    
    Serial.printf("acc_off: %.2f, %.2f, %.2f\n", acc_off_x, acc_off_y, acc_off_z);
    Serial.printf("gyr_off: %.2f, %.2f, %.2f\n", gyr_off_x, gyr_off_y, gyr_off_z);
    
    initialized = true;
    Serial.println("Done!");
  } else {
    Serial.println("Using saved calibration values");
    
    // Create temporary xyzFloat objects
    xyzFloat accOffsets;
    accOffsets.x = acc_off_x;
    accOffsets.y = acc_off_y;
    accOffsets.z = acc_off_z;
    
    xyzFloat gyrOffsets;
    gyrOffsets.x = gyr_off_x;
    gyrOffsets.y = gyr_off_y;
    gyrOffsets.z = gyr_off_z;
    
    // Now set the offsets
    myMPU6500.setAccOffsets(accOffsets);
    myMPU6500.setGyrOffsets(gyrOffsets);
    
    Serial.printf("acc_off: %.2f, %.2f, %.2f\n", acc_off_x, acc_off_y, acc_off_z);
    Serial.printf("gyr_off: %.2f, %.2f, %.2f\n", gyr_off_x, gyr_off_y, gyr_off_z);
  }

  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
  myMPU6500.setGyrRange(MPU9250_GYRO_RANGE_250); // highest resolution
  myMPU6500.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU9250_DLPF_6);

  xTaskCreate(send_anomaly_task, "send_anomaly_task", 4096, NULL, 1, &send_anomaly_task_handler);
  xTaskCreate(buzzer_anomaly_task, "buzzer_anomaly_task", 4096, NULL, 1, NULL);
  xTaskCreate(fft_sampling_task, "fft_sampling_task", 4096, send_anomaly_task_handler, 1, NULL);
}

void loop() {
 vTaskDelete(NULL); // Delete the task if it is not needed anymore
}
