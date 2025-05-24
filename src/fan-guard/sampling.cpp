#include "projdefs.h"
#include "sampling.h"

xyzFloat motion_samples[SAMPLE_RATE];

bool classify_anomaly(xyzFloat *motion_samples,ina_data_t *ina_samples){
  Serial.printf("\n classify anomaly\n");
  return true;
}

void send_anomaly_mqtt(bool anomaly_class,xyzFloat *motion_samples,ina_data_t *ina_samples){
  Serial.printf("*** Anomaly mpu sent ***");
}


void high_freq_sampling(void *args){

  Serial.printf("--- SAMPLING ---\n");
  for (int i =0; i<SAMPLE_RATE; i++) {
    motion_samples[i] = my_mpu6500.getGValues();
    //ina_samples[i] = read_ina_filtered();
    vTaskDelay(pdTICKS_TO_MS(1000/SAMPLE_RATE));
  }
  Serial.print("\nSampling finished\n");

  bool anomaly_class = classify_anomaly(motion_samples,ina_samples);

  if(anomaly_class)
    send_anomaly_mqtt(anomaly_class,motion_samples,ina_samples);

  xTaskNotifyGive((TaskHandle_t)args);
  vTaskDelete(NULL);
}
