#ifndef SHARED_DEFS_H
#define SHARED_DEFS_H
#include <Wire.h>
#include <config.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>

#define ADXL345_SDA 41
#define ADXL345_SCL 42

struct DATA_TO_SEND{
  unsigned long time_stamp = 0;
  float *rms_array = {0};
  volatile bool anomaly = false;
};

typedef DATA_TO_SEND data_to_send_t;

extern TaskHandle_t fft_sampling_task_handle;

extern QueueHandle_t xQueue_data;
extern RTC_DATA_ATTR bool anomaly_detected;
extern RTC_DATA_ATTR bool anomaly_sent;
extern RTC_DATA_ATTR bool this_reboot_send_rms;
extern RTC_DATA_ATTR data_to_send_t anomaly;
extern RTC_DATA_ATTR Adafruit_ADXL345_Unified accel;
void init_shared_queues();
#endif