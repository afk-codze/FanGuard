#ifndef SHARED_DEFS_H
#define SHARED_DEFS_H
#include <Wire.h>
#include <MPU6500_WE.h>
#define MPU6500_ADDR 0x68

struct ANOMALY{
  unsigned long time_stamp;
  float *rms_array;
};

typedef ANOMALY anomaly_data_t;

extern RTC_DATA_ATTR bool anomaly_detected;
extern RTC_DATA_ATTR bool anomaly_sent;
extern RTC_DATA_ATTR anomaly_data_t anomaly;
extern RTC_DATA_ATTR MPU6500_WE myMPU6500;
#endif