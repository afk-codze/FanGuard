#ifndef SHARED_DEFS_H
#define SHARED_DEFS_H
#include <Wire.h>
#include <config.h>
#include "driver/adc.h"
#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_pm.h"

#define MPU6500_ADDR 0x68

enum DataType {
  TYPE_UNKNOWN,
  TYPE_INA,
  TYPE_MPU,
};

// Define an enum for 'Classification'
enum DataClassification {
  CLASS_UNKNOWN,
  CLASS_1,
  CLASS_2,
  CLASS_3,
};

struct DATA_TO_SEND{
  unsigned long time_stamp = 0;
  DataType type = TYPE_UNKNOWN;
  DataClassification classification = CLASS_UNKNOWN;
  char *data = "";
};

typedef DATA_TO_SEND data_to_send_t;

extern RTC_DATA_ATTR volatile bool is_first_boot;

// Global variable to store the calculated motion detection threshold.
// This threshold is determined during the baseline calibration.
extern RTC_DATA_ATTR uint8_t calculated_threshold;

extern QueueHandle_t xQueue_data;
extern RTC_DATA_ATTR int id_device;

void init_shared_queues();
const char* dataTypeToString(DataType type);
const char* dataClassificationToString(DataClassification classification);
void send_anomaly_mqtt(data_to_send_t ina_anomaly);
#endif