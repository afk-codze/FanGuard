#include "esp_attr.h"
#include <Arduino.h>
#include "config.h"
#include "shared-defs.h"

QueueHandle_t xQueue_data = NULL;

RTC_DATA_ATTR volatile bool is_first_boot = true;
RTC_DATA_ATTR int id_device = 0;
RTC_DATA_ATTR uint8_t calculated_threshold=0;

const char* dataTypeToString(DataType type) {
  switch (type) {
    case TYPE_INA: return "INA";
    case TYPE_MPU: return "MPU";
    default:  return "unknown";
  }
}

const char* dataClassificationToString(DataClassification classification) {
  switch (classification) {
    case CLASS_1:    return "CLASS_1";
    case CLASS_2: return "CLASS_2";
    case CLASS_3:     return "CLASS_3";
    default:                  return "unknown";
  }
}



void init_shared_queues() {
    xQueue_data = xQueueCreate(QUEUE_SIZE, sizeof(data_to_send_t));
    if(xQueue_data ==  NULL) {
        Serial.println("Queue creation failed!");
        while(1); // Halt on critical failure
    }
    
}