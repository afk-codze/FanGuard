#include "esp_attr.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include "communication.h"
#include <PubSubClient.h>
#include "shared-defs.h"
#include "secrets.h"
#include <ArduinoJson.h>
#include "config.h"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include "esp_task_wdt.h"

// Network Configuration
#define MSG_BUFFER_SIZE 256  // Maximum size for MQTT messages

/* Global Variables --------------------------------------------------------- */

WiFiClient espClient;        // WiFi client instance
PubSubClient client(espClient);  // MQTT client instance

char msg[MSG_BUFFER_SIZE];   // Buffer for MQTT messages


void send_anomaly_mqtt(data_to_send_t ina_anomaly){
  xQueueSend(xQueue_data, &ina_anomaly, 0);
  xTaskCreate(communication_task, "communication_task", 4096, xTaskGetCurrentTaskHandle(), 1, NULL);
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000)) == 0) {
    // Timeout occurred
    Serial.println("[ERROR] Notification timeout - MQTT task not responding");
  } else {
    Serial.println("Received notification from MQTT task");
  }
}


/* WiFi Management ---------------------------------------------------------- */
/**
 * @brief Initializes and manages WiFi connection
 * @note Implements retry logic with status monitoring
 */
void wifi_init(){
  Serial.printf("\n[WiFi] Connecting to %s\n", WIFI_SSID);
  //WiFi.mode(WIFI_STA); 
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  int numberOfTries = WIFI_MAX_RETRIES;

  while (1) {
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL: 
        Serial.printf("[WiFi] SSID not found\n"); 
        break;

      case WL_CONNECT_FAILED:
        Serial.printf("[WiFi] Failed - WiFi not connected! \n");
        vTaskDelete(NULL); 
        break;

      case WL_CONNECTION_LOST: 
        Serial.printf("[WiFi] Connection was lost\n"); 
        break;

      case WL_SCAN_COMPLETED:  
        Serial.printf("[WiFi] Scan is completed\n"); 
        break;

      case WL_DISCONNECTED:    
        Serial.printf("[WiFi] WiFi is disconnected\n"); 
        break;

      case WL_CONNECTED:
        
        Serial.printf("[WiFi] WiFi is connected!\n");
        return;

      default:
        Serial.printf("[WiFi] WiFi Status: %d\n", WiFi.status());
        break;
    }
    vTaskDelay(500);
    if (numberOfTries <= 0) {
      Serial.printf("[WiFi] Max retries exceeded\n");
      WiFi.disconnect();
      vTaskDelete(NULL);; 
    } else {
      numberOfTries--;
    }
  }
}

/* MQTT Functions ----------------------------------------------------------- */
/**
 * @brief Establishes MQTT connection
 * 
 */
void connect_mqtt() {
  int numberOfTries = WIFI_MAX_RETRIES;
  char clientId[50];
  long r = random(1000);
  sprintf(clientId, "%d", id_device);
  Serial.printf("\n[MQTT] Connecting to %s\n", MQTT_SERVER);
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setSocketTimeout(60);
  client.setKeepAlive(60);
  
  while (!client.connect(clientId)) {
    if(numberOfTries <= 0){
      Serial.printf("[MQTT] Max retries exceeded ");
      vTaskDelete(NULL);
    }else{
      numberOfTries--;
    }
    Serial.printf(".");
    vTaskDelay(RETRY_DELAY);
  }
  
  if (!client.connected()) {
    Serial.printf("[MQTT] Timeout\n");
    vTaskDelete(NULL);     
  }

  Serial.printf("[MQTT] Connected\n");

}

void send_mqtt(){
  data_to_send_t data;
  while(xQueueReceive(xQueue_data, &data, (TickType_t)100)) {
    Serial.printf("----%d----",data.time_stamp);
    send_to_mqtt(data);
  }
}


float round_to_2dp(float value) {
    return round(value * 100) / 100;
}

/**
 * @brief Publishes data to MQTT broker
 */
void send_to_mqtt(data_to_send_t data_to_send){
  StaticJsonDocument<256> doc;

  doc["time_stamp"] = data_to_send.time_stamp;
  doc["type"] = dataTypeToString(data_to_send.type); 
  doc["classification"] = dataClassificationToString(data_to_send.classification);
  doc["data"] = data_to_send.data; 

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  Serial.print("Sending JSON: ");
  Serial.println(jsonBuffer);

  String topicStr;
  if (data_to_send.type == TYPE_MPU) {
    topicStr = String(id_device) + "/" + MPU_TOPIC;
  } else {
    topicStr = String(id_device) + "/" + INA_TOPIC;
  }
  const char* topic = topicStr.c_str();
  Serial.printf("\nTopic: %s\n",topic);
  if (client.publish(topic, jsonBuffer)){
    Serial.printf("[MQTT] Publishing: %s\n", jsonBuffer);
  }else{
    Serial.printf("[MQTT] ERROR while publishing rms: %s\n", jsonBuffer);
    Serial.printf("client.connected(): %d",client.connected());
  }
}


void communication_task(void *args){
  esp_wifi_start();
  wifi_init();
  connect_mqtt();
  send_mqtt();
  esp_wifi_stop();
  xTaskNotifyGive((TaskHandle_t)args);
  vTaskDelete(NULL);
}
