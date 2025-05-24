#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "shared-defs.h"
#include "hmac.h"
#include <esp_wifi.h>
#include <esp_wifi_types.h>
// MQTT Client declaration
extern PubSubClient client;

// Data structures
struct rtt_data {
    int id;
    float avg;
    float rtt;
};

// WiFi functions
void wifi_init();

// MQTT functions
void connect_mqtt(void *arg);
//void callback(char* topic, byte* message, unsigned int length);
void send_to_mqtt(data_to_send_t data_to_send);

void send_anomaly_mqtt(data_to_send_t ina_anomaly);
// Task handlers
void communication_mqtt_task(void *pvParameters);

// Utility functions
void print_rtts();
void print_volume_of_communication();
void start_time_communication();
void end_time_comunication();
void communication_task(void *args);