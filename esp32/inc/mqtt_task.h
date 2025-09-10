#ifndef MQTT_TASK_H
#define MQTT_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "shared_types.h"

// MQTT configuration
#define MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#define MQTT_USERNAME   "iot_user"
#define MQTT_PASSWORD   "iot_password"
#define MQTT_CLIENT_ID  "esp32_001"

// MQTT topics
#define MQTT_TOPIC_SENSOR_DATA   "sensor/data/temperature_humidity"
#define MQTT_TOPIC_DEVICE_STATUS "sensor/status/esp32_health"

void mqtt_init(void);
void MqttTask_CreateTask(void);
bool mqtt_is_connected(void);
esp_err_t mqtt_publish_sensor_data(const SensorData_t* sensor_data);
esp_err_t mqtt_publish_device_status(const char* status);
esp_err_t mqtt_reconfigure(const MqttConfigData_t* new_config);

#endif // MQTT_TASK_H
