#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

typedef struct {
    float fTemperature;
    float fHumidity;
} SensorData_t;

typedef struct {
    char broker_uri[128];
    char username[32];
    char password[64];
    char client_id[32];
} MqttConfigData_t;

#endif // SHARED_TYPES_H
