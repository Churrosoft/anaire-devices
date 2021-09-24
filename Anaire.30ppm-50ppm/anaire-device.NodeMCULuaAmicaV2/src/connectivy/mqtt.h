#ifndef ANAIRE_MQTT_H
#define ANAIRE_MQTT_H
#include <Arduino.h>
#include <Print.h>
#include <ESP8266WiFi.h> // Wifi ESP8266

//EEPROM:
#include "eeprom.h"
#include "global-vars.h"
#include "config.h"
#include "../sensors/sensors.h"
// MQTT
#include <PubSubClient.h>

void Init_MQTT();
void Send_Message_Cloud_App_MQTT();
void Receive_Message_Cloud_App_MQTT(char *, byte *, unsigned int);
void MQTTReconnect();

//FIRMWARE STUFF:
void firmware_update(void);
void update_started(void);
void update_finished(void);
void update_progress(int, int);
void update_error(int);

#endif