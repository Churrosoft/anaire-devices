#ifndef EEPROM_H
#define EEPROM_H

#include <Arduino.h>
// Save config values to EEPROM
#include <ESP_EEPROM.h>
#include <Print.h>
#include "../changelog.h"
#include <ESP8266WiFi.h>                          // Wifi ESP8266

struct MyEEPROMStruct
{
    char anaire_device_name[24] = "";         // Device name; default to anaire_device_id
    uint16_t CO2ppm_warning_threshold = 700;  // Warning threshold; default to 700ppm
    uint16_t CO2ppm_alarm_threshold = 1000;   // Alarm threshold; default to 1000ppm
    char MQTT_server[24] = "mqtt.anaire.org"; // MQTT server url or public IP address. Default to Anaire Portal on portal.anaire.org
    uint16_t MQTT_port = 80;                  // MQTT port; Default to Anaire Port on 30183
    boolean sound_alarm = true;               // Global flag to control sound alarm; default to true
    boolean ABC = false;                      // Automatic baseline Correction; default to false
    uint16_t FRC_value = 400;                 // Forced ReCalibration value; default to 400ppm
    uint16_t temperature_offset = 0;          // temperature offset for SCD30 CO2 measurements

    uint16_t altitude_compensation = 0; // altitude compensation for SCD30 CO2 measurements
    char wifi_user[24] = "";            // WiFi user to be used on WPA Enterprise. Default to null (not used)
    char wifi_password[24] = "";        // WiFi password to be used on WPA Enterprise. Default to null (not used)
} typedef MyEEPROMStruct;

extern MyEEPROMStruct eepromConfig;

void Read_EEPROM(void);
void Write_EEPROM(void);
void Wipe_EEPROM(void);
void Print_Config(void);
#endif