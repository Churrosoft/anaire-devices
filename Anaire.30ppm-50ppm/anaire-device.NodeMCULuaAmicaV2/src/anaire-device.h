#ifndef ANAIRE_DEVICE_H
#define ANAIRE_DEVICE_H

// Arduino / lib deeps

#include <Arduino.h>
#include <Print.h>

// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// WiFi
#include <ESP8266WiFi.h>                          // Wifi ESP8266
extern "C" {
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "c_types.h"
}

#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>                         // to be reached on anaire_device_id.local in the local network

#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <DoubleResetDetector.h>

// OLED ssd1306 screen
#include <Wire.h>
#include "SSD1306Wire.h"

// MQTT
#include <PubSubClient.h>

// Sensirion SCD CO2, temperature and humidity sensor
#include "../lib/esp-scd30/src/paulvha_SCD30.h"

//JSON
#include <ArduinoJson.h>

// own deeps
#include "config.h"
#include "changelog.h"

#endif