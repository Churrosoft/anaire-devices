#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <Print.h>
#include <ESP8266WiFi.h>                          // Wifi ESP8266

// OLED ssd1306 screen
#include <Wire.h>
#include "SSD1306Wire.h"

//EEPROM:
#include "eeprom.h"
#include "global-vars.h"
#include "config.h"

void update_OLED_Status();
void update_OLED_CO2(int, float, float);

#endif