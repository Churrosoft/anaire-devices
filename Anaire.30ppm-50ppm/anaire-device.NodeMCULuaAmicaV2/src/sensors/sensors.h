#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>
#include <Print.h>

// MHZ14A CO2 sensor: software serial port
#include "SoftwareSerial.h" // Remove if using HardwareSerial or non-uno compatabile device
#include "MHZ19.h"          // https://github.com/WifWaf/MH-Z19 Library
// AZ-Delivery DHT11
#include "DHTesp.h"
// Ticker library to blink leds and buzzer
#include <Ticker.h> //Ticker Library

#include "global-vars.h"
#include "config.h"

//EEPROM:
#include "eeprom.h"

void Evaluate_CO2_Value(void);
void Read_DHT11(void);

void SCD30_Do_Temperature_Offset(void);
void SCD30_Do_Measurement_Interval(void);
void SCD30_Do_Forced_Calibration_Factor(void);
void SCD30_Do_Altitude_Compensation(void);
void SCD30_Do_AutoSelfCalibration(void);
void SCD30DeviceInfo(void);
void Calibrate_SCD30(void);
void Read_SCD30(void);
void Read_Sensors(void);
void Read_MHZ14A(void);
void Calibrate_MHZ14A(void);
void Setup_sensors(void);

// LED:
// To blink on CO2_STATUS_BUILTIN_LED_GPIO
void changeState_CO2_STATUS_BUILTIN_LED_GPIO(void);
void changeState_BUZZER_GPIO(void);

#endif