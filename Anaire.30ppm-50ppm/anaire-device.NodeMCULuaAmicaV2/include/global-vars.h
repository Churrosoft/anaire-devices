#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H
// WIP EN EL WIP DEL WIP:
#include <Arduino.h>
#include <Print.h>

// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// WiFi
#include <ESP8266WiFi.h> // Wifi ESP8266
extern "C"
{
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "c_types.h"
}

#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h> // to be reached on anaire_device_id.local in the local network

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

#include "../lib/esp-scd30/src/paulvha_SCD30.h"

//CO2:
enum CO2_sensors
{
    none,
    MHZ14A,
    SCD30_
}; // possible sensors integrated in the SW
// CO2 device status
enum CO2_status
{
    ok,
    warning,
    alarm
};

// MHZ14A CO2 sensor: software serial port
#include "SoftwareSerial.h" // Remove if using HardwareSerial or non-uno compatabile device
#include "MHZ19.h"          // https://github.com/WifWaf/MH-Z19 Library
// AZ-Delivery DHT11
#include "DHTesp.h"
// Ticker library to blink leds and buzzer
#include <Ticker.h> //Ticker Library
//END

extern char response_CO2[9];  // holds the received data from MHZ14A CO2 sensor
extern int response_CO2_high; // holds upper byte
extern int response_CO2_low;  // holds lower byte

extern int CO2ppm_value;       // CO2 ppm measured value
extern int CO2ppm_accumulated; // Accumulates co2 measurements for a MQTT period
extern int CO2ppm_samples;     // Counts de number of samples for a MQTT period

// Initialize DHT sensor
extern DHTesp dht;
extern float temperature; // Read temperature as Celsius
extern float humidity;    // Read humidity in %

// device status
extern boolean err_global;
extern boolean err_wifi;
extern boolean err_MQTT;
extern boolean err_co2;
extern boolean err_dht;
extern boolean err_oled;

// Ticker library to blink leds and buzzer
extern Ticker blinker_CO2_STATUS_BUILTIN_LED_GPIO; // to blink CO2_STATUS_BUILTIN_LED_GPIO
extern Ticker blinker_BUZZER_GPIO;                 // to blink BUZZER_GPIO for the alarm sound

// flag to update OLED display with status info from main loop instead of button ISR
extern boolean update_OLED_status_flag;

// flag to update OLED display with CO2 info from main loop instead of button ISR
extern boolean update_OLED_co2_flag;

// flag to store flash button status
extern boolean flash_button_pressed_flag;

// to indicate if push button has been pushed to ack the alarm and switch off the buzzer
extern boolean alarm_ack;

// to know when there is an updating process in place
extern boolean updating;

extern WiFiClient wifi_client;
extern const int WIFI_CONNECT_TIMEOUT; // 5 seconds
extern int wifi_status;
extern WiFiServer wifi_server; // to check if it is alive
extern String wifi_ssid;       // your network SSID (name)
extern String wifi_password;   // your network psk password

extern String anaire_device_id; // HEX version, for easier match to mac address

// MQTT
extern char MQTT_message[256];
extern String MQTT_send_topic;
extern String MQTT_receive_topic; // config messages will be received in config/id
extern PubSubClient MQTT_client;
extern char received_payload[384];

// Measurements loop: time between measurements
extern unsigned long measurements_loop_start; // holds a timestamp for each control loop start

// MQTT loop: time between MQTT measurements sent to the cloud
extern unsigned long MQTT_loop_start;      // holds a timestamp for each cloud loop start
extern unsigned long lastReconnectAttempt; // MQTT reconnections

// Errors loop: time between error condition recovery
extern unsigned long errors_loop_start; // holds a timestamp for each error loop start

// flash button duration: time since flash button was pressed
extern unsigned long flash_button_press_start; // holds a timestamp for each control loop start

//OLED
extern SSD1306Wire display; // ADDRESS, SDA, SCL

extern CO2_sensors co2_sensor;
extern CO2_status co2_device_status;
//json
extern StaticJsonDocument<384> jsonBuffer;

extern SCD30 airSensor;
extern MHZ19 myMHZ19;

extern SoftwareSerial mySerial;

#endif