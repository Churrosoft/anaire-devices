#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include <Print.h>


// Measurements loop: time between measurements
constexpr unsigned int measurements_loop_duration = 5000;   // 5 seconds

// MQTT loop: time between MQTT measurements sent to the cloud
constexpr unsigned int MQTT_loop_duration = 30000;          // 30 seconds

// Errors loop: time between error condition recovery
constexpr unsigned int errors_loop_duration = 3000;         // 3 seconds

// CO2 Blinking period, used to reflect CO2 status on builtin led and buzzer
constexpr int WARNING_BLINK_PERIOD = 1000;            // 1 second
constexpr int ALARM_BLINK_PERIOD = 200;               // 0.2 seconds

// Number of seconds after reset during which a subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

// OLED ssd1306 screen
#include <Wire.h>
#include "SSD1306Wire.h"

// OLED ssd1306 screen
// SCL and SDA pin connections
#define OLED_SCK_GPIO 14  // signal GPIO14 (D5)
#define OLED_SDA_GPIO 12  // signal GPIO12 (D6)

// Sensirion SCD CO2, temperature and humidity sensor
#define SCD30WIRE Wire
#define SCD30_SCK_GPIO 14 // signal GPIO14 (D5)
#define SCD30_SDA_GPIO 12 // signal GPIO12 (D6)

/* // CO2 sensors
enum CO2_sensors
{
  none,
  MHZ14A,
  SCD30
}; // possible sensors integrated in the SW
CO2_sensors co2_sensor = none; */

// AZ-Delivery Active Buzzer
#define BUZZER_GPIO 4 // signal GPIO4 (D2)

// GPIO0 (D3) is button FLASH on nodemcu PCB
#define FLASH_BUTTON_GPIO 0

// NodeMCU Builtin LED, used to provide CO2 visual status info:
// CO2 OK: off
// WARNING: blinks slow
// ALARM: blinks fast
#define CO2_STATUS_BUILTIN_LED_GPIO 16 // GPIO16 (D0), closer to the usb port, on the NodeMCU PCB - below the button labeled R (Reset) on 3D Box
//#define CO2_STATUS_BUILTIN_LED_GPIO 2                           // GPIO2 (D4), on the ESP-12 module’s PCB, far away from the usb port - near button labeled A (Alarm) on 3D Box

// The other NodeMCU builtin LED, used to provide device status info:
// Device OK: off
// Any error or config status: on
//#define DEVICE_STATUS_BUILTIN_LED_GPIO 16                       // GPIO16 (D0), closer to the usb port, on the NodeMCU PCB -  below the button labeled R (Reset) on 3D Box
#define DEVICE_STATUS_BUILTIN_LED_GPIO 2 // GPIO2 (D4), on the ESP-12 module’s PCB, far away from the usb port - near button labeled A (Alarm) on 3D Box


#endif