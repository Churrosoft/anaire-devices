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

// Sensirion SCD CO2, temperature and humidity sensor
#define SCD30WIRE Wire
#define SCD30_SCK_GPIO 14 // signal GPIO14 (D5)
#define SCD30_SDA_GPIO 12 // signal GPIO12 (D6)

constexpr unsigned long SCD30_WARMING_TIME = 2000;                                 // SCD30 CO2 sensor warming time
constexpr unsigned long SCD30_CALIBRATION_TIME = 180000;                           // SCD30 CO2 CALIBRATION TIME: 3 min = 180000 ms
constexpr uint16_t SCD30_MEASUREMENT_INTERVAL = measurements_loop_duration / 1000; // time between measurements

// MHZ14A CO2 sensor: software serial port
#define MHZ_BAUDRATE 9600                              // Native to the sensor (do not change)
constexpr unsigned long MHZ14A_WARMING_TIME = 180000;      // MHZ14A CO2 sensor warming time: 3 minutes = 180000 ms
constexpr unsigned long MHZ14A_SERIAL_TIMEOUT = 3000;      // MHZ14A CO2 serial start timeout: 3 seconds = 3000 ms
constexpr unsigned long MHZ14A_CALIBRATION_TIME = 1200000; // MHZ14A CO2 CALIBRATION TIME: 20 min = 1200000 ms
#define swSerialRX_gpio 13
#define swSerialTX_gpio 15

constexpr byte measurement_command[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; // Command to get measurements from MHZ14A CO2 sensor
constexpr byte calibration_command[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78}; // Command to calibrate MHZ14A CO2 sensor

// AZ-Delivery DHT11
#define DHT_GPIO 5 // signal GPIO5 (D1)

#endif