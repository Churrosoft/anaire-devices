//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Anaire 30/50ppm - Open CO2, temperature and humidity measurement device connected to the Anaire Cloud Application https://github.com/anaireorg/anaire-cloud
// 20201109 anaire.org anaire@anaire.org
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// HW Parts - Common:
//   Control board: AZDelivery ESP8266 ESP-12F NodeMCU Lua Amica V2 https://www.az-delivery.de/es/products/nodemcu
//   Buzzer: AZDelivery Active Buzzer - https://www.az-delivery.de/es/products/buzzer-modul-aktiv?_pos=2&_sid=39cea0af6&_ss=r
//   Display: AZDelivery 0.91 inch OLED I2C Display 128 x 32 Pixels https://www.az-delivery.de/es/products/0-91-zoll-i2c-oled-display
//   3D printed Box designed by Anaire: https://www.thingiverse.com/thing:4694633
//
// HW Parts - Sensors:
//   Anaire 30ppm, better measurements precision: Sensirion SCD30 for CO2, temperature and humidity https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors/carbon-dioxide-sensors-scd30/
//   Anaire 50ppm, less cost: MHZ14A for CO2 https://www.winsen-sensor.com/sensors/co2-sensor/mh-z14a.html and AZ-Delivery DHT11 Temperature and humidity sensor - https://www.az-delivery.de/es/products/dht11-temperatursensor-modul
//
// SW Setup:
//   Install the usb to uart driver from https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
//   Start Arduino and open Preferences window.
//   Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.
//   Open Boards Manager from Tools > Board menu and find esp8266 platform by esp8266 community and install the software for Arduino from a drop-down box.
//   Select "NodeMCU 1.0" board from Tools > Board menu after installation
//
// Install the following libraries in Arduino IDE:
//   WiFiEsp by bportaluri https://github.com/bportaluri/WiFiEsp
//   Pubsubclient by Nick O'Leary https://pubsubclient.knolleary.net/
//   ArduinoJson by Benoit Blanchon https://arduinojson.org/?utm_source=meta&utm_medium=library.properties
//   esp8266-oled-ssd1306 by ThingPulse, Fabrice Weinberg https://github.com/ThingPulse/esp8266-oled-ssd1306
//   DHTesp by beegee_tokyo https://github.com/beegee-tokyo/DHTesp
//   ESP_EEPROM by j-watson https://github.com/jwrw/ESP_EEPROM
//   WifiManager by tzapu,tablatronix https://github.com/tzapu/WiFiManager
//   Double Reset detector by Stephen Denne https://github.com/datacute/DoubleResetDetector
//
//   MHZ-Z19 Library from WifWaf https://github.com/WifWaf/MH-Z19 Library - to manage Winsen CO2 sensors (tested with MHZ-14A and MHZ-19c) - INSTALL FROM ZIP FILE with Sketch-> Include Library-> Add .ZIP library
//   Modified paulvha SCD30 library, get it from anaire github on https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/docs/scd30-master.zip - INSTALL FROM ZIP FILE with Sketch-> Include Library-> Add .ZIP library
//
// Design:
// - The ID (last 3 fields from MAC address) and IP address are shown on OLED display during boot and after pressing the Flash button
// - Pressing the reset button ("R") twice in less than 10 seconds restarts the device in a captive web configuration portal, useful to configure local wifi settings and MQTT endpoint
// - All other local config parameters (like name, thresholds, local alarm, etc.) are configured via the cloud app, after connecting
// - Pressing Alarm button ("A") toggles between activating/deactivating local sound alarm alarms, and also show device information on the display
//   * Stops local sound alerting until pressed again or until CO2 level decays below the warning threshold, deactivating therefore local alerting and reseting the local alerting system
//   * It also shows device ID and IP address after being pressed during 3 seconds
// - Pressing Alarm button ("A") during more than 10 seconds activates forced calibration
// - Firmware update with latest fimrware available though cloud app
//   * Always using latest firmware on https://github.com/anaireorg/anaire-devices/blob/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin
// - Built in LED in GPIO16-D0 (the one that blinks near the nodemcu usb connector):
//   * When CO2 Status is "ok" (below warning threshold) LED and buzzer are off (normal status)
//   * When CO2 Status is "warning" builtin LED and external buzzer alternate blink at a slow pace (WARNING_BLINK_PERIOD)
//   * When CO2 Status is "alarm" builtin LED and external buzzer alternate blink at fast pace (ALARM_BLINK_PERIOD)
// - The device is designed to work only with the CO2 sensor, so buzzer, DHT11 humidity and temperature sensor, and OLED display are optional
// - The device is designed to recover from Wifi, MQTT or sensors reading temporal failures. Local measurements will always be shown in the local display.
// - The web server is activated, therefore entering the IP on a browser allows to see device specific details and measurements; device forced calibration is also available through the web server
