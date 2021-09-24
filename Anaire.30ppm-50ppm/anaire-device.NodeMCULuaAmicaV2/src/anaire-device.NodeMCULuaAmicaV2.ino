#include "anaire-device.h"

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen
// i.e: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String anaire_device_id = String(ESP.getChipId(), HEX); // HEX version, for easier match to mac address

// The neatest way to access variables stored in EEPROM is using a structure
// Init to default values; if they have been chaged they will be readed later, on initialization
MyEEPROMStruct eepromConfig;

// Save config values to EEPROM
#include <ESP_EEPROM.h>

// Measurements loop: time between measurements
unsigned long measurements_loop_start; // holds a timestamp for each control loop start

// MQTT loop: time between MQTT measurements sent to the cloud
unsigned long MQTT_loop_start;          // holds a timestamp for each cloud loop start
unsigned long lastReconnectAttempt = 0; // MQTT reconnections

// Errors loop: time between error condition recovery
unsigned long errors_loop_start; // holds a timestamp for each error loop start

// flash button duration: time since flash button was pressed
unsigned long flash_button_press_start; // holds a timestamp for each control loop start

// CO2 Blinking period, used to reflect CO2 status on builtin led and buzzer
// (on config.h)

WiFiClient wifi_client;
const int WIFI_CONNECT_TIMEOUT = 5000; // 5 seconds
int wifi_status = WL_IDLE_STATUS;
WiFiServer wifi_server(80);        // to check if it is alive
String wifi_ssid = WiFi.SSID();    // your network SSID (name)
String wifi_password = WiFi.psk(); // your network psk password

// RTC Memory Address for the DoubleResetDetector to use
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// for 128x32 displays:
SSD1306Wire display(0x3c, OLED_SDA_GPIO, OLED_SCK_GPIO, GEOMETRY_128_32); // ADDRESS, SDA, SCL

// MQTT
char MQTT_message[256];
String MQTT_send_topic = "measurement";
String MQTT_receive_topic = "config/" + anaire_device_id; // config messages will be received in config/id
PubSubClient MQTT_client(wifi_client);
char received_payload[384];

//JSON
StaticJsonDocument<384> jsonBuffer;

SCD30 airSensor;

/* // MHZ14A CO2 sensor: software serial port
#include "SoftwareSerial.h" // Remove if using HardwareSerial or non-uno compatabile device
#include "MHZ19.h"          // https://github.com/WifWaf/MH-Z19 Library */
MHZ19 myMHZ19;
/* #define MHZ_BAUDRATE 9600                              // Native to the sensor (do not change)
const unsigned long MHZ14A_WARMING_TIME = 180000;      // MHZ14A CO2 sensor warming time: 3 minutes = 180000 ms
const unsigned long MHZ14A_SERIAL_TIMEOUT = 3000;      // MHZ14A CO2 serial start timeout: 3 seconds = 3000 ms
const unsigned long MHZ14A_CALIBRATION_TIME = 1200000; // MHZ14A CO2 CALIBRATION TIME: 20 min = 1200000 ms
#define swSerialRX_gpio 13
#define swSerialTX_gpio 15 */
SoftwareSerial mySerial(swSerialRX_gpio, swSerialTX_gpio);

char response_CO2[9];  // holds the received data from MHZ14A CO2 sensor
int response_CO2_high; // holds upper byte
int response_CO2_low;  // holds lower byte

int CO2ppm_value = 0;       // CO2 ppm measured value
int CO2ppm_accumulated = 0; // Accumulates co2 measurements for a MQTT period
int CO2ppm_samples = 0;     // Counts de number of samples for a MQTT period

/* // AZ-Delivery DHT11
#include "DHTesp.h"
#define DHT_GPIO 5 // signal GPIO5 (D1)
 */
// Initialize DHT sensor
DHTesp dht;
float temperature; // Read temperature as Celsius
float humidity;    // Read humidity in %
CO2_sensors co2_sensor = none;

// the device can have one of those CO2 status
CO2_status co2_device_status = ok; // initialized to ok

// device status
boolean err_global = false;
boolean err_wifi = false;
boolean err_MQTT = false;
boolean err_co2 = false;
boolean err_dht = false;
boolean err_oled = false;

// Ticker library to blink leds and buzzer
Ticker blinker_CO2_STATUS_BUILTIN_LED_GPIO; // to blink CO2_STATUS_BUILTIN_LED_GPIO
Ticker blinker_BUZZER_GPIO;                 // to blink BUZZER_GPIO for the alarm sound

// flag to update OLED display with status info from main loop instead of button ISR
boolean update_OLED_status_flag = false;

// flag to update OLED display with CO2 info from main loop instead of button ISR
boolean update_OLED_co2_flag = false;

// flag to store flash button status
boolean flash_button_pressed_flag = false;

// to indicate if push button has been pushed to ack the alarm and switch off the buzzer
boolean alarm_ack = false;

// to know when there is an updating process in place
boolean updating = false;

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{

  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(115200);
  while (!Serial)
  {
    Serial.print("Attempting to open monitor serial port ");
    delay(1000); // wait 1 seconds for connection
  }

  // Enable debug
  Serial.setDebugOutput(true);

  // print info
  Serial.println();
  Serial.println();
  Serial.println("### INIT ANAIRE DEVICE ###########################################");

  // Initialize DEVICE_STATUS_BUILTIN_LED to ON while the device is initializing
  pinMode(DEVICE_STATUS_BUILTIN_LED_GPIO, OUTPUT);
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

  // Initialize CO2_STATUS_BUILTIN_LED to OFF
  pinMode(CO2_STATUS_BUILTIN_LED_GPIO, OUTPUT);
  digitalWrite(CO2_STATUS_BUILTIN_LED_GPIO, HIGH);

  // Initialize BUZZER to OFF
  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, LOW);

  // Push button: attach interrupt to toggle sound alarm and provide device info on the display
  // It will be used also to launch forced calibration
  pinMode(FLASH_BUTTON_GPIO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLASH_BUTTON_GPIO), push_button_handler, FALLING);

  // Init OLED display
  display.init();

  // Read EEPROM config values
  Read_EEPROM();

  // Print config
  Print_Config();

  // Wifi captive portal if double reset detection
  if (WiFi.SSID() == "")
  {
    Serial.println("We haven't got any access point credentials, so get them now");
    initialConfig = true;
  }
  else
  {
    Serial.println("Access point credentials existing");
    initialConfig = false;
  }
  if (drd.detectDoubleReset())
  {
    Serial.println("Double Reset Detected");
    initialConfig = true;
  }
  else
  {
    Serial.println("Double Reset NOT Detected");
    initialConfig = false;
  }

  // If a double reset was detected start configuration portal
  if (initialConfig)
  {

    Serial.println("Starting configuration portal...");

    // Device status LED ON
    digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

    // Update display to inform
    display.flipScreenVertically();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "CONFIG");
    display.drawString(0, 16, "ESP_" + String(anaire_device_id));
    display.display(); // update OLED display

    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    // Captive portal parameters
    WiFiManagerParameter custom_wifi_html("<p>Set WPA2 Enterprise</p>"); // only custom html
    WiFiManagerParameter custom_wifi_user("User", "WPA2 Enterprise user", eepromConfig.wifi_user, 24);
    WiFiManagerParameter custom_wifi_password("Password", "WPA2 Enterprise Password", eepromConfig.wifi_password, 24);
    WiFiManagerParameter custom_mqtt_html("<p>Set MQTT server</p>"); // only custom html
    WiFiManagerParameter custom_mqtt_server("Server", "MQTT server", eepromConfig.MQTT_server, 24);
    char port[6];
    itoa(eepromConfig.MQTT_port, port, 10);
    WiFiManagerParameter custom_mqtt_port("Port", "MQTT port", port, 6);
    //wifiManager.setSaveParamsCallback(saveParamCallback);

    // Add parameters
    wifiManager.addParameter(&custom_wifi_html);
    wifiManager.addParameter(&custom_wifi_user);
    wifiManager.addParameter(&custom_wifi_password);
    wifiManager.addParameter(&custom_mqtt_html);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    wifiManager.setConfigPortalTimeout(600);

    //it starts an access point
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal())
    {
      Serial.println("Not connected to WiFi but continuing anyway.");
    }
    else
    {
      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
    }

    // Device status LED OFF
    digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);

    // Save parameters to EEPROM only if any of them changed
    bool write_eeprom = false;

    if (eepromConfig.wifi_user != custom_wifi_user.getValue())
    {
      strncpy(eepromConfig.wifi_user, custom_wifi_user.getValue(), sizeof(eepromConfig.wifi_user));
      eepromConfig.wifi_user[sizeof(eepromConfig.wifi_user) - 1] = '\0';
      write_eeprom = true;
      Serial.print("WiFi user: ");
      Serial.println(eepromConfig.wifi_user);
    }
    if (eepromConfig.wifi_password != custom_wifi_password.getValue())
    {
      strncpy(eepromConfig.wifi_password, custom_wifi_password.getValue(), sizeof(eepromConfig.wifi_password));
      eepromConfig.wifi_password[sizeof(eepromConfig.wifi_password) - 1] = '\0';
      write_eeprom = true;
      Serial.print("WiFi password: ");
      Serial.println(eepromConfig.wifi_password);
    }
    if (eepromConfig.MQTT_server != custom_mqtt_server.getValue())
    {
      strncpy(eepromConfig.MQTT_server, custom_mqtt_server.getValue(), sizeof(eepromConfig.MQTT_server));
      eepromConfig.MQTT_server[sizeof(eepromConfig.MQTT_server) - 1] = '\0';
      write_eeprom = true;
      Serial.print("MQTT server: ");
      Serial.println(eepromConfig.MQTT_server);
    }

    if (eepromConfig.MQTT_port != atoi(custom_mqtt_port.getValue()))
    {
      eepromConfig.MQTT_port = atoi(custom_mqtt_port.getValue());
      //strncpy(eepromConfig.MQTT_port, custom_mqtt_port.getValue(), sizeof(eepromConfig.MQTT_port));
      //eepromConfig.MQTT_port[sizeof(eepromConfig.MQTT_port) - 1] ='\0';
      write_eeprom = true;
      Serial.print("MQTT port: ");
      Serial.println(eepromConfig.MQTT_port);
    }

    if (write_eeprom)
    {
      Write_EEPROM();
    }

    // Reset
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up

    // so resetting the device allows to go back into config mode again when it reboots.
    delay(1000);
  }

  // Normal start, without captive portal
  else
  {

    // Put anaire.org in the display
    display.flipScreenVertically();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_24);
    display.drawString(64, 4, "anaire.org");
    display.display(); // update OLED display
    delay(1000);       // to show anaire.org
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, String(anaire_device_id));
    display.drawString(0, 16, sw_version);
    display.display(); // update OLED display
    delay(1000);

    // Attempt to connect to WiFi network:
    Connect_WiFi();

    // Attempt to connect to MQTT broker
    if (!err_wifi)
    {
      Init_MQTT();
    }

    // Initialize and warm up device sensors
    Setup_sensors();

    // Init loops
    measurements_loop_start = millis();
    MQTT_loop_start = millis();
    errors_loop_start = millis();

    // Set DEVICE_STATUS_BUILTIN_LED to OFF when finishing initializing
    digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

  // If a firmware update is in progress do not do anything else
  if (!updating)
  {

    //Serial.println ("--- LOOP BEGIN ---");

    // Measurement loop
    if ((millis() - measurements_loop_start) >= measurements_loop_duration)
    {

      // New timestamp for the loop start time
      measurements_loop_start = millis();

      // Read sensors
      Read_Sensors();

      // Evaluate CO2 value
      Evaluate_CO2_Value(); // this is to avoud to refresh the display with the las co2 measurement value

      // Accumulates samples
      CO2ppm_accumulated += CO2ppm_value;
      CO2ppm_samples++;
    }

    // MQTT loop
    if ((millis() - MQTT_loop_start) >= MQTT_loop_duration)
    {

      // New timestamp for the loop start time
      MQTT_loop_start = millis();

      // Message the MQTT broker in the cloud app to send the measured values
      if (!err_wifi)
      {
        Send_Message_Cloud_App_MQTT();
      }

      // Reset samples
      CO2ppm_accumulated = 0;
      CO2ppm_samples = 0;
    }

    // if FLASH button was pressed flag to updated OLED was set; the display cannot be updated from the ISR, but from main loop after this flag is set
    if (((update_OLED_co2_flag) && ((millis() - flash_button_press_start)) > 3000))
    { // do it only if button was pressed more than 3s ago
      update_OLED_CO2(CO2ppm_value, temperature, humidity);
      update_OLED_co2_flag = false; // reset flag
    }

    // if FLASH button was pressed flag to updated OLED was set; the display cannot be updated from the ISR, but from main loop after this flag is set
    if (update_OLED_status_flag)
    {
      update_OLED_Status();
      update_OLED_status_flag = false; // reset flag
    }

    // if the flash button was pressed more than 10 seconds ago and it is still low, launch calibration
    if (digitalRead(FLASH_BUTTON_GPIO))
    {                                    // button not pressed
      flash_button_pressed_flag = false; // reset flash button press start
    }
    else if ((millis() - flash_button_press_start) > 10000)
    { // button is pressed since >10s
      if (co2_sensor == MHZ14A)
      {
        Calibrate_MHZ14A();
      }
      if (co2_sensor == SCD30_)
      {
        Calibrate_SCD30();
      }
    }

    // Call the double reset detector loop method every so often,so that it can recognise when the timeout expires.
    // You can also call drd.stop() when you wish to no longer consider the next reset as a double reset.
    drd.loop();

    // Process wifi server requests
    Check_WiFi_Server();

    // Errors loop
    if ((millis() - errors_loop_start) >= errors_loop_duration)
    {

      // New timestamp for the loop start time
      errors_loop_start = millis();

      // Try to recover error conditions
      if (err_co2)
      {
        Serial.println("--- err_co2");
        Setup_sensors(); // Init co2 sensors
      }

      if ((err_wifi) || (WiFi.status() != WL_CONNECTED))
      {
        Serial.println("--- err_wifi");
        err_wifi = true;
        Connect_WiFi(); // Attempt to connect to WiFi network:
      }

      //Reconnect MQTT if needed
      if ((!MQTT_client.connected()) && (!err_wifi))
      {
        Serial.println("--- err_mqtt");
        err_MQTT = true;
      }

      //Reconnect MQTT if needed
      if ((err_MQTT) && (!err_wifi))
      {
        Serial.println("--- MQTT reconnect");
        // Attempt to connect to MQTT broker
        //MQTTReconnect();
        Init_MQTT();
      }

      // if not there are not connectivity errors, receive MQTT messages
      if ((!err_MQTT) && (!err_wifi))
      {
        MQTT_client.loop();
      }
    }

    //Serial.println("--- END LOOP");
    //Serial.println();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

// ISR to respond to button pressing to toggle local alarm reporting through builtin LED and buzzer
// Enters each time the button is pressed
ICACHE_RAM_ATTR void push_button_handler()
{

  flash_button_press_start = millis(); // Save press button start time to keep a defined time the device info on screen, and launch calibration if after 3 seconds the button si still pressed (LOW)
  flash_button_pressed_flag = true;

  //Serial.print ("FLASH Push button start: ");
  //Serial.println (flash_button_press_start);

  if (!alarm_ack)
  {
    Serial.println("FLASH Push button interrupt - alarm_ack ON");
    // Switch off the buzzer to stop the sound alarm
    blinker_BUZZER_GPIO.detach();
    digitalWrite(BUZZER_GPIO, LOW);
    alarm_ack = true; // alarm has been ack
  }

  else
  {
    Serial.println("FLASH Push button interrupt - alarm_ack OFF");
    alarm_ack = false; // alarm has been reset
    // Evaluate last CO2 measurement and update buzzer accordingly
    Evaluate_CO2_Value();
  }

  // Set flag to print device info in OLED display from the control loop
  update_OLED_status_flag = true;
}
