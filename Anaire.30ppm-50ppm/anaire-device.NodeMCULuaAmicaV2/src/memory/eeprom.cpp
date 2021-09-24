#include "eeprom.h"

void Read_EEPROM()
{

  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyEEPROMStruct));
  String _anaire_device_id = String(ESP.getChipId(), HEX); // HEX version, for easier match to mac address

  // Wipe EEPROM
  /*
    boolean result = EEPROM.wipe();
    if (result) {
    Serial.println("All EEPROM data wiped");
    } else {
    Serial.println("EEPROM data could not be wiped from flash store");
    }
  */

  // Check if the EEPROM contains valid data from another run
  // If so, overwrite the 'default' values set up in our struct
  if (EEPROM.percentUsed() >= 0)
  {

    Serial.println("EEPROM has data from a previous run.");
    Serial.print(EEPROM.percentUsed());
    Serial.println("% of ESP flash space currently used");

    // Read saved data
    EEPROM.get(0, eepromConfig);
    Print_Config();
  }
  else
  {
    _anaire_device_id.toCharArray(eepromConfig.anaire_device_name, sizeof(eepromConfig.anaire_device_name));
    Serial.println("No EEPROM data - using default config values");
  }
}

void Write_EEPROM()
{

  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyEEPROMStruct));

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromConfig);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "EEPROM Commit OK" : "EEPROM Commit failed");
}

void Wipe_EEPROM()
{
  boolean result = EEPROM.wipe();
  if (result)
  {
    Serial.println("All EEPROM data wiped");
  }
  else
  {
    Serial.println("EEPROM data could not be wiped from flash store");
  }
}

void Print_Config()
{
  //TODO: remove this, move to common/vars file
  String _anaire_device_id = String(ESP.getChipId(), HEX); // HEX version, for easier match to mac address
  String _wifi_ssid = WiFi.SSID();    // your network SSID (name)

  Serial.println("#######################################");
  Serial.print("device id: ");
  Serial.println(_anaire_device_id);
  Serial.print("anaire device name: ");
  Serial.println(eepromConfig.anaire_device_name);
  Serial.print("SW version: ");
  Serial.println(sw_version);
  Serial.print("WiFi SSID: ");
  Serial.println(_wifi_ssid);
  Serial.print("WiFi user: ");
  Serial.println(eepromConfig.wifi_user);
  Serial.print("WiFi password: ");
  Serial.println(eepromConfig.wifi_password);
  Serial.print("CO2ppm Warning threshold: ");
  Serial.println(eepromConfig.CO2ppm_warning_threshold);
  Serial.print("CO2ppm Alarm threshold: ");
  Serial.println(eepromConfig.CO2ppm_alarm_threshold);
  Serial.print("MQTT server: ");
  Serial.println(eepromConfig.MQTT_server);
  Serial.print("MQTT Port: ");
  Serial.println(eepromConfig.MQTT_port);
  Serial.print("Sound Alarm: ");
  Serial.println(eepromConfig.sound_alarm);
  Serial.print("Automatic Baseline Correction: ");
  Serial.println(eepromConfig.ABC);
  Serial.print("Forced Recalibration Value: ");
  Serial.println(eepromConfig.FRC_value);
  Serial.print("Temperature Offset: ");
  Serial.println(eepromConfig.temperature_offset);
  Serial.print("Altitude Compensation: ");
  Serial.println(eepromConfig.altitude_compensation);
  Serial.println("#######################################");
}