#include "mqtt.h"

void Init_MQTT()
{
  Serial.print("Attempting to connect to the MQTT broker ");
  Serial.print(eepromConfig.MQTT_server);
  Serial.print(":");
  Serial.println(eepromConfig.MQTT_port);

  // Attempt to connect to MQTT broker
  MQTT_client.setBufferSize(512); // to receive messages up to 512 bytes length (default is 256)
  MQTT_client.setServer(eepromConfig.MQTT_server, eepromConfig.MQTT_port);
  MQTT_client.setCallback(Receive_Message_Cloud_App_MQTT);
  MQTT_client.connect(anaire_device_id.c_str());

  if (!MQTT_client.connected())
  {
    err_MQTT = true;
    MQTTReconnect();
  }
  else
  {
    err_MQTT = false;
    lastReconnectAttempt = 0;
    // Once connected resubscribe
    MQTT_client.subscribe(MQTT_receive_topic.c_str());
    Serial.print("MQTT connected - Receive topic: ");
    Serial.println(MQTT_receive_topic);
  }
}

#define MOCK_ALL
// Send measurements to the cloud application by MQTT
void Send_Message_Cloud_App_MQTT()
{

  // Print info
  Serial.print("Sending MQTT message to the send topic: ");
  Serial.println(MQTT_send_topic);
  randomSeed(analogRead(0));

  //sprintf(MQTT_message, "{id: %s,CO2: %d,humidity: %f,temperature: %f}", anaire_device_id.c_str(), (int)random(300, 900), (double)random(3000, 9000) / 100, (double)random(3000, 9000) / 100);
  sprintf(MQTT_message, "{id: %s,CO2: %d,humidity: %f,temperature: %f}", anaire_device_id.c_str(), (int)(CO2ppm_accumulated / CO2ppm_samples), humidity, temperature);
  Serial.print(MQTT_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents
  MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);
}

// callback function to receive configuration messages from the cloud application by MQTT
void Receive_Message_Cloud_App_MQTT(char *topic, byte *payload, unsigned int length)
{

  //StaticJsonDocument<300> jsonBuffer;
  boolean write_eeprom = false; // to track if writing the eeprom is required

  memcpy(received_payload, payload, length);

  Serial.print("Message arrived: ");
  Serial.println(received_payload);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(jsonBuffer, received_payload);

  // Test if parsing succeeds.
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Update name
  if ((jsonBuffer["name"]) && (eepromConfig.anaire_device_name != jsonBuffer["name"]))
  {
    strncpy(eepromConfig.anaire_device_name, jsonBuffer["name"].as<const char *>(), sizeof(eepromConfig.anaire_device_name));
    eepromConfig.anaire_device_name[sizeof(eepromConfig.anaire_device_name) - 1] = '\0';
    write_eeprom = true;
    Serial.print("Anaire device name: ");
    Serial.println(eepromConfig.anaire_device_name);
  }

  // Update warning threshold
  if ((jsonBuffer["warning"]) && (eepromConfig.CO2ppm_warning_threshold != (int)jsonBuffer["warning"]))
  {
    eepromConfig.CO2ppm_warning_threshold = (int)jsonBuffer["warning"];
    Evaluate_CO2_Value();
    write_eeprom = true;
    Serial.print("New warning threshold: ");
    Serial.println(eepromConfig.CO2ppm_warning_threshold);
  }

  // Update alarm threshold
  if ((jsonBuffer["caution"]) && (eepromConfig.CO2ppm_alarm_threshold != (int)jsonBuffer["caution"]))
  {
    eepromConfig.CO2ppm_alarm_threshold = (int)jsonBuffer["caution"];
    Evaluate_CO2_Value();
    write_eeprom = true;
    Serial.print("New alarm threshold: ");
    Serial.println(eepromConfig.CO2ppm_alarm_threshold);
  }

  // Update sound alarm
  if ((jsonBuffer["alarm"]) && ((eepromConfig.sound_alarm) && (jsonBuffer["alarm"] == "OFF")))
  {
    eepromConfig.sound_alarm = false;
    write_eeprom = true;
    blinker_BUZZER_GPIO.detach();   // stop buzzer blinking
    digitalWrite(BUZZER_GPIO, LOW); // update BUZZER_GPIO to LOW to stop sound
    Serial.println("Alarm sound value: OFF");
  }

  if ((jsonBuffer["alarm"]) && ((!eepromConfig.sound_alarm) && (jsonBuffer["alarm"] == "ON")))
  {
    eepromConfig.sound_alarm = true;
    Evaluate_CO2_Value();
    write_eeprom = true;
    Serial.println("Alarm sound value: ON");
  }

  // Check MQTT server
  if ((jsonBuffer["MQTT_server"]) && (eepromConfig.MQTT_server != jsonBuffer["MQTT_server"]))
  {
    strncpy(eepromConfig.MQTT_server, jsonBuffer["MQTT_server"], sizeof(eepromConfig.MQTT_server));
    eepromConfig.MQTT_server[sizeof(eepromConfig.MQTT_server) - 1] = '\0';
    write_eeprom = true;
    Serial.print("MQTT Server: ");
    Serial.println(eepromConfig.MQTT_server);

    //Attempt to connect to MQTT broker
    if (!err_wifi)
    {
      Init_MQTT();
    }
  }

  // Check MQTT port
  if ((jsonBuffer["MQTT_port"]) && (eepromConfig.MQTT_port != int(jsonBuffer["MQTT_port"])))
  {
    eepromConfig.MQTT_port = int(jsonBuffer["MQTT_port"]);
    //strncpy(eepromConfig.MQTT_port, jsonBuffer["MQTT_port"], sizeof(eepromConfig.MQTT_port));
    //eepromConfig.MQTT_port[sizeof(eepromConfig.MQTT_port) - 1] = '\0';
    write_eeprom = true;
    Serial.print("MQTT Port: ");
    Serial.println(eepromConfig.MQTT_port);
    // Attempt to connect to MQTT broker
    if (!err_wifi)
    {
      Init_MQTT();
    }
  }

  // Check FRC value
  if ((jsonBuffer["FRC_value"]) && (eepromConfig.FRC_value != (uint16_t)jsonBuffer["FRC_value"]))
  {
    eepromConfig.FRC_value = (uint16_t)jsonBuffer["FRC_value"];
  }

  // Check temperature offset
  if ((jsonBuffer["temperature_offset"]) && (eepromConfig.temperature_offset != (uint16_t)jsonBuffer["temperature_offset"]))
  {
    eepromConfig.temperature_offset = (uint16_t)jsonBuffer["temperature_offset"];
    if (co2_sensor == MHZ14A)
    {
      //MHZ14A_Do_Temperature_Offset();
    }
    if (co2_sensor == SCD30_)
    {
      SCD30_Do_Temperature_Offset();
    }
  }

  // Check altitude_compensation
  if ((jsonBuffer["altitude_compensation"]) && (eepromConfig.altitude_compensation != (uint16_t)jsonBuffer["altitude_compensation"]))
  {
    eepromConfig.altitude_compensation = (uint16_t)jsonBuffer["altitude_compensation"];
    if (co2_sensor == MHZ14A)
    {
      //MHZ14A_Do_Altitude_Compensation();
    }
    if (co2_sensor == SCD30_)
    {
      SCD30_Do_Altitude_Compensation();
    }
  }

  // If calibration has been enabled, justo do it
  if ((jsonBuffer["FRC"]) && (jsonBuffer["FRC"] == "ON"))
  {
    if (co2_sensor == MHZ14A)
    {
      Calibrate_MHZ14A();
    }
    if (co2_sensor == SCD30_)
    {
      Calibrate_SCD30();
    }
  }

  // Update ABC Automatic Baseline Calibration
  if ((jsonBuffer["ABC"]) && ((eepromConfig.ABC) && (jsonBuffer["ABC"] == "OFF")))
  {
    eepromConfig.ABC = false;
    write_eeprom = true;
    if (co2_sensor == MHZ14A)
    {
      //MHZ14A_Do_AutoSelfCalibration()
    }
    if (co2_sensor == SCD30_)
    {
      SCD30_Do_AutoSelfCalibration();
    }
    Serial.println("ABC: OFF");
  }

  if ((jsonBuffer["ABC"]) && ((!eepromConfig.ABC) && (jsonBuffer["ABC"] == "ON")))
  {
    eepromConfig.ABC = true;
    write_eeprom = true;
    if (co2_sensor == MHZ14A)
    {
      //MHZ14A_Do_AutoSelfCalibration()
    }
    if (co2_sensor == SCD30_)
    {
      SCD30_Do_AutoSelfCalibration();
    }
    Serial.println("ABC: ON");
  }

  // if update flag has been enabled, wipe EEPROM and update to latest bin
  if (((jsonBuffer["update"]) && (jsonBuffer["update"] == "ON")))
  {
    //boolean result = EEPROM.wipe();
    //if (result) {
    //  Serial.println("All EEPROM data wiped");
    //} else {
    //  Serial.println("EEPROM data could not be wiped from flash store");
    //}

    // update firmware to latest bin
    Serial.println("Update firmware to latest bin");
    firmware_update();
  }

  // If factory reset has been enabled, just do it
  if ((jsonBuffer["factory_reset"]) && (jsonBuffer["factory_reset"] == "ON"))
  {
    Wipe_EEPROM(); // Wipe EEPROM
    ESP.reset();   // This is a bit crude. For some unknown reason webserver can only be started once per boot up
  }

  // If reboot, just do it, without cleaning the EEPROM
  if ((jsonBuffer["reboot"]) && (jsonBuffer["reboot"] == "ON"))
  {
    ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up
  }

  //print info
  Serial.println("MQTT update - message processed");
  Print_Config();

  // save the new values
  if (write_eeprom)
  {
    Write_EEPROM();
  }
}

// MQTT reconnect function
void MQTTReconnect()
{
  //Try to reconnect only if it has been more than 5 sec since last attemp
  unsigned long now = millis();
  if (now - lastReconnectAttempt > 5000)
  {
    lastReconnectAttempt = now;
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (MQTT_client.connect(anaire_device_id.c_str()))
    {
      err_MQTT = false;
      Serial.println("MQTT connected");
      lastReconnectAttempt = 0;
      // Once connected resubscribe
      MQTT_client.subscribe(MQTT_receive_topic.c_str());
      Serial.print("MQTT connected - Receive topic: ");
      Serial.println(MQTT_receive_topic);
    }
    else
    {
      err_MQTT = true;
      Serial.print("failed, rc=");
      Serial.print(MQTT_client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

//FIRMWARE STUFF:
void firmware_update()
{

  // For remote firmware update
  BearSSL::WiFiClientSecure UpdateClient;
  int freeheap = ESP.getFreeHeap();

  Serial.println("### FIRMWARE UPGRADE ###");

  // Add optional callback notifiers
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);
  UpdateClient.setInsecure();

  // Try to set a smaller buffer size for BearSSL update
  bool mfln = UpdateClient.probeMaxFragmentLength("raw.githubusercontent.com", 443, 512);
  Serial.printf("\nConnecting to https://raw.githubusercontent.com\n");
  Serial.printf("MFLN supported: %s\n", mfln ? "yes" : "no");
  if (mfln)
  {
    UpdateClient.setBufferSizes(512, 512);
  }
  UpdateClient.connect("raw.githubusercontent.com", 443);
  if (UpdateClient.connected())
  {
    Serial.printf("MFLN status: %s\n", UpdateClient.getMFLNStatus() ? "true" : "false");
    Serial.printf("Memory used: %d\n", freeheap - ESP.getFreeHeap());
    freeheap -= ESP.getFreeHeap();
  }
  else
  {
    Serial.printf("Unable to connect\n");
  }

  // Run http update
  //t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/src/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/anaireorg/anaire-devices/main/Anaire.30ppm-50ppm/anaire-device.NodeMCULuaAmicaV2/anaire-device.NodeMCULuaAmicaV2.ino.nodemcu.bin");

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void update_started()
{
  Serial.println("CALLBACK:  HTTP update process started");
  updating = true;
}

void update_finished()
{
  Serial.println("CALLBACK:  HTTP update process finished");
  Serial.println("### FIRMWARE UPGRADE COMPLETED - REBOOT ###");
  updating = false;
}

void update_progress(int cur, int total)
{
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err)
{
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
  updating = false;
}
