#include "wifi.h"


// Connect to WiFi network
void Connect_WiFi()
{

  Serial.print("Attempting to connect to Network named: ");
  Serial.println( WiFi.SSID()); // print the network name (SSID);

  // Set wifi mode
  WiFi.mode(WIFI_STA);

  // If there are not wifi user and wifi password defined, proceed to traight forward configuration
  if ((strlen(eepromConfig.wifi_user) == 0) && (strlen(eepromConfig.wifi_password) == 0))
  {

    //WiFi.begin(wifi_ssid, wifi_password);
    WiFi.begin();
  }

  else
  { // set up wpa2 enterprise

    Serial.print("Attempting to authenticate with WPA Enterprise ");
    Serial.print("User: ");
    Serial.println(eepromConfig.wifi_user);
    Serial.print("Password: ");
    Serial.println(eepromConfig.wifi_password);

    // Setting ESP into STATION mode only (no AP mode or dual mode)
    wifi_set_opmode(STATION_MODE);

    struct station_config wifi_config;

    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char *)wifi_config.ssid,  WiFi.SSID().c_str());
    strcpy((char *)wifi_config.password, wifi_password.c_str());

    wifi_station_set_config(&wifi_config);
    //uint8_t target_esp_mac[6] = {0x24, 0x0a, 0xc4, 0x9a, 0x58, 0x28};
    //wifi_set_macaddr(STATION_IF,target_esp_mac);
    wifi_station_set_wpa2_enterprise_auth(1);

    // Clean up to be sure no old data is still inside
    wifi_station_clear_cert_key();
    wifi_station_clear_enterprise_ca_cert();
    wifi_station_clear_enterprise_identity();
    wifi_station_clear_enterprise_username();
    wifi_station_clear_enterprise_password();
    wifi_station_clear_enterprise_new_password();

    // Set up authentication
    //wifi_station_set_enterprise_identity((uint8*)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_username((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_password((uint8 *)eepromConfig.wifi_password, strlen((char *)eepromConfig.wifi_password));

    wifi_station_connect();
  }

  // Timestamp for connection timeout
  int wifi_timeout_start = millis();

  // Wait for warming time while blinking blue led
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifi_timeout_start) < WIFI_CONNECT_TIMEOUT))
  {
    delay(500); // wait 0.5 seconds for connection
    Serial.print(".");
  }

  // Status
  if (WiFi.status() != WL_CONNECTED)
  {
    err_wifi = true;
  }
  else
  {
    err_wifi = false;

    wifi_server.begin(); // start the web server on port 80
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Print your WiFi shield's MAC address:
    Serial.print("MAC Adress: ");
    Serial.println(WiFi.macAddress());

    // Set mDNS to anaire_device_id.local
    if (!MDNS.begin(String(anaire_device_id)))
    {
      Serial.println("Error mDNS");
    }
    else
    {
      Serial.print("Hostname: ");
      Serial.print(String(anaire_device_id));
      Serial.println(".local");
    }
  }
}


// Print wifi status on serial monitor
void Print_WiFi_Status()
{

  // Get current status
  //  WL_CONNECTED: assigned when connected to a WiFi network;
  //  WL_NO_SHIELD: assigned when no WiFi shield is present;
  //  WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
  //  WL_NO_SSID_AVAIL: assigned when no SSID are available;
  //  WL_SCAN_COMPLETED: assigned when the scan networks is completed;
  //  WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
  //  WL_CONNECTION_LOST: assigned when the connection is lost;
  //  WL_DISCONNECTED: assigned when disconnected from a network;

  //wifi_status = WiFi.status();
  Serial.print("wifi_status: ");
  Serial.println(WiFi.status());

  // Print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  //ip_address = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Print your WiFi shield's MAC address:
  Serial.print("MAC Adress: ");
  Serial.println(WiFi.macAddress());

  // Print the received signal strength:
  //wifi_rssi_dbm = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void Check_WiFi_Server()
{
  WiFiClient client = wifi_server.available(); // listen for incoming clients
  if (client)
  {                               // if you get a client,
    Serial.println("new client"); // print a message out the serial port
    String currentLine = "";      // make a String to hold incoming data from the client
    while (client.connected())
    { // loop while the client's connected
      if (client.available())
      {                         // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        Serial.write(c);        // print it out the serial monitor
        if (c == '\n')
        { // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // Print current info
            client.print("Anaire Device ID: ");
            client.print(anaire_device_id);
            client.println("<br>");
            client.print("Anaire Device name: ");
            client.print(eepromConfig.anaire_device_name);
            client.println("<br>");
            client.print("SW version: ");
            client.print(sw_version);
            client.println("<br>");
            client.print("MAC Adress: ");
            client.print(WiFi.macAddress());
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("CO2ppm_alarm_threshold: ");
            client.print(eepromConfig.CO2ppm_alarm_threshold);
            client.println("<br>");
            client.print("CO2ppm_warning_threshold: ");
            client.print(eepromConfig.CO2ppm_warning_threshold);
            client.println("<br>");
            client.print("MQTT Server: ");
            client.print(eepromConfig.MQTT_server);
            client.println("<br>");
            client.print("MQTT Port: ");
            client.print(eepromConfig.MQTT_port);
            client.println("<br>");
            client.print("Sound alarm: ");
            client.print(eepromConfig.sound_alarm);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            if (co2_sensor == SCD30_)
            {
              client.print("CO2 Sensor: Sensirion SCD30");
              client.println("<br>");
              uint16_t val;
              //SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO, false);
              airSensor.getMeasurementInterval(&val);
              client.print("SCD30 Measurement Interval: ");
              client.print(val);
              client.println("<br>");
              client.print("SCD30 AutoselfCalibration: ");
              client.print(airSensor.getAutoSelfCalibration());
              client.println("<br>");
              airSensor.getForceRecalibration(&val);
              client.print("SCD30 Force Recalibration: ");
              client.print(val);
              client.println("<br>");
              airSensor.getTemperatureOffset(&val);
              client.print("SCD30 Temperature Offset: ");
              client.print(val);
              client.println("<br>");
              airSensor.getAltitudeCompensation(&val);
              client.print("SCD30 AltitudeCompensation: ");
              client.print(val);
              client.println("<br>");
            }
            else
            {
              client.print("CO2 Sensor: Winsen MH-Z14A/Z19x");
              client.println("<br>");
              char myVersion[4];
              myMHZ19.getVersion(myVersion);
              client.print("\nFirmware Version: ");
              for (byte i = 0; i < 4; i++)
              {
                client.print(myVersion[i]);
                if (i == 1)
                  client.print(".");
              }
              client.println("<br>");
              client.print("Range: ");
              client.println(myMHZ19.getRange());
              client.println("<br>");
              client.print("Accuracy: ");
              client.println(myMHZ19.getAccuracy(true)); // force to get the value, even if nor available
              client.println("<br>");
              client.print("Background CO2: ");
              client.println(myMHZ19.getBackgroundCO2());
              client.println("<br>");
              client.print("Temperature Cal: ");
              client.println(myMHZ19.getTempAdjustment());
              client.println("<br>");
              client.print("ABC Status: ");
              myMHZ19.getABC() ? client.println("ON") : client.println("OFF");
              client.println("<br>");
              client.print("Temperature: ");
              client.println(myMHZ19.getTemperature(true, true)); // force to get the value in float
              client.println("<br>");
              client.print("CO2Raw: ");
              client.println(myMHZ19.getCO2Raw(true)); // force to get the value in float
              client.println("<br>");
            }
            client.println("------");
            client.println("<br>");
            client.print("CO2 PPM: ");
            client.print(CO2ppm_value);
            client.println("<br>");
            client.print("Temperature: ");
            client.print(temperature);
            client.println("<br>");
            client.print("Humidity: ");
            client.print(humidity);
            client.println("<br>");
            client.print("CO2 STATUS: ");
            switch (co2_device_status)
            {
            case ok:
              client.print("OK");
              break;
            case warning:
              client.print("WARNING");
              break;
            case alarm:
              client.print("ALARM");
              break;
            }
            client.println("<br>");
            client.println("------");
            client.println("<br>");

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to calibrate the device.<br>");
            //client.print("Click <a href=\"/L\">here</a> turn the LED on pin 6 off<br>");

            client.println("<br>");
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else
          { // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r')
        {                   // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H"))
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
        if (currentLine.endsWith("GET /L"))
        {
          //digitalWrite(LED, LOW);                // GET /L turns the LED off
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}