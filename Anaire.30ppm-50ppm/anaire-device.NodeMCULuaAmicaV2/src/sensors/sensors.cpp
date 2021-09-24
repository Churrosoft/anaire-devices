#include "sensors.h"


// Evaluate CO2 value versus warning and alarm threasholds and process CO2 alarm information
void Evaluate_CO2_Value()
{

  // Status: ok

  // Recovering to "ok" status stops any warning or alarm warnings and quits special mode (after pressing flash button), therefore after passing by "ok" status
  // the device is "reset" and when entering warning or alarm state the device will report localy again by blinking o2_builtin_led_gpio16 led and the buzzer,
  // and pushing the flash button will be required if the user wanrts to stop lignt and sound alerting

  if (CO2ppm_value < eepromConfig.CO2ppm_warning_threshold)
  {

    co2_device_status = ok; // update co2 status
    alarm_ack = false;      // Init alarm ack status

    blinker_CO2_STATUS_BUILTIN_LED_GPIO.detach();    // stop CO2_STATUS_BUILTIN_LED_GPIO blinking
    digitalWrite(CO2_STATUS_BUILTIN_LED_GPIO, HIGH); // update CO2_STATUS_BUILTIN_LED_GPIO_gpio1 to OFF

    blinker_BUZZER_GPIO.detach();   // stop buzzer blinking
    digitalWrite(BUZZER_GPIO, LOW); // update BUZZER_GPIO to LOW to stop sound
  }

  // Status: warning
  else if ((CO2ppm_value >= eepromConfig.CO2ppm_warning_threshold) && (CO2ppm_value < eepromConfig.CO2ppm_alarm_threshold))
  {

    co2_device_status = warning; // update device status

    blinker_CO2_STATUS_BUILTIN_LED_GPIO.attach_ms(WARNING_BLINK_PERIOD, changeState_CO2_STATUS_BUILTIN_LED_GPIO); // warning blink of light on CO2 LED

    if ((!alarm_ack) && (eepromConfig.sound_alarm))
    {                                                                               // flash button hasn't been pressed to disable and sound_alarm is enabled
      blinker_BUZZER_GPIO.attach_ms(WARNING_BLINK_PERIOD, changeState_BUZZER_GPIO); // warning sound on buzzer
    }
  }

  // Status: alarm
  else
  {

    co2_device_status = alarm; // update device status

    blinker_CO2_STATUS_BUILTIN_LED_GPIO.attach_ms(ALARM_BLINK_PERIOD, changeState_CO2_STATUS_BUILTIN_LED_GPIO); // warning blink of light on CO2 LED

    if ((!alarm_ack) && (eepromConfig.sound_alarm))
    {                                                                             // flash button hasn't been pressed to disable and sound_alarm is enabled
      blinker_BUZZER_GPIO.attach_ms(ALARM_BLINK_PERIOD, changeState_BUZZER_GPIO); // alarm sound on buzzer
    }
  }

  // Print info on serial monitor
  switch (co2_device_status)
  {
  case ok:
    Serial.println("STATUS: CO2 OK");
    break;
  case warning:
    Serial.println("STATUS: CO2 WARNING");
    break;
  case alarm:
    Serial.println("STATUS: CO2 ALARM");
    break;
  }

  // Update display OLED with new values
  update_OLED_co2_flag = true;
}


// TODO: MOVE TO ANOTHER FILE:


// Read temperature and humidity values from DHT11
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
void Read_DHT11()
{

  TempAndHumidity lastValues = dht.getTempAndHumidity();

  // Read humidity as percentage
  humidity = lastValues.humidity;

  // Read temperature as Celsius
  temperature = lastValues.temperature;

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    err_dht = true;
    humidity = 0;
    temperature = 0;
  }
  else
  {
    err_dht = false;
    Serial.print("DHT11 Humidity: ");
    Serial.print(humidity);
    Serial.print(" % \n");
    Serial.print("DHT11 Temperature: ");
    Serial.print(temperature);
    Serial.println("ºC");
  }
}

void SCD30_Do_AutoSelfCalibration()
{
  Serial.print("\nReading SCD30 AutoSelfCalibration before change: ");
  Serial.println(airSensor.getAutoSelfCalibration());
  Serial.print("Setting new SCD30 AutoSelfCalibration to: ");
  Serial.println(eepromConfig.ABC);

  if (airSensor.setAutoSelfCalibration(eepromConfig.ABC))
  {
    Serial.print("Reading SCD30 AutoSelfCalibration after change: ");
    Serial.println(airSensor.getAutoSelfCalibration());
  }
  else
  {
    Serial.println("Could not set new SCD30 AutoSelfCalibration");
  }
}

void SCD30_Do_Temperature_Offset()
{
  uint16_t val;

  if (airSensor.getTemperatureOffset(&val))
  {

    Serial.print("\nReading SCD30 Temperature Offset before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Temperature Offset to: ");
    Serial.println(eepromConfig.temperature_offset);

    if (airSensor.setTemperatureOffset(eepromConfig.temperature_offset))
    {

      if (airSensor.getTemperatureOffset(&val))
      {
        Serial.print("Reading SCD30 Temperature Offset after making change: ");
        Serial.println(val);
      }
      else
      {
        Serial.println("Could not obtain SCD30 Temperature Offset");
      }
    }
    else
    {
      Serial.println("Could not set new SCD30 Temperature Offset");
    }
  }
  else
  {
    Serial.println("Could not obtain Temperature Offset");
  }
}

void SCD30_Do_Measurement_Interval()
{
  uint16_t val;

  if (airSensor.getMeasurementInterval(&val))
  {
    Serial.print("\nReading SCD30 Measurement Interval before change: ");
    Serial.println(val);

    Serial.print("Setting SCD30 new Measurement Interval to: ");
    Serial.println(SCD30_MEASUREMENT_INTERVAL);

    if (airSensor.setMeasurementInterval(SCD30_MEASUREMENT_INTERVAL))
    {
      Serial.print("Change SCD30 Measurement Interval to: ");
      Serial.println(SCD30_MEASUREMENT_INTERVAL);

      if (airSensor.getMeasurementInterval(&val))
      {
        Serial.print("Reading SCD30 Measurement Interval after change: ");
        Serial.println(val);
      }
      else
      {
        Serial.println("Could not obtain SCD30 Measurement Interval");
      }
    }
    else
    {
      Serial.print("Could not change SCD30 Measurement Interval to ");
      Serial.println(SCD30_MEASUREMENT_INTERVAL);
    }
  }
  else
  {
    Serial.println("Could not obtain SCD30 Measurement Interval");
  }
}

void SCD30_Do_Forced_Calibration_Factor()
{
  uint16_t val;
  bool result;

  if (airSensor.getForceRecalibration(&val))
  {
    Serial.print("\nReading SCD30 Forced Calibration Factor before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Forced Calibration Factor to: ");
    Serial.println(eepromConfig.FRC_value);

    result = airSensor.setForceRecalibration(eepromConfig.FRC_value);
    Serial.print("\nResponse after sending FRC command: ");
    Serial.println(result);
    delay(1000);

    if (airSensor.getForceRecalibration(&val))
    {
      Serial.print("Reading SCD30 Forced Calibration Factor after change: ");
      Serial.println(val);
    }
    else
    {
      Serial.println("Could not obtain SCD30 forced calibration factor");
    }
  }
  else
  {
    Serial.println("Could not obtain SCD30 Forced Calibration Factor");
  }
}

void SCD30_Do_Altitude_Compensation()
{

  /* paulvha : you can set EITHER the Altitude compensation of the pressure.
     Setting both does not make sense as both overrule each other, but it is included for demonstration

     see Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf

        The CO2 measurement value can be compensated for ambient pressure by feeding the pressure value in mBar to the sensor.
        Setting the ambient pressure will overwrite previous and future settings of altitude compensation. Setting the argument to zero
        will deactivate the ambient pressure compensation. For setting a new ambient pressure when continuous measurement is running
        the whole command has to be written to SCD30.

        Setting altitude is disregarded when an ambient pressure is given to the sensor
  */

  uint16_t val;

  if (airSensor.getAltitudeCompensation(&val))
  {
    Serial.print("\nReading SCD30 Altitude Compensation before change: ");
    Serial.println(val);

    Serial.print("Setting new SCD30 Altitude Compensation to: ");
    Serial.println(eepromConfig.altitude_compensation);

    if (airSensor.setAltitudeCompensation(eepromConfig.altitude_compensation))
    {

      if (airSensor.getAltitudeCompensation(&val))
      {
        Serial.print("Reading SCD30 Altitude Compensation after change: ");
        Serial.println(val);
      }
      else
      {
        Serial.println("Could not obtain SCD30 Altitude Compensation");
      }
    }
    else
    {
      Serial.println("Could not set new SCD30 Altitude Compensation");
    }
  }
  else
  {
    Serial.println("Could not obtain SCD30 Altitude Compensation");
  }
}

void SCD30DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) + 1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 33 digits (32 serial + 0x0)

  if (airSensor.getSerialNumber(buf))
  {
    Serial.print(F("SCD30 serial number: "));
    Serial.println(buf);
  }

  // read Firmware level
  if (airSensor.getFirmwareLevel(val))
  {
    Serial.print("SCD30 Firmware level - Major: ");
    Serial.print(val[0]);

    Serial.print("\t, Minor: ");
    Serial.println(val[1]);
  }
  else
  {
    Serial.println("Could not obtain firmware level");
  }
}


// Calibrate Sensirion SCD30 CO2, humidity and temperature sensor
// Requires that the device is localed on open air for several minutes
void Calibrate_SCD30()
{

  // Print info
  Serial.println();
  Serial.println("Calibrating SCD30 sensor...");

  // Device status LED ON
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  //display.init();
  //display.flipScreenVertically();

  // Wait for calibrating time while reading values at maximum speed
  int counter = SCD30_CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < SCD30_CALIBRATION_TIME)
  {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Calibrando");
    display.drawString(0, 16, "SCD30 " + String(counter));
    display.display(); // update OLED display
    // if not there are not connectivity errors, receive MQTT messages, to be able to interrupt calibration process
    if ((!err_MQTT) && (!err_wifi))
    {
      MQTT_client.loop();
    }
    Serial.print(".");
    delay(500);
    Serial.println(".");
    delay(500);
    counter = counter - 1;
  }

  // Send forced calibration command setting as zero reference value
  SCD30_Do_Forced_Calibration_Factor();

  delay(1000);

  // Print info
  Serial.println("SCD30 sensor calibrated");

  // Device status LED OFF
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);
}

// Read Sensirion SCD30 CO2, humidity and temperature sensor
void Read_SCD30()
{

  // Init I2C bus for SCD30
  //SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO, false);

  if (airSensor.dataAvailable())
  {
    CO2ppm_value = airSensor.getCO2();
    temperature = airSensor.getTemperature();
    humidity = airSensor.getHumidity();

    Serial.print("SCD30 co2(ppm): ");
    Serial.print(CO2ppm_value);

    Serial.print(" temp(C): ");
    Serial.print(temperature, 1);

    Serial.print(" humidity(%): ");
    Serial.print(humidity, 1);

    Serial.println();
  }
  else
  {
    Serial.println("SCD30 No data available");
  }
}

// Read sensors
void Read_Sensors()
{

  switch (co2_sensor)
  {
  case SCD30_:
    Read_SCD30(); // Read co2, temperature and humidity
    break;
  case MHZ14A:
    Read_MHZ14A(); // Read co2 and temperature
    Read_DHT11();  // Read temperature and humidity
    break;
  case none:
    break;
  }
}

// Read MHZ14A CO2 sensor
void Read_MHZ14A()
{

  // Timestamp for serial up start time
  //int serial_up_start = millis();

  // clears out any garbage in the RX buffer
  //while (((mySerial.available()) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
  //  int garbage = mySerial.read();
  //  delay(100);
  //  Serial.println ("Cleaning mySerial data...");
  //}

  // Send out read command to the sensor - 9 bytes
  //mySerial.write(measurement_command, 9);

  // pauses the sketch and waits for the TX buffer to send all its data to the sensor
  //mySerial.flush();

  // Timestamp for serial up start time
  //serial_up_start = millis();

  // pauses the sketch and waits for the sensor response
  //if (((!mySerial.available()) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT))) {
  //  Serial.println ("Waiting for mySerial data...");
  //  delay(1000);
  //}

  // once data is available, it reads it to a variable 9 bytes
  //mySerial.readBytes(response_CO2, 9);

  // calculates CO2ppm value
  //response_CO2_high = (int)response_CO2[2];
  //response_CO2_low = (int)response_CO2[3];
  //CO2ppm_value = (256 * response_CO2_high) + response_CO2_low;

  // WiWaf library
  CO2ppm_value = myMHZ19.getCO2(true);

  if (myMHZ19.errorCode != RESULT_OK) // RESULT_OK is an alias for 1. Either can be used to confirm the response was OK.
  {
    //err_co2 = true;
    Serial.println("Failed to receive CO2 value - Error");
    Serial.print("Response Code: ");
    Serial.println(myMHZ19.errorCode); // Get the Error Code value
  }

  // prints calculated CO2ppm value to serial monitor
  Serial.print("MHZ14A/19c CO2ppm_value: ");
  Serial.println(CO2ppm_value);

  // WiWaf library
  //temperature = (float) myMHZ19.getTemperature(true, true)/10;
  //if(myMHZ19.errorCode != RESULT_OK)              // RESULT_OK is an alias for 1. Either can be used to confirm the response was OK.
  //{
  //  Serial.println("Failed to receive temperature value - Error");
  //  Serial.print("Response Code: ");
  //  Serial.println(myMHZ19.errorCode);          // Get the Error Code value
  //}

  // prints temperature value to serial monitor
  //Serial.print ("MHZ14A/19c temperature: ");
  //Serial.println (temperature);
}

// Calibrate MHZ14A sensor
// Requires that the device is localed on open air for several minutes
void Calibrate_MHZ14A()
{

  // Print info
  Serial.println("Calibrating Winsen MHZ14A/MHZ19x CO2 sensor...");

  // Device status LED ON
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, LOW);

  // Timestamp for calibrating start time
  int calibrating_start = millis();

  //display.init();
  //display.flipScreenVertically();

  // Wait for calibrating time to stabalise...
  int counter = MHZ14A_CALIBRATION_TIME / 1000;
  while ((millis() - calibrating_start) < MHZ14A_CALIBRATION_TIME)
  {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Calibrando");
    display.drawString(0, 16, "MHZ14A " + String(counter));
    display.display(); // update OLED display
    // if not there are not connectivity errors, receive MQTT messages, to be able to interrupt calibration process
    if ((!err_MQTT) && (!err_wifi))
    {
      MQTT_client.loop();
    }
    Serial.print(".");
    delay(500);
    Serial.println(".");
    delay(500);
    counter = counter - 1;
  }

  // Take a reading which be used as the zero point for 400 ppm
  myMHZ19.calibrate();

  // Print info
  Serial.println("MHZ14A CO2 sensor calibrated");

  // Device status LED OFF
  digitalWrite(DEVICE_STATUS_BUILTIN_LED_GPIO, HIGH);
}



// Detect and initialize co2 sensors
void Setup_sensors()
{

  // Try Sensirion SCD-30 first

  // Init I2C bus for SCD30
  SCD30WIRE.begin(SCD30_SDA_GPIO, SCD30_SCK_GPIO);

  if (airSensor.begin(SCD30WIRE) == true)
  {

    Serial.println("Air sensor Sensirion SCD30 detected.");

    co2_sensor = SCD30_;

    // display device info
    SCD30DeviceInfo();

    // Set SCD30 (change in global variables at the start of SCD section in the initial section of this program)
    SCD30_Do_Measurement_Interval();
    SCD30_Do_Temperature_Offset();
    SCD30_Do_Altitude_Compensation();
    SCD30_Do_AutoSelfCalibration();

    // Timestamp for warming up start time
    int warming_up_start = millis();

    // Print info
    Serial.println("Warming up SCD30 sensor...");

    // Wait for warming time while blinking blue led
    int counter = int(SCD30_WARMING_TIME / 1000);

    // Print welcome screen
    display.init();
    display.flipScreenVertically();
    while ((millis() - warming_up_start) < SCD30_WARMING_TIME)
    {
      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 0, "Anaire30ppm");
      display.drawString(0, 10, "ID " + String(anaire_device_id));
      display.drawString(0, 20, String(counter));
      display.display(); // update OLED display
      Serial.print(".");
      delay(500); // wait 500ms
      Serial.println(".");
      delay(500); // wait 500ms
      counter = counter - 1;
    }

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Anaire30ppm");
    display.drawString(0, 10, "ID " + String(anaire_device_id));
    display.drawString(0, 20, String(counter));
    display.display(); // update OLED display

    // Print info
    Serial.println("Sensirion SCD30 CO2 sensor setup complete");
  }

  // Then MHZ14A
  else
  {

    // Initialize serial port to communicate with MHZ14A CO2 sensor. This is a software serial port
    //swSerial.begin(9600, SWSERIAL_8N1, swSerialRX_gpio, swSerialTX_gpio, false, 128);
    //Serial.println("swSerial Txd is on pin: " + String(swSerialTX_gpio));
    //Serial.println("swSerial Rxd is on pin: " + String(swSerialRX_gpio));

    mySerial.begin(MHZ_BAUDRATE); // Uno example: Begin Stream with MHZ19 baudrate
    myMHZ19.begin(mySerial);      // *Important, Pass your Stream reference here

    // Timestamp for serial up start time
    int serial_up_start = millis();

    while (((!mySerial) && ((millis() - serial_up_start) < MHZ14A_SERIAL_TIMEOUT)))
    {
      Serial.println("Attempting to open serial port 1 to communicate to MHZ14A CO2 sensor");
      delay(1000); // wait 1 seconds for connection
    }

    // If the timeout was completed it is an error
    if ((millis() - serial_up_start) > MHZ14A_SERIAL_TIMEOUT)
    {
      err_co2 = true;
      co2_sensor = none;
      return;
    }

    else
    {

      Serial.println("CO2 sensor MH-Z14A/MH-Z19x detected.");

      // Disable ABC
      myMHZ19.autoCalibration(eepromConfig.ABC); // make sure auto calibration is off
      //Serial.print("ABC Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");  // now print it's status

      // Set range to 5000
      //myMHZ19.setRange(5000);
      myMHZ19.setRange(2000);

      char myVersion[4];
      myMHZ19.getVersion(myVersion);

      Serial.print("\nFirmware Version: ");
      for (byte i = 0; i < 4; i++)
      {
        Serial.print(myVersion[i]);
        if (i == 1)
          Serial.print(".");
      }
      Serial.println();
      Serial.print("Range: ");
      Serial.println(myMHZ19.getRange());
      Serial.print("Background CO2: ");
      Serial.println(myMHZ19.getBackgroundCO2());
      Serial.print("Temperature Cal: ");
      Serial.println(myMHZ19.getTempAdjustment());
      Serial.print("ABC Status: ");
      myMHZ19.getABC() ? Serial.println("ON") : Serial.println("OFF");

      // Filter incorrect values
      myMHZ19.setFilter(true, true);

      err_co2 = false;
      co2_sensor = MHZ14A;

      // Timestamp for warming up start time
      int warming_up_start = millis();

      // Print info
      Serial.println("Warming up MHZ14A CO2 sensor...");

      // Wait for warming time while blinking blue led
      int counter = int(MHZ14A_WARMING_TIME / 1000);
      // Print welcome screen
      display.init();
      display.flipScreenVertically();
      while ((millis() - warming_up_start) < MHZ14A_WARMING_TIME)
      {
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Anaire50ppm");
        display.drawString(0, 10, "ID " + String(anaire_device_id));
        display.drawString(0, 20, String(counter));
        display.display(); // update OLED display
        Serial.print(".");
        delay(500); // wait 500ms
        Serial.println(".");
        delay(500); // wait 500ms
        counter = counter - 1;
      }

      display.clear();
      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 0, "Anaire50ppm");
      display.drawString(0, 10, "ID " + String(anaire_device_id));
      display.drawString(0, 20, String(counter));
      display.display(); // update OLED display

      // Print info
      Serial.println("Warming up MHZ14A CO2 sensor complete");

      // Print info
      Serial.println("MHZ14A CO2 sensor setup complete");

      // Setup DHT11
      dht.setup(DHT_GPIO, DHTesp::DHT11);
    }
  }
}


// LED:
// To blink on CO2_STATUS_BUILTIN_LED_GPIO
void changeState_CO2_STATUS_BUILTIN_LED_GPIO()
{
  digitalWrite(CO2_STATUS_BUILTIN_LED_GPIO, !(digitalRead(CO2_STATUS_BUILTIN_LED_GPIO))); //Invert Current State of LED CO2_STATUS_BUILTIN_LED_GPIO
  update_OLED_co2_flag = true;
}

// To blink on BUZZER_GPIO
void changeState_BUZZER_GPIO()
{
  digitalWrite(BUZZER_GPIO, !(digitalRead(BUZZER_GPIO))); //Invert Current State of BUZZER
}
