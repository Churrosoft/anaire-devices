#include "oled.h"

// Update CO2 info on OLED display
void update_OLED_CO2(int CO2ppm_value, float temperature, float humidity)
{

  // setup display and text format
  display.init();
  display.flipScreenVertically();
  display.clear();

  // display CO2 measurement
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  //display.setFont(ArialMT_Plain_24);
  //display.drawString(0, 4, String(CO2ppm_value) + "ppm");
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 8, "ppm");
  display.setFont(ArialMT_Plain_24);
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  //display.drawString(36, 4, String(CO2ppm_value));
  display.drawString(64, 4, String(CO2ppm_value));

  // And temperature and humidity
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(128, 0, String(int(temperature)) + "º  ");
  display.drawString(128, 16, String(int(humidity)) + "%");
  display.display(); // update OLED display
}

// Update device status on OLED display
void update_OLED_Status()
{

  // setup display and text format
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // display device id on first line
  display.drawString(0, 0, "ID " + String(anaire_device_id));

  // display IP address on second line
  String ipaddress = WiFi.localIP().toString();
  display.drawStringMaxWidth(0, 10, 128, ipaddress);

  // if there is an error display it on third line
  if (err_co2)
  {
    display.drawString(0, 20, "ERR CO2");
  }
  else if (err_wifi)
  {
    display.drawString(0, 20, "ERR WIFI");
  }
  else if (err_MQTT)
  {
    display.drawString(0, 20, "ERR MQTT");
  }
  else if (err_dht)
  {
    display.drawString(0, 20, "ERR DHT");
  }
  else
  {
    //display.drawString(0, 20, "device ok");
    display.drawString(0, 20, sw_version);
  }

  display.display(); // update OLED display
}