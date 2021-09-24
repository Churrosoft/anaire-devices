#ifndef CHANGELOG_H
#define CHANGELOG_H
#include <Arduino.h>
#include <Print.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String sw_version = "v2.20210923.Naima";
// v2.20210619.Naima homogeneizes treatment of MH-Z14A and MH-Z19c
// v2.2021520.Malik changes on code comments to be consistent with the launch of new Anaire PiCO2 device and documentation update on Guthub. First major upgrade.
// 20210307 Fixed remoted updates by reducing BearSSL buffer size
// 20210304 Troubleshooting remote updates after enabling serial debug; display modified to show ppm in 16p before ppm value on 24p font
// 20210228 Fixed execution of individual MQTT commands; firmware updates work if Wifi connection is fast
// 20210223 Fixed MQTT error problem when Wifi didn't connect on the first try
// 20210221 Range of Winsen MH-Z14A/MH-Z19c set up to 2000ppm as it is enough to secure environments against COVID and provides more accuracy
// 20210221 The CO2 measurement sent by MQTT every MQTT loop (30s) is calculated as the mean of the measured values during the measurement loop (5s)
// 20210923 Migrate to Platformio + code split
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#endif