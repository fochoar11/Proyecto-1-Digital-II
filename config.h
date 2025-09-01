/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "fochoar11"
#define IO_KEY       "xxxxxxxxxxxxxxxxxx"

/******************************* WIFI **************************************/

#define WIFI_SSID "javier"
#define WIFI_PASS "javier8a"

#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

