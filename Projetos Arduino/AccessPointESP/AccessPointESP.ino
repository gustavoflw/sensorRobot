#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// Wi-Fi
const char* ssid = "elonmusk";
const char* password = "elonmusk";
uint8_t newMACAddress[] = {0x5C, 0xE8, 0x83, 0x36, 0x9A, 0xC3};

void setup(void)
{
  WiFi.softAP(ssid, password);
  wifi_set_macaddr(SOFTAP_IF, &newMACAddress[0]);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop(void)
{
  
}
