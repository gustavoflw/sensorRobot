#ifndef WIFI_H
#define WIFI_H

#include <ESP8266WiFi.h>
#include <WiFiClient.h>

/* To use as Access Point
 *  Default IP: 192.168.4.1
 */
void setupWifiAsAP(const char *wifi_ssid, const char *wifi_password, 
  uint8_t* newMACAddress)
{
    wifi_set_macaddr(SOFTAP_IF, newMACAddress);
    WiFi.softAP(wifi_ssid, wifi_password); //begin WiFi access point
}

/* To use as client
 *  Can customize IP, gateway, etc.
 */
void setupWifiAsClient(const char *wifi_ssid, const char *wifi_password, 
  IPAddress localIP, IPAddress gateway, IPAddress subnet, IPAddress dns) 
{
  // To use as client
  WiFi.config(localIP, gateway, subnet, dns, dns);
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("WiFi: ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("[OK]");
}

// Tries to reconnect if wifi lost connection
void reconnectIfNoWifi(unsigned long* millis_now, unsigned long* millis_wifiCheck, unsigned long millis_wifiInterval)
{
  if (WiFi.status() != WL_CONNECTED && (*millis_now) - (*millis_wifiCheck) >= millis_wifiInterval) {
    Serial.println("Reconnecting to WiFi...");
    *millis_wifiCheck = *millis_now;
    WiFi.disconnect();
    WiFi.reconnect();
  }
}

#endif
