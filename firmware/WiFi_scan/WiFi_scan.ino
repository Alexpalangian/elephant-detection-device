#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Scanning for networks...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("No networks found!");
  } else {
    Serial.println(String(n) + " networks found:");
    for (int i = 0; i < n; i++) {
      Serial.println("  " + String(i+1) + ": " + WiFi.SSID(i) + 
                     " (Signal: " + String(WiFi.RSSI(i)) + " dBm)" +
                     (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " [OPEN]" : " [SECURED]"));
    }
  }
}

void loop() {}