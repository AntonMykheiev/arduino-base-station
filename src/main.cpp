#include <ArduinoJson.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

const char* ssid = "";
const char* password = "";

WiFiUDP udp;
const char* udpAddress = "";
const int udpPort = 00000;

#define RTCM_SERIAL Serial1
#define BUFFER_SIZE 512

void setup() {
  RTCM_SERIAL.begin(115200);
  WiFi.begin(ssid, password);
  udp.begin(udpPort);
}

void loop() {
  static uint8_t buffer[BUFFER_SIZE];
  static size_t index = 0;

  while (RTCM_SERIAL.available()) {
    uint8_t byte = RTCM_SERIAL.read();
    buffer[index++] = byte;

    if (index >= BUFFER_SIZE || (index > 3 && buffer[0] == 0xD3)) {
      udp.beginPacket(udpAddress, udpPort);
      udp.write(buffer, index);
      udp.endPacket();

      index = 0;
    }
  }
}