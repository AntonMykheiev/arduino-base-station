#include <WiFi.h>
#include <WiFiUdp.h>

#define SERIAL_RTK Serial1

const char* ssid = "";
const char* password = "";

const char* udpAddress = "";
const int udpPort = 00000;

static uint8_t rtcmBuffer[1024];
static size_t index = 0;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  SERIAL_RTK.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(" Connected!");

  udp.begin(udpPort);
}

void loop() {
  while (SERIAL_RTK.available()) {
    uint8_t data = SERIAL_RTK.read();

    if (index == 0 && data != 0xD3) continue;

    rtcmBuffer[index++] = data;

    if (index > 2) {
      uint16_t length = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];

      if (index >= length + 6) {
        udp.beginPacket(udpAddress, udpPort);
        udp.write(rtcmBuffer, index);
        udp.endPacket();
        Serial.println("RTCM data sent!");

        index = 0;
      }
    }
  }
}