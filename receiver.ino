ADC_MODE(ADC_VCC);   // MUST be first line before includes

#include <SPI.h>
#include <LoRa.h>

#define NSS   15
#define RST   16
#define DIO0  4

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RST, OUTPUT);
  digitalWrite(RST, LOW);
  delay(100);
  digitalWrite(RST, HIGH);
  delay(100);

  SPI.begin();
  LoRa.setPins(NSS, RST, DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x34);
  LoRa.enableCrc();

  Serial.println("RX Ready");
}

void loop() {
  // Continuous VCC reading
  float vcc = ESP.getVcc() / 1000.0;

  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // VCC at moment of reception
    float rxVcc = ESP.getVcc() / 1000.0;

    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    Serial.println("======== Packet Received ========");
    Serial.print("Data    : "); Serial.println(received);
    Serial.print("RSSI    : "); Serial.print(LoRa.packetRssi()); Serial.println(" dBm");
    Serial.print("VCC now : "); Serial.print(rxVcc, 3);          Serial.println(" V");
    Serial.print("VCC idle: "); Serial.print(vcc, 3);            Serial.println(" V");
    Serial.print("Sag     : "); Serial.print(vcc - rxVcc, 4);    Serial.println(" V");
    Serial.println("=================================");

  } else {
    // Print idle VCC every second
    static unsigned long last = 0;
    if (millis() - last > 1000) {
      Serial.print("Idle VCC: ");
      Serial.print(vcc, 3);
      Serial.println(" V");
      last = millis();
    }
  }
}
