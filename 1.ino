#include <SPI.h>
#include <LoRa.h>

#define NSS   15   // D8
#define RST   5    // D1
#define DIO0  4    // D2

int counter = 0;

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

  // Must match Node A and Node D
  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x34);
  LoRa.enableCrc();

  Serial.println("TX Ready");
}

void loop() {

  String msg = "<HELLO " + String(counter) + ">";

  Serial.print("Sending: ");
  Serial.println(msg);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  counter++;

  delay(2000);
}
