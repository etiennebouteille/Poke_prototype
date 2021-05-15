#include <SPI.h>
#include <RH_RF95.h>

#include <Wire.h>
#include <Adafruit_GFX.h>

#define LED      13

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

int16_t packetnum = 0;  // packet counter, we increment per xmission

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  delay(500);
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather LoRa RX/TX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  rf95.setTxPower(23, false);
  rf95.setSpreadingFactor(12);
  
  pinMode(LED, OUTPUT);

  Serial.print("LoRa radio @"); Serial.print((int)RF95_FREQ); Serial.println(" MHz");

  delay(500);
}

void loop() {
  
  
    char radiopacket[20] = "Hello";
    itoa(packetnum++, radiopacket+5, 10);
    radiopacket[9] = 0;

    Serial.print("Sending "); Serial.println(radiopacket);
    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf95.waitPacketSent();

    delay(1000);
}
