#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 8);  // CE, CSN

const byte address[6] = "00001";
//Creating Data Structure
struct dataStruct {
  int turbidity;
  int ppm;
  int depth;
  int bottomSignalStrength;
  int topDistanceData;
  int topStrengthData;
} data;


void setup() {
  Serial.begin(9600);
  if (radio.begin()) {
    Serial.print("nice");
  } else {
    Serial.print("downtown");
  }
  radio.setChannel(5);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(0, address);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    Serial.print("TDS: ");
    Serial.println(data.ppm);
    Serial.print("Turbidity: ");
    Serial.println(data.turbidity);
    Serial.print("Bottom Depth: ");
    Serial.println(data.depth);
    Serial.print("Bottom Signal Strength: ");
    Serial.println(data.bottomSignalStrength);
    Serial.print("Top Distance: ");
    Serial.println(data.topDistanceData);
    Serial.print("Top Signal Strength: ");
    Serial.println(data.topStrengthData);
  }
}