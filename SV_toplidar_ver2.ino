#include<SoftwareSerial.h>
SoftwareSerial Serial1(6,5);
int dist; //distance
int check;//save check value
int uart[44];//save data
const int StartCh = 0x54;// frame header of data package
const int VerLen = 0x2C;
int a;
int b;
int SV[10];

void setup() {
  Serial.begin(9600);
  Serial1.begin(230400);
}

void loop() {
  for (int i = 0; i < 3; i++){
    SV[i] = Serial1.read();
    dist = Serial1.read();
    Serial.println(dist);
    delay(1000);
  }
  
  //Serial.print("Second option:");
  //b = Serial1.read();
  //Serial.println(b);
  
}
