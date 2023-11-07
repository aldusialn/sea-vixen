// Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>  // Header file for software serial port

// Define serial for bottom LiDAR
SoftwareSerial Serial1(3, 4);  // Define software serial port name as Serial1 and define pin2 as RX & pin3 as TX
SoftwareSerial Serial2(6, 5);  // Define ports for upper LiDAR

// Upper LiDAR code
int upper_dist;            // Actual distance measurements of LiDAR
int upper_strength;        // Signal strength of LiDAR
int upper_check;           // Save check value
int f;                     // Second-worst possible solution (DON'T TOUCH!!)
int upper_uart[44];        // Save data measured by LiDAR
const int StartCh = 0x54;  // Frame header of data package
const int VerLen = 0x2C;

// Serial1 - bottom LiDAR
int dist;                 // Actual distance measurements of LiDAR
int strength;             // Signal strength of LiDAR
int check;                // Save check value
int v;                    // Worst possible programming solution (DON'T TOUCH!!)
int uart[9];              // Save data measured by LiDAR
const int HEADER = 0x59;  // Frame header of data package

// Define for TDS
#define TdsSensorPin A1
#define VREF 5.0           // Analog reference voltage (Volt) of the ADC
#define SCOUNT 30          // Sum of sample point
int analogBuffer[SCOUNT];  // Store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

// Initializing radio pins
RF24 radio(9, 8);  // CE, CSN

// Address
const byte address[6] = "00001";

// Creating Data Structure
struct dataStruct {
  int turbidity;
  int ppm;
  int depth;
  int bottomSignalStrength;
} data;

// Function declaration for getMedianNum
int getMedianNum(int bArray[], int iFilterLen);

void setup() {

  Serial.begin(9600);   // Arduino
  pinMode(TdsSensorPin, INPUT);

  while (!Serial) {
    ;  // Wait for the serial port to connect. Needed for the native USB port only
  }

  if (!radio.begin()) {
    Serial.print("Radio did not start");
  } else {
    Serial.print("Started");
  }
  radio.setChannel(5);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop() {
  // Bottom LiDAR code
  Serial1.begin(115200);
  while(!Serial1.available()) {
  if (Serial1.available())  // Check if the serial port has data input
  {
    if (Serial1.read() == HEADER)  // Assess data package frame header 0x59
    {
      uart[0] = HEADER;
      if (Serial1.read() == HEADER)  // Assess data package frame header 0x59
      {
        uart[1] = HEADER;
        for (v = 2; v < 9; v++)  // Save data in an array
        {
          uart[v] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff))  // Verify the received data as per protocol
        {
          dist = uart[2] + uart[3] * 256;      // Calculate distance value
          strength = uart[4] + uart[5] * 256;  // Calculate signal strength value
          Serial.print("Bottom depth:");
          Serial.print(dist);
        }
      }
    }
  }
  }
  Serial1.end();

  // TDS
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  // Every 40 milliseconds, read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  // Read the analog value and store it in the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                   // Read the analog value more stable by the median filtering algorithm, and convert it to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                // Temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                             // Temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5;  // Convert voltage value to TDS value
  }
  // Send message to receiver
  /*
  data.ppm = tdsValue;              // Get ppm
  data.turbidity = analogRead(A2);  // Get turbidity
  data.depth = dist;
  data.bottomSignalStrength = strength;
  Serial.print(dist);
  radio.write(&data, sizeof(data));  
  */
  // Upper LiDAR
  Serial2.begin(230400);
  while (!Serial2.available()) {
  if (Serial2.available())  // Check if the serial port has data input
  {
    if (Serial2.read() == StartCh)  // Assess data package frame header 0x59
    {
      upper_uart[0] = StartCh;
      upper_uart[1] = VerLen;
      for (f = 6; f < 40; f++)  // Save data in an array
      {
        upper_uart[f] = Serial2.read();
      }
      for (f = 0; f < 43; f++) {
        upper_check += upper_uart[f];
      }
      if (upper_uart[43] == (upper_check & 0xff))  // Verify the received data as per protocol
      {
        for (f = 6; f < 40; f += 2)  // Save data in an array
        {
          upper_dist = upper_uart[f];
          upper_strength = upper_uart[f + 1];
          Serial.print("distance = ");
          Serial.print(upper_dist);
          Serial.print('\t');
          Serial.print("strength = ");
          Serial.print(upper_strength);
          Serial.print('\n');
        }
      }
    }
  }
  }
  Serial2.end();
  
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
