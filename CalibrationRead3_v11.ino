/* Fix issue: avgBnorm() only save the latest value if read 3 fingers at once. Work!!
 * 
 */


#include <SPI.h>
#include <math.h>

int PinThumb = 10;
int PinIndex = 9;
int PinMiddle = 8;
int PinMOSI = 11;
int PinMISO = 12;
int PinSCK = 13;

float Bnorm, avgThumb, avgIndex, avgMiddle;

void setup() {
  pinMode (PinMOSI, OUTPUT);
  pinMode (PinMISO, INPUT);
  pinMode (PinSCK, OUTPUT);
  pinMode (PinThumb, OUTPUT);
  pinMode (PinIndex, OUTPUT);
  pinMode (PinMiddle, OUTPUT);
  digitalWrite(PinThumb, HIGH);
  digitalWrite(PinIndex, HIGH);
  digitalWrite(PinMiddle, HIGH);
  Serial.begin(9600);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE1);
  
  avgThumb = avgBnorm(PinThumb);
  avgIndex = avgBnorm(PinIndex);
  avgMiddle = avgBnorm(PinMiddle);
}

void loop() { 
  sensorRead(PinThumb); Serial.print(Bnorm);  Serial.print("\t");
  sensorRead(PinIndex); Serial.print(Bnorm);  Serial.print("\t");
  sensorRead(PinMiddle); Serial.print(Bnorm);  Serial.print("\t");
  Serial.println();
  delay(10);  
}

void sensorRead(int PinSS)  {
  sensorRead(PinSS, false);
}

void sensorRead (int PinSS, bool calibration) {
  uint8_t readBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t writeBuffer[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t errorBits, CRC, rollingCounter;
  int16_t Bx, By, Bz;

  digitalWrite(PinSS, LOW);
  writeBuffer[0] = 0x00;
  writeBuffer[1] = 0x00;
  writeBuffer[2] = 0xFF; // Timeout value is set as 65 ms
  writeBuffer[3] = 0xFF; // Timeout value is set as 65 ms
  writeBuffer[4] = 0x00;
  writeBuffer[5] = 0x00;
  writeBuffer[6] = 0x93; // Marker is set as 2 to get XYZ measurement. OP Code for GET1 message: 19 in Decimal.
  writeBuffer[7] = ComputeCRC(0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x93); // CRC
  delayMicroseconds(1);
  for (int i = 0; i < 8; i++) {
    readBuffer[i] = SPI.transfer(writeBuffer[i]);
  }
  delayMicroseconds(1500);
  digitalWrite(PinSS, HIGH);

  Bx = (readBuffer[1] & 0x3F) << 8;
  Bx += readBuffer[0];
  if (Bx >= 8192) {
    Bx -= 16384;
  }

  By = (readBuffer[3] & 0x3F) << 8;
  By += readBuffer[2];
  if (By >= 8192) {
    By -= 16384;
  }

  Bz = (readBuffer[5] & 0x3F) << 8;
  Bz += readBuffer[4];
  if (Bz >= 8192) {
    Bz -= 16384;
  }

  errorBits = readBuffer[0] >> 6;
  CRC = readBuffer[7];
  rollingCounter = readBuffer[6] & 0x3F;

  Bnorm = sqrt( (double)Bx * (double)Bx + (double)By * (double)By + (double)Bz * (double)Bz );  //Serial.print(Bnorm);  Serial.print("\t");

  if (calibration == false & PinSS==PinThumb) { 
    Bnorm = Bnorm - avgThumb;  //Serial.print(avgThumb); Serial.print("\t");  Serial.print(Bnorm); Serial.print("\t"); 
  }
  
  if (calibration == false & PinSS==PinIndex) { 
    Bnorm = Bnorm - avgIndex;  //Serial.print(avgIndex); Serial.print("\t");  Serial.print(Bnorm); Serial.print("\t"); 
  }

  if (calibration == false & PinSS==PinMiddle) { 
    Bnorm = Bnorm - avgMiddle;  //Serial.print(avgMiddle); Serial.print("\t");  Serial.print(Bnorm); Serial.print("\t"); 
  }
}

float avgBnorm(int PinSS)  {
  static float readings[6];
  static float average, sum; 
  for (int i=0; i<6; i++) {
    sensorRead(PinSS, true);
    readings[i] = Bnorm;  //Serial.print("array= ");  Serial.print(readings[i]); Serial.print("\t");
    delay(1);
  }
  sum = readings[3]+readings[4]+readings[5];  //Serial.print("sum= ");  Serial.print(sum); Serial.print("\t");
  average = sum/3.0;  //Serial.print("avg= ");  Serial.print(average); Serial.print("\t");
  return average;
}

void initReading()  {
  const int numReadings = 10;     // average number of readings
  int readings[numReadings];      // the readings from the sensor
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  } 
}

uint8_t ComputeCRC(uint8_t Byte0, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5, uint8_t Byte6) {
  uint8_t CRC = 0xFF;
  char CRCArray[] = {
  0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26,
  0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
  0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02,
  0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
  0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB,
  0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
  0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39,
  0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
  0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3,
  0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
  0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92,
  0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
  0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B,
  0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
  0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F,
  0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
  0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23,
  0xEE, 0xC1, 0xB0, 0x9F, 0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
  0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
  0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
  0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE,
  0x33, 0x1C, 0x6D, 0x42
  };

  CRC = CRCArray[CRC ^ Byte0];
  CRC = CRCArray[CRC ^ Byte1];
  CRC = CRCArray[CRC ^ Byte2];
  CRC = CRCArray[CRC ^ Byte3];
  CRC = CRCArray[CRC ^ Byte4];
  CRC = CRCArray[CRC ^ Byte5];
  CRC = CRCArray[CRC ^ Byte6];
  CRC = ~CRC;
  return CRC;
}
