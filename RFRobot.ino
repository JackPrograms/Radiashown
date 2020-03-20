#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN

const byte addresses[][6] = {"00001", "00002"};
int XcoordinateReceived = 0;
int YcoordinateReceived = 0;
int XcoordinateRobot = 0;
int YcoordinateRobot = 0;

int robotMoveComplete = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino set as receiver>");
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_LOW);
}

void loop() {
  delay(1000);
  
  radio.startListening();
  if (radio.available()) {
    radio.read(&XcoordinateReceived, sizeof(XcoordinateReceived));
    Serial.println("X-Coordinate to go to: ");
    Serial.println(XcoordinateReceived);
  }
  if (radio.available()) {
    radio.read(&YcoordinateReceived, sizeof(YcoordinateReceived));
    Serial.println("Y-Coordinate to go to: ");
    Serial.println(YcoordinateReceived);
  }

  delay(5000);

  //---------------------------------------------------------------------------------------------------
  //ROBOT CODE THAT MOVES ROBOT TO COORDINATES, assigns X to XcoordinateRobot and Y to YcoordinateRobot
  //---------------------------------------------------------------------------------------------------
  robotMoveComplete = 1;
  radio.stopListening();
  
  radio.write(&robotMoveComplete, sizeof(robotMoveComplete));
  delay(1000);

  Serial.println("X-Coordinate of robot: ");
  Serial.println(XcoordinateRobot);
  radio.write(&XcoordinateRobot, sizeof(XcoordinateRobot));
  delay(500);
  
  Serial.println("Y-Coordinate of robot: ");
  Serial.println(YcoordinateRobot);
  radio.write(&YcoordinateRobot, sizeof(YcoordinateRobot));

  delay(1000);
}
