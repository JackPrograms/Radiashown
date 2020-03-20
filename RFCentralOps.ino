#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;  

int coordinateSend[2];
int coordinateSendIndex = 0;
boolean newcoordinatesSend = false;
int coordinateSendNumber = 0;

int XcoordinateReceived = 0;
int YcoordinateReceived = 0;

int robotMoveComplete = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino set as transmitter>");
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_LOW);
}

void loop() {
  const char text[] = "Connected";
  radio.write(&text, sizeof(text));
  
  delay(1000);
  radio.stopListening();
  
  Serial.println("Enter X-Coordinate: ");
  coordinateSendIndex = 0;
  receiveInput();
  radio.write(&coordinateSend[coordinateSendIndex], sizeof(coordinateSend[coordinateSendIndex]));

  Serial.println("Enter Y-Coordinate: ");
  coordinateSendIndex = 1;
  receiveInput();
  radio.write(&coordinateSend[coordinateSendIndex], sizeof(coordinateSend[coordinateSendIndex]));
  
  robotMoveComplete = 0;
  delay(5000);

  radio.startListening();
  
  //WAITING FOR ROBOT TO FINISH MOVING -------------------------------------------

  char robotMoveStatus = 0;
  radio.read(&robotMoveStatus, sizeof(robotMoveStatus));
  if (robotMoveStatus = '1'){
    if (radio.available()) {
      radio.read(&XcoordinateReceived, sizeof(XcoordinateReceived));
      Serial.println("X-Coordinate of robot: ");
      Serial.println(XcoordinateReceived);
    }
    if (radio.available()) {
      radio.read(&YcoordinateReceived, sizeof(YcoordinateReceived));
      Serial.println("Y-Coordinate of robot: ");
      Serial.println(YcoordinateReceived);
    }  
  }
}

void receiveInput(){
  while (newData = false){
    static byte index = 0;
    char endMarker = '\n';
    char data;
    if (Serial.available() > 0) {
      data = Serial.read();
      
      if (data != endMarker){
        receivedChars[index] = data;
        index++;
        if (index >= numChars){
          index = numChars - 1;
        }
        
      }
      else{
          receivedChars[index] = '\0';
          index = 0;
          dataNumber = 0;
          dataNumber = atoi(receivedChars);
          coordinateSend[coordinateSendIndex] = dataNumber;
          newData = true;
      }
    }
    delay(500);
  }
  newData = false;
}
