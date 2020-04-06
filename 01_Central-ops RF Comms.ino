#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;  

int coordinateSend[4];
int coordinateSendIndex = 0;    //Used to store message in proper buffer
boolean newcoordinatesSend = false;
int coordinateSendNumber = 0;

int RF_X = 0;
int RF_Y = 0;
int RF_theta = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  Serial.println("Enter Instruction: ");
  Serial.println("0 to stop");
  Serial.println("1 to seek");
  Serial.println("2 for test function");
  Serial.println("3 to move to specific coordinate");
  Serial.println("4 to return home");
  coordinateSendIndex = 0;
  receiveInput();
  radio.stopListening();    //Stop listening after receiving a instruction message from user. Prepares to send the message
  radio.write(&coordinateSend[coordinateSendIndex], sizeof(coordinateSend[coordinateSendIndex]));
  
  //Only ask for coordinates if moving to specific coordinate
  if (coordinateSend[0] == 3) {
    //Asks user for input before sending out coordinates
    Serial.println("Enter X-Coordinate: ");
    coordinateSendIndex = 1;
    receiveInput();
    
    Serial.println("Enter Y-Coordinate: ");
    coordinateSendIndex = 2;
    receiveInput();
    
    Serial.println("Enter theta: ");
    coordinateSendIndex = 3;
    receiveInput();
    
    Serial.print("Sending... ");  //Notifies user when the send is starting
    delay(1000);
    radio.write(&coordinateSend[1], sizeof(coordinateSend[1]));
    
    delay(2000);
    radio.write(&coordinateSend[2], sizeof(coordinateSend[2]));
    
    delay(2000);
    radio.write(&coordinateSend[3], sizeof(coordinateSend[3]));

    
    //Without the delays, some of the read values may get dropped
    Serial.println("Sent!");      //Notifies user when send is complete
  }
  delay(500);
  //Only start listening after central ops sends first message
  radio.startListening();
}

//Asks user for input from serial monitor
void receiveInput(){
  while (newData == false){
    static byte index = 0;
    char endMarker = '\n';
    char data;

    //Allows program to listen to messages while waiting for new user input
    if(radio.available())
    {
      Serial.println("Incoming Coordinates ...");
      
      radio.read(&RF_X, sizeof(RF_X));
      
      while(!radio.available());
      radio.read(&RF_Y, sizeof(RF_Y));
      
      while(!radio.available());
      radio.read(&RF_theta, sizeof(RF_theta));
      
      Serial.print("Current Coordinates: ");
      Serial.print(RF_X);
      Serial.print(", ");
      Serial.print(RF_Y);
      Serial.print(", ");
      Serial.println(RF_theta);
    }
  
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
    delay(50);
  }
  newData = false;
}
