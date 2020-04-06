#include <SharpIR.h>
#include <RPLidar.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

/*********************************
 * Variables with comment "Tuneable" means should be adjusted depending on floor texture
 * ******************************/

/***************Instructions Mask***************/
#define ins_Idle 0
#define ins_Seek 1
#define ins_TestFunc 2
#define ins_CommandToCoord 3
#define D 1     //Forwards
#define B 2     //Backwards
#define L 3     //Left
#define R 4     //Right

/***************Pin Definitions***************/
//IR Sensors
#define Pin_IROut1 A0   //Side 1 is left side
#define Pin_IROut2 A1   //Side 2 is right side
#define IRModel 1080

//Wifi Module
#define Pin_WifiRxd 1
#define Pin_WifiTxd 2

//Motor Shield
#define Pin_Motor1A 4   //Side 1 is left side
#define Pin_Motor1B 5
#define Pin_MotorEnable 6
#define Pin_Motor2A 7   //Side 2 is right side
#define Pin_Motor2B 8
#define Pin_Motor1Power 9
#define Pin_Motor2Power 10
#define Pin_Motor1EnA 20    //Motor 1 Encoder A output
#define Pin_Motor2EnA 21    //Motor 2 Encoder A output

//Lidar
#define Pin_LidarTxd 18
#define Pin_LidarRxd 19
#define RPLIDAR_MOTOR 44

//RF Module
#define Pin_RFCE 27
#define Pin_RFCSN 28
#define Pin_RFMISO 50
#define Pin_RFMOSI 51
#define Pin_RFSCK 52

//Sonar Sensor
#define Pin_SonarTrig 46
#define Pin_SonarEcho 47

/***************Variable Definitions***************/
//Timing
#define UpdateInterval 10000      //Update Coords every 10s
unsigned long startTime = 0;
unsigned long lastUpdateTime = 0;

//Instruction
int8_t StateCurrent = ins_Idle;

//Coordinate: x, y, theta (0 deg points 12 o'clock, angle increases clockwise)
float CoordCurrent[3] = {0,0,0};
float CoordDesired[3] = {0,0,0};

//Comms
RF24 radio(Pin_RFCE, Pin_RFCSN);
const byte addresses[][6] = {"00001", "00002"};
int RF_Ins = 0;
int RF_X = 0;
int RF_Y = 0;
int RF_theta = 0;

//Lidar
#define RangeLimitLow 400       //How close robot can get to wall before reaction. Tuneable
#define RangeLimitHigh 2000     //How close robot can get to wall before reaction. Tuneable
#define SpreadTolerance 5       //Spread of allowable angle ready (ex. 90+/- 5 deg)
#define ScanSpread 15
#define SamplesNeeded 4         //Lidar samples used to avoid noise
bool lidarscan_OK = 0;
bool Seek_toggle = 0;           //For toggle lidar on/off
RPLidar lidar;                  //Initiate lidar object

//Motor Shield Varialbes
#define ENC_Count_Rev 480   //Motor Encoder Output Pulses Per Rotation
#define speedOpt 60         //Need to test with detector on. Whether acceleration is needed or not needs to be tested too
#define accel 10            //How fast speed increases per cycle up to speedOpt. Robot has trouble speeding up with heavy load. Tuneable
#define turnspeed_dom 50    //Speed for turning. Tuneable
#define turnspeed_sub 50
#define angencConst 6.2     //Ratio of encoder to angle. Tuneable (Need re-tuning when radiation detector is installed)
volatile long Val_Encoder1 = 0;     //Tracks value of encoder
volatile long Val_Encoder2 = 0;
long Val_Encoder1_old = 0;
long Val_Encoder2_old = 0;
int speed1 = 0;             //Speed of motor when travelling forward and backward
int speed2 = 0;

/***************Sensor Setup***************/
//IR Setup
SharpIR IRSensor1 = SharpIR(Pin_IROut1, IRModel);
SharpIR IRSensor2 = SharpIR(Pin_IROut2, IRModel);

void setup() {
  lidar.begin(Serial1);
  
  /***************Pins Init***************/
  //lidar motor pin  
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  //Motor Shield Pins
  pinMode(Pin_MotorEnable, OUTPUT);
  pinMode(Pin_Motor1A, OUTPUT);  //Motor 1 pin init
  pinMode(Pin_Motor1B, OUTPUT);
  pinMode(Pin_Motor1Power, OUTPUT);
  pinMode(Pin_Motor2A, OUTPUT);  //Motor 2 pin init
  pinMode(Pin_Motor2B, OUTPUT);
  pinMode(Pin_Motor2Power, OUTPUT);
  pinMode(Pin_Motor1EnA, INPUT_PULLUP);

  //Sonar Pins
  pinMode(Pin_SonarTrig, OUTPUT);
  pinMode(Pin_SonarEcho, INPUT);
  digitalWrite(Pin_SonarTrig, LOW);
  
  //Motor Shield Setup
  digitalWrite(Pin_MotorEnable, HIGH);  //Enable Motor Shield
  attachInterrupt(digitalPinToInterrupt(Pin_Motor1EnA), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_Motor2EnA), updateEncoder2, RISING);

  //RF Setup
  radio.begin();
  radio.openWritingPipe(addresses[0]);      // 00001
  radio.openReadingPipe(1, addresses[1]);   // 00002
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  //For Testing Only
  // delay(2000);
  // startTime = millis();
  // StateCurrent = ins_Seek;
//    Serial.begin(9600);
}

void loop() {
  /*----------Determine new state of robot----------*/
  //Check new comms came in
  if (!radio.available()) {           //If no new comms came in, carry out instructions as planned
    switch (StateCurrent) {
      case ins_Idle:    //Robot starts off Idle. Waits for start signal from central_Ops
        //wait for comms to tell it what to do
        startTime = millis();
        lastUpdateTime = millis();
        movStop();
      break;
      case ins_Seek:
        //If robot is close to origin and have been running for over 30s, move to origin and stop seeking
        if (millis() - startTime > 30000 && CoordCurrent[0] > -500 && CoordCurrent[0] < 500 && CoordCurrent[1] > -500 && CoordCurrent[1] < 500) {
          updateCoords(CoordDesired, 0, 0, 0);
          StateCurrent = ins_CommandToCoord;
        }
        else {
          //If robot just started runnin or far from the origin
          scan();
          float adjDist_L = 0;
          float adjDist_R = 0;
          float adjDist_D = 0;
          int8_t scan_counter_L = 0;
          int8_t scan_counter_R = 0;
          int8_t scan_counter_D = 0;
          while(!lidarscan_OK) {
            if (IS_OK(lidar.waitPoint()) && !lidar.getCurrentPoint().startBit) {
              //readings with startbit = 1 always seem to be really bad
              //Can use lpf instead of averaging to get rid of errors in sampling https://dsp.stackexchange.com/questions/39063/simple-software-low-pass-filter
              float scan_angle = lidar.getCurrentPoint().angle;
              float scan_dist = 0;
              //stores samples on the left side of the robot
              if (scan_counter_L < SamplesNeeded && scan_angle > 90 - SpreadTolerance && scan_angle < 90 + SpreadTolerance) {
                scan_dist = lidar.getCurrentPoint().distance;
                if (scan_dist > 0.1 || scan_dist < -0.1) {
                  //Sometimes distance comes back and reads 0. Do not store sample if that happens
                  adjDist_L += scan_dist*cos(abs(90-scan_angle)*PI/180);        //Use trig to find the distance directly left of robot
                  scan_counter_L++;
                }
              }
              //stores samples on the front of the robot
              else if (scan_counter_D < SamplesNeeded && scan_angle > 180 - SpreadTolerance && scan_angle < 180 + SpreadTolerance) {
                scan_dist = lidar.getCurrentPoint().distance;
                if (scan_dist > 0.1 || scan_dist < -0.1) {
                  adjDist_D += scan_dist*cos(abs(180-scan_angle)*PI/180);        //Use trig to find the distance directly in front of robot
                  scan_counter_D++;
                }
              }
              //stores samples on the righ side of the robot
              else if (scan_counter_R < SamplesNeeded && scan_angle > 270 - SpreadTolerance && scan_angle < 270 + SpreadTolerance) {
                scan_dist = lidar.getCurrentPoint().distance;
                if (scan_dist > 0.1 || scan_dist < -0.1) {
                  adjDist_R += scan_dist*cos(abs(270-scan_angle)*PI/180);        //Use trig to find the distance directly right of robot
                  scan_counter_R++;
                }
              }
              //Once enough samples for each side has been stored, stop reading
              if (scan_counter_D >= SamplesNeeded && scan_counter_L >= SamplesNeeded && scan_counter_R >= SamplesNeeded) {
                lidarscan_OK = 1;
              }
            }
          }
          lidarscan_OK = 0;
          //Find average length of normalized distance
          adjDist_L = adjDist_L/SamplesNeeded;
          adjDist_D = adjDist_D/SamplesNeeded;
          adjDist_R = adjDist_R/SamplesNeeded;
          //if nothing is close in front
          if (adjDist_D > RangeLimitLow+200) {
            //check left side of robot
            //if very far from left wall
            if (adjDist_L > RangeLimitHigh*3) {
              //Move forward a little, turn 90 degrees left. Allow robot to keep scanning one way 
              pauseScan();
              BroadcastCoord();
              Translate(D, 500);
              Rot(L, 90);
            }
            //if kind of too far from left wall
            else if (adjDist_L > RangeLimitHigh) {
              //Move closer to the left wall
              pauseScan();
              BroadcastCoord();
              Rot(L, 90);
              Translate(D, adjDist_L - (RangeLimitHigh + RangeLimitLow)/2);
              Rot(R, 90);
            }
            //if too close to left wall
            else if (adjDist_L < RangeLimitLow) {
              //move robot away from wall
              pauseScan();
              BroadcastCoord();
              Rot(R, 90);
              Translate(D, (RangeLimitHigh + RangeLimitLow)/2 - adjDist_L);
              Rot(L, 90);
            }
            //if within range of from left wall (>RangeLimitLow, <RangeLimitHigh)
            else {
              //Check if robot travelling parallel with left wall. Checks point slightly ahead and behind on left side of robot. Calculate angle of line
              float adjDist_Low = 0;
              float adjDist_High = 0;
              int8_t scan_counter_Low = 0;
              int8_t scan_counter_High = 0;
              while(!lidarscan_OK) {
                if (IS_OK(lidar.waitPoint()) && !lidar.getCurrentPoint().startBit) {
                  float scan_angle = lidar.getCurrentPoint().angle;
                  float scan_dist = lidar.getCurrentPoint().distance;
                  //Reads angle slightly in front and behind of directly left of robot. Use the lengths to calculate robot angle to left wall
                  //if angle is slightly behind
                  if (scan_counter_Low < SamplesNeeded && scan_angle > 90 - ScanSpread - SpreadTolerance && scan_angle < 90 - ScanSpread + SpreadTolerance) {
                    scan_dist = lidar.getCurrentPoint().distance;
                    if (scan_dist > 0.1 || scan_dist < -0.1) {
                      adjDist_Low += scan_dist*cos(abs(90-ScanSpread-scan_angle)*PI/180);
                      scan_counter_Low++;
                    }
                  }
                  //if angle is slightly ahead
                  else if (scan_counter_High < SamplesNeeded && scan_angle > 90 + ScanSpread - SpreadTolerance && scan_angle < 90 + ScanSpread + SpreadTolerance) {
                    scan_dist = lidar.getCurrentPoint().distance;
                    if (scan_dist > 0.1 || scan_dist < -0.1) {
                      adjDist_High += scan_dist*cos(abs(90+ScanSpread-scan_angle)*PI/180);
                      scan_counter_High++;
                    }
                  }
                  if (scan_counter_Low >= SamplesNeeded && scan_counter_High >= SamplesNeeded) {
                    lidarscan_OK = 1;
                  }
                }
              }
              lidarscan_OK = 0;
              adjDist_Low = adjDist_Low/SamplesNeeded;
              adjDist_High = adjDist_High/SamplesNeeded;
              //Using cosine law to find robot angle with wall
              float len_opp = sqrt(sq(adjDist_Low) + sq(adjDist_High) - 2*adjDist_Low*adjDist_High*cos(ScanSpread*PI/90));    //cos(2*ScanSpread*PI/180)
              float angle2wall = 90 - ScanSpread - acos((sq(adjDist_Low) + sq(len_opp) - sq(adjDist_High))/(2*adjDist_Low*len_opp))*180/PI;
              //If robot angle is too off from wall
              if (angle2wall > 15) {
                pauseScan();
                BroadcastCoord();
                Rot(R, angle2wall);
              }
              else if (angle2wall < -15) {
                pauseScan();
                BroadcastCoord();
                Rot(L, -angle2wall);
              }
              //If robot is relatively parallel with the wall
              else {
                //check right side of robot
                //if right side of robot is far
                if (adjDist_R > RangeLimitLow) {
                  //Ideal condition. Move forward and proceed with scan
                  if (readSonar() > 20 || IRSensor1.distance() > 10 || IRSensor2.distance() > 0) {
                    movFwd(); 
                  }
                  // else {
                    //Object detection code
                  // }
                }
                // if right side is close
                else {
                  //move robot to coord in between the left and right walls
                  pauseScan();
                  BroadcastCoord();
                  Rot(L, 90);
                  Translate(D, (adjDist_L - adjDist_R)/2);
                  Rot(R, 90);
                }
              }
            }
          }
          //If robot is pretty close to the front wall
          else {
            //Check if robot travelling parallel with front wall. Checks point slightly ahead and behind on left side of robot. Calculate angle of line. Lets robot turn to be parallel with front wall
            float adjDist_Low = 0;
            float adjDist_High = 0;
            int8_t scan_counter_Low = 0;
            int8_t scan_counter_High = 0;
            while(!lidarscan_OK) {
              if (IS_OK(lidar.waitPoint()) && !lidar.getCurrentPoint().startBit) {
                float scan_angle = lidar.getCurrentPoint().angle;
                float scan_dist = 0;
                //Reads angle slightly left and right of directly in front of the robot. Use the lengths to calculate robot angle to front wall
                //scan slightly to the left
                if (scan_counter_Low < SamplesNeeded && scan_angle > 180 - ScanSpread - SpreadTolerance && scan_angle < 180 - ScanSpread + SpreadTolerance) {
                  scan_dist = lidar.getCurrentPoint().distance;
                  if (scan_dist > 0.1 || scan_dist < -0.1) {
                    adjDist_Low += scan_dist*cos(abs(180-ScanSpread-scan_angle)*PI/180);
                    scan_counter_Low++;
                  }
                }
                //scan slightly to the right
                else if (scan_counter_High < SamplesNeeded && scan_angle > 180 + ScanSpread - SpreadTolerance && scan_angle < 180 + ScanSpread + SpreadTolerance) {
                  scan_dist = lidar.getCurrentPoint().distance;
                   if (scan_dist > 0.1 || scan_dist < -0.1) {
                    adjDist_High += scan_dist*cos(abs(180+ScanSpread-scan_angle)*PI/180);
                    scan_counter_High++;
                   }
                }
                if (scan_counter_Low >= SamplesNeeded && scan_counter_High >= SamplesNeeded) {
                  lidarscan_OK = 1;
                }
              }
            }
            pauseScan();
            BroadcastCoord();
            lidarscan_OK = 0;
            adjDist_Low = adjDist_Low/SamplesNeeded;
            adjDist_High = adjDist_High/SamplesNeeded;
            //Using cosine law to find robot angle with front wall
            float len_opp = sqrt(sq(adjDist_Low) + sq(adjDist_High) - 2*adjDist_Low*adjDist_High*cos(ScanSpread*PI/90));    //cos(2*ScanSpread*PI/180)
            float angle2wall =  90 - ScanSpread - acos((sq(adjDist_Low) + sq(len_opp) - sq(adjDist_High))/(2*adjDist_Low*len_opp))*180/PI;
            //Rotates robot to be parallel with front wall
            Rot(R, 90 + angle2wall);
          }
        }
        //Updates coordinates to central-ops every "UpdateInterval x ms" amount of time if robot has been continuously moving forward
        if (millis() - lastUpdateTime > UpdateInterval) {
          pauseScan();
          BroadcastCoord();
        }
      break;
      case ins_TestFunc:
        //Command for testing specific functions
        Rot(R, 90);
        Rot(L, 45);
        Rot(L, 45);
        updateCoords(CoordDesired, 1000, 1000, 0);
        MoveCurrent2Desired();
        updateCoords(CoordDesired, 0, 0, 0);
        MoveCurrent2Desired();
        delay(4000);
      break;
      case ins_CommandToCoord:
        //Instruction that directs robot to specific coord
        //Stops Lidar Scan
        pauseScan();
        BroadcastCoord();
        lidar.stop();
        analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
        Seek_toggle = 0;
        //Moves robot to desired coords
        MoveCurrent2Desired();
        //Resets to idle state
        StateCurrent = ins_Idle;
      break;
    }
  }
  else {    //if there was comms
    pauseScan();
    radio.read(&RF_Ins, sizeof(RF_Ins));
    StateCurrent = RF_Ins;
    if (RF_Ins == ins_CommandToCoord) {
      delay(200);
      readCoord();
      updateCoords(CoordDesired, RF_X+0.0, RF_Y+0.0, RF_theta+0.0);
    }
    else if (RF_Ins == 4) {
      //Return home command
      delay(200);
      updateCoords(CoordDesired, 0, 0, 0);
      StateCurrent = ins_CommandToCoord;
    }
    else {
      BroadcastCoord();
    }
  }
}
/***************Lidar Functions***************/
void pauseScan() {
  //Updates distance travelled from movfwd in seek state 
  float mm_encoded = Encoder2mm(Val_Encoder1);    //movStop resets encoder values, grab distance travelled then stop, update x,y coords
  movStop();
  //update movFwd coords
  CoordCurrent[0] += mm_encoded * sin(CoordCurrent[2]*PI/180);
  CoordCurrent[1] += mm_encoded * cos(CoordCurrent[2]*PI/180);
}

void scan() {
  //Calls for lidar sensor
  //Lidar angle 0 deg points 6 o'clock, increases clockwise
  if (!Seek_toggle || !IS_OK(lidar.waitPoint())) {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       //detected...
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 200);
       delay(1000);
    }
    Seek_toggle = 1;
  }
}

/***************Movement Functions***************/
void MoveCurrent2Desired() {
  //Moves robot from current coord to desired coord
  float x_diff = CoordDesired[0] - CoordCurrent[0];
  float y_diff = CoordDesired[1] - CoordCurrent[1];
  float turn1 = 0;
  //Figure out first turn angle based on desired coordinates and current coordinates. Finds angle so robot can drive in straight line there
  if (CoordDesired[0] > CoordCurrent[0]) {
    //Quadrant I
    if (CoordDesired[1] > CoordCurrent[1]) {
      turn1 = 180/PI*atan(x_diff/y_diff) - CoordCurrent[2];
    }
    //Quadrant II
    else {
      turn1 = 180 + 180/PI*atan(x_diff/y_diff) - CoordCurrent[2];
    }
  }
  else {
    //Quadrant IV
    if (CoordDesired[1] > CoordCurrent[1]) {
      turn1 = 360 + 180/PI*atan(x_diff/y_diff) - CoordCurrent[2];
    }
    //Quadrant III
    else {
      turn1 = 180 + 180/PI*atan(x_diff/y_diff) - CoordCurrent[2];
    }
  }
  //Accounts for floating point errors. Don't turn if angle is very small
  if (turn1 < -0.1 || turn1 > 0.1) {
    //Determine direction and angle to turn
    if (turn1 < 0) {
      turn1 = 360 + turn1;
    }
    if (turn1 > 180) {
      Rot(L, 360 - turn1);
    }
    else {
      Rot(R, turn1);
    }
  }
  //Move the hypotenuse of x and y coord diff
  if (x_diff < -0.1 || x_diff > 0.1 || y_diff < -0.1 || y_diff > 0.1) {
    float translate_distance = sqrt(sq(x_diff) + sq(y_diff));
    Translate(D, translate_distance);
  }
  //Rotates robot to final desired angle
  float turn2 = CoordDesired[2] - CoordCurrent[2];
  if (turn2 < -0.1 || turn2 > 0.1) {
    if (turn2 < 0) {
      turn2 = 360 + turn2;
    }
    if (turn2 > 180) {
      Rot(L, 360 - turn2);
    }
    else {
      Rot(R, turn2);
    }
  }
}

void Rot(int direction, float deg) {
  //Turns and updates angle
  delay(100);
  movStop();
  int angle_encoded = 0;
  int EncoderCount = Angle2Encoder(deg);            //Determines how many encoder counts cooresponds to a specific angle
  if (direction == L) {
    turnL();
    while(Val_Encoder2 < EncoderCount) {
      if (IRSensor1.distance() < RangeLimitLow/10 - 5) {
        //Object detection code
        break;
      }
    }
    angle_encoded = Encoder2Angle(Val_Encoder2);    //Stores angle that has actually been rotated
    movStop();                                      //movStop resets encoder values, grab distance travelled then stop, update x,y coords
    CoordCurrent[2] = (CoordCurrent[2] - angle_encoded);      //Update coords
  }
  else if (direction == R) {
    turnR();
    while(Val_Encoder1 < EncoderCount) {
      if (IRSensor2.distance() < RangeLimitLow/10 - 5) {
        //Object detection code
        break;
      }
    }
    angle_encoded = Encoder2Angle(Val_Encoder1);    //Stores angle that has actually been rotated
    movStop();
    CoordCurrent[2] = (CoordCurrent[2] + angle_encoded);      //Update coords
  }

  //Make sure angle is between 0 and 360 deg;
  if (CoordCurrent[2] < 0)
    CoordCurrent[2] += 360;
  else if (CoordCurrent[2] > 360)
    CoordCurrent[2] -= 360;
  
  BroadcastCoord();
}

void Translate(int direction, float mm) {
  //Moves and updates x,y coords based on robot angle
  delay(100);
  movStop();
  int EncoderCount = mm2Encoder(mm);         //Determines how many encoder counts cooresponds to a specific distance
  int mm_encoded = 0;
  if (direction == D)
  {
    while(Val_Encoder1 < EncoderCount) {     //Waits until robot travelled the distance
      if (readSonar() < 20 || IRSensor1.distance() < 10 || IRSensor2.distance() < 10) {
        //Object detection code
        movStop();
        break;
      }
      movFwd();   
    }
    mm_encoded = Encoder2mm(Val_Encoder1);   //Stores angle that has actually been moved
    movStop();                               //movStop resets encoder values, grab distance travelled then stop, update x,y coords
    CoordCurrent[0] += mm_encoded * sin(CoordCurrent[2]*PI/180);
    CoordCurrent[1] += mm_encoded * cos(CoordCurrent[2]*PI/180);
  }
  else if (direction == B)
  {
    while(Val_Encoder1 < EncoderCount) {     //Waits until robot travelled the distance 
      // if (used Lidar to read backwards) {
      //   //Object detection code
      //   break;
      // } 
      movBack();
    }
    mm_encoded = Encoder2mm(Val_Encoder1);    //movStop resets encoder values, grab distance travelled then stop, update x,y coords
    movStop();
    CoordCurrent[0] -= mm_encoded * sin(CoordCurrent[2]*PI/180);
    CoordCurrent[1] -= mm_encoded * cos(CoordCurrent[2]*PI/180);
  }
  BroadcastCoord();
}

void updateCoords(float CoordSet[], float x, float y, float theta) {
  //Function allows changing of all 3 indicies of a specific coordinate set at once
  CoordSet[0] = x;
  CoordSet[1] = y;
  //normalize angle
  if (theta < 0)
    theta += 360;
  else if (theta > 360)
    theta -= 360;
  CoordSet[2] = theta;
}

int readSonar() {
  //Uses ultrasonic sensor to detect distance of an object. Output in cm
  unsigned long SonarDuration;
  digitalWrite(Pin_SonarTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Pin_SonarTrig, LOW);
  SonarDuration = pulseIn(Pin_SonarEcho, HIGH);
  digitalWrite(Pin_SonarTrig, LOW);
  return SonarDuration/29/2;
}

/***************Comms Functions***************/
void readCoord() {
  delay(200);
  while(!radio.available());      //Wait until next message arrives to read
  radio.read(&RF_X, sizeof(RF_X));

  delay(200);
  while(!radio.available());
  radio.read(&RF_Y, sizeof(RF_Y));

  delay(200);
  while(!radio.available());
  radio.read(&RF_theta, sizeof(RF_theta));

  //Without the delays, some of the read values may get dropped
}

void BroadcastCoord() {
  //Tells central-ops CoordCurrent
  radio.stopListening();

  //Writes current coords to buffer
  RF_X = (int) CoordCurrent[0];
  RF_Y = (int) CoordCurrent[1];
  RF_theta = (int) CoordCurrent[2];
  //Send the coords
  delay(2000);
  radio.write(&RF_X, sizeof(RF_X));
  delay(1000);
  radio.write(&RF_Y, sizeof(RF_Y));
  delay(1000);
  radio.write(&RF_theta, sizeof(RF_theta));
  delay(500);
  //Without the delays, sent values may get dropped

  lastUpdateTime = millis();      //Stores last time coordinates has been updated
  radio.startListening();         //Start listening again
}

/***************Motor Functions***************/
void movStop() {
  //Turns off all motors and reset encoder counts, speeds
  digitalWrite(Pin_Motor1A, LOW);
  digitalWrite(Pin_Motor1B, LOW);
  digitalWrite(Pin_Motor2A, LOW);
  digitalWrite(Pin_Motor2B, LOW);
  Val_Encoder1 = 0;
  Val_Encoder2 = 0;
  Val_Encoder1_old = 0;
  Val_Encoder2_old = 0;
  speed1 = 0;
  speed2 = 0;
  analogWrite(Pin_Motor1Power, 0);
  analogWrite(Pin_Motor2Power, 0);
}

void movFwd() {
  //Move robot forward
  //Accelerates robot
  if (speed1 < speedOpt - accel + 1) {
    speed1 += accel;
    speed2 += accel;
  }
  else {
    int speed_diff = (Val_Encoder1 - Val_Encoder1_old - Val_Encoder2 + Val_Encoder2_old)/5;    //speed_diff calculation is Tuneable. Adjust depending on surface and slippage
    if (speed_diff > 0) {             //if speed2 too slow. Try to keep speeds around 100
      if (speed2 < speedOpt)          //Tries to keep speeds around speedOpt
        speed2 = speed1 + speed_diff;
      else
        speed1 = speed2 - speed_diff;
    }
    else if (speed_diff < 0) {        //speed1 too slow
      if (speed1 < speedOpt)
        speed1 = speed2 - speed_diff;   //Reversed because speed_diff will be negative
      else
        speed2 = speed1 + speed_diff;
    }
  }
  analogWrite(Pin_Motor1Power, speed1);
  analogWrite(Pin_Motor2Power, speed2);
  digitalWrite(Pin_Motor1A, LOW);
  digitalWrite(Pin_Motor1B, HIGH);
  digitalWrite(Pin_Motor2A, LOW);
  digitalWrite(Pin_Motor2B, HIGH);
  //Stores old encoder values
  Val_Encoder1_old = Val_Encoder1;
  Val_Encoder2_old = Val_Encoder2;
}

void movBack() {
  //Move robot backwards
  //Accelerates robot
  if (speed1 < speedOpt - accel + 1) {
    speed1 += accel;
    speed2 += accel;
  }
  else {
    int speed_diff = (Val_Encoder1 - Val_Encoder1_old - Val_Encoder2 + Val_Encoder2_old)/5;    //speed_diff calculation is Tuneable. Adjust depending on surface and slippage
    if (speed_diff > 0) {             //if speed2 too slow. Try to keep speeds around 100
      if (speed2 < speedOpt)          //Tries to keep speeds around speedOpt
        speed2 = speed1 + speed_diff;
      else
        speed1 = speed2 - speed_diff;
    }
    else if (speed_diff < 0) {        //speed1 too slow
      if (speed1 < speedOpt)
        speed1 = speed2 - speed_diff;   //Reversed because speed_diff will be negative
      else
        speed2 = speed1 + speed_diff;
    }
  }
  analogWrite(Pin_Motor1Power, speed1);
  analogWrite(Pin_Motor2Power, speed2);
  digitalWrite(Pin_Motor1A, HIGH);
  digitalWrite(Pin_Motor1B, LOW);
  digitalWrite(Pin_Motor2A, HIGH);
  digitalWrite(Pin_Motor2B, LOW);
  //Stores old encoder values
  Val_Encoder1_old = Val_Encoder1;
  Val_Encoder2_old = Val_Encoder2;
}

void turnR() {
  //Move robot forward
  analogWrite(Pin_Motor1Power, turnspeed_dom);
  analogWrite(Pin_Motor2Power, turnspeed_sub+2);    //Tuneable. Right motor seems to turn slower than left
  digitalWrite(Pin_Motor1A, LOW);
  digitalWrite(Pin_Motor1B, HIGH);
  digitalWrite(Pin_Motor2A, HIGH);
  digitalWrite(Pin_Motor2B, LOW);
}

void turnL() {
  //Move robot forward
  analogWrite(Pin_Motor1Power, turnspeed_sub);
  analogWrite(Pin_Motor2Power, turnspeed_dom+2);    //Tuneable. Right motor seems to turn slower than left
  digitalWrite(Pin_Motor1A, HIGH);
  digitalWrite(Pin_Motor1B, LOW);
  digitalWrite(Pin_Motor2A, LOW);
  digitalWrite(Pin_Motor2B, HIGH);
}

float Encoder2mm(int Encoder_val) {
  //Changes encoder count to millimeters
  return Encoder_val*65.0*PI/ENC_Count_Rev;
}

int mm2Encoder(float mm) {
  //Changes millimeters to number of encoder counts
  return (int) (mm*ENC_Count_Rev/(65.0*PI));
}

float Encoder2Angle(int Encoder_val) {
  //Changes encoder count to angle
  return Encoder_val*1.0/angencConst;
}

int Angle2Encoder(float angle) {
  //Changes angle to number of encoder counts
  return (int) (angle*angencConst);
}

void updateEncoder1() {
  //increment value for each pulse from encoder 1
  Val_Encoder1++;
}

void updateEncoder2() {
  //increment value for each pulse from encoder 2
  Val_Encoder2++;
}
