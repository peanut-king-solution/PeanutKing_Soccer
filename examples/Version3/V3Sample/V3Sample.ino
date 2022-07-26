#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup() {
  robot.init();
//  robot.enableScanning(true, ALLSENSORS, false);
}

long timer = 0;
byte data[4];
byte cmd = 0x55;
int i=0;

void loop() {
  if (millis() - timer > 1000){
    timer = millis();
    robot.dataFetch();
    
    Serial.print("Compass: ");
    Serial.print(robot.compass);
    Serial.println();
    
    Serial.print("U: ");
    for(int i=0; i<4; i++) {
      Serial.print(robot.ultrasonic[i]);
      Serial.print(' ');
    }
    Serial.println();
    
    Serial.print("I: ");
    for(int i=0; i<12; i++) {
      Serial.print(robot.eye[i]);
      Serial.print(' ');
    }
    Serial.println();
    
    Serial.print("C: ");
    for(int i=0; i<4; i++) {
      Serial.print(robot.colorRGB[i].r);
      Serial.print(' ');
      Serial.print(robot.colorRGB[i].g);
      Serial.print(' ');
      Serial.print(robot.colorRGB[i].b);
      Serial.print("    ");
    }
    Serial.println();
  }
}

void singleSensorTest (void) {
//  robot.I2CSensorRead(robot.topBoardAddr,Ultrasonic,8);
  for(int i=0; i<8; i++) {
    char c = robot.rxBuff[i];
    //Serial.print(c);
    Serial.print(robot.rxBuff[i]);
    Serial.print(' ');
  }
  Serial.println();
}


void i2cTest (void) {
  Serial.print(data[0]);
  Serial.print(' ');
  Serial.print(data[1]);
  Serial.print(' ');
  Serial.print(data[2]);
  Serial.print(' ');
  Serial.println(data[3]);
}
