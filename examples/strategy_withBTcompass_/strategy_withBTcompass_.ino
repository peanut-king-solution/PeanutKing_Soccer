#include <PeanutKing_Soccer_V2.h>
static PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

bool start = false;

void setup() {
  robot.init();
  robot.enableScanning(true, ALLSENSORS, true);
  robot.EYEBOUNDARY = 20;
}

void strategy(void) {
  //static int16_t stuckTime;
  //static bool stuckFlag;
  
  static bool outside[4];
  
  int16_t direct, speed,
    eyeAngle = robot.eyeAngle,
    x = (robot.ultrasonic[left] - robot.ultrasonic[right])/2,
    y = robot.ultrasonic[back] - 15,
    ballAngle = robot.maxEye,
    reading = robot.eye[ballAngle];
    
  for ( int i=0; i<4; i++ ) {
    if ( robot.isWhite[i] && !outside[i] )
      outside[i] = true;
  }
    
  if ( reading > 10 ) {
    speed = 180;
    if ( robot.outBound[left] ) {
      switch (ballAngle) {
        case 1:
        case 12:
          direct = 0;
          break;
        case 11:
          direct = 0;
          speed = 0;
          break;
        case 10:
        case 9:
        case 8:
          direct = 180;
          break;
        default:
          direct = eyeAngle*1.5;
        for (uint8_t i=0; i<4; i++ )
          outside[i] = false;
      }
    }
    else if ( robot.outBound[right] ) {
      switch (ballAngle) {
        case 1:
        case 2:
          direct = 0;
        break;
        case 3:
          direct = 0;
          speed = 0;
        break;
        case 4:
        case 5:
        case 6:
          direct = 180;
        break;
        default:
          direct = 360 - (360-eyeAngle)*1.5;
        for (uint8_t i=0; i<4; i++ )
          outside[i] = false;
      }
    }
/*
    else if ( robot.outBound[front] && robot.outBound[front] )
      stop();          // Stop
*/
    else {
      if (eyeAngle<180)
        direct = eyeAngle*1.5;
      else
        direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5;
      for (uint8_t i=0; i<4; i++ )
        outside[i] = false;
    }
  }
  else {
    speed = 150;
    if ( x > 5 )
      direct = 270;
    else if ( x < -5 )
      direct = 90;
    else if ( y > 5 )
      direct = 180;
    else if ( y < -5 )
      direct = 0;
    else {
      direct = 180;
      speed = 0;
      for (uint8_t i=0; i<4; i++ )
        outside[i] = false;
    }
  }

  if ( speed == 0 )
    robot.motorStop();
  else
    robot.moveSmart(direct,  speed);
}

uint32_t bttimer = 0;
void loop() {
  if (robot.buttTrigRead(1)) {
    start = ! start;
    if ( !start )
      robot.motorStop();
  }
  
  if ( start )
    strategy();
  else {
    robot.lcdMenu();
  }
  
    
  if (Serial1.available()) {
   if ( millis() - bttimer > 100 ) {
      Serial1.println(robot.compass);
      bttimer = millis();
    }
  }
}
