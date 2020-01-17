#include <PeanutKing_Soccer_V2.h>
static PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

byte 
  Attack = 5,
  Defend = 10,
  MovingSpeed  = 5,
  BallPossession = 5,
  Precision = 5;

//  Do not modifiy
void setup() {
  robot.init();
  robot.enableScanning(true, ALLSENSORS, true);
  robot.EYEBOUNDARY = 120-Attack*10;
}

void strategy(void) {
  static bool outside[4];
  
  static int
    attackSpeed = 80 + MovingSpeed*10,
    defendSpeed = 50 + MovingSpeed*8,
    homeX = 0,
    homeY = 80 - Defend*8;
    
  int16_t 
    direct   = 0,
    speed    = attackSpeed,
    rotation = 0,
    eyeAngle = 0,
    x = 0,
    y = 0,
    MaxEye  = 0,
    reading = 0;
    
  robot.compassRead();
  robot.compoundEyeRead();
  reading = robot.eye[robot.maxEye];
  eyeAngle = robot.eyeAngle;
  
  if ( reading > robot.EYEBOUNDARY ) {
    for ( int i=1; i<3; i++ ) {
      robot.ultrasonicRead(i);
      robot.whiteLineCheck(i);
      if ( robot.isWhite[i] )
        outside[i] = true;
      else if ( robot.ultrasonic[i]>32 )
        outside[i] = false;
    }
    
    rotation = (robot.ultrasonic[right] - robot.ultrasonic[left])/6;
    speed = reading>700 ? attackSpeed-BallPossession*4 : attackSpeed;
    
    if ( outside[left] ) {
      if (eyeAngle<120)
        direct = eyeAngle*1.5;
      else if (eyeAngle>300)
        direct = 0;
      else if (eyeAngle>265)
        speed = 0;
      else if (eyeAngle>195)
        direct = 180;
      else
        direct = 360 - (360-eyeAngle)*1.5;
    }
    else if ( outside[right] ) {
      if (eyeAngle>240)
        direct = 360 - (360-eyeAngle)*1.5;
      else if (eyeAngle<60)
        direct = 0;
      else if (eyeAngle<95)
        speed = 0;
      else if (eyeAngle<165)
        direct = 180;
      else
        direct = eyeAngle*1.5;
    }
    else {
      for ( int i=0; i<=4; i+=3 ) {
        robot.ultrasonicRead(i);
        robot.whiteLineCheck(i);
        if ( robot.isWhite[i] )
          outside[i] = true;
        else if ( robot.ultrasonic[i]>32 )
          outside[i] = false;
      }
      if ( outside[front] && (eyeAngle<100 || eyeAngle>260) )
        speed = 0;
      else if ( outside[back]  && (eyeAngle>270 && eyeAngle>90) )
        speed = 0;
      else if (eyeAngle<180)
        direct = eyeAngle*1.5;
      else
        direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5;
    }
  }
  else {
    for (uint8_t i=0; i<4; i++ )
      robot.ultrasonicRead(i);

    y = robot.ultrasonic[back] - 12 - homeY;
    x = (robot.ultrasonic[left] - robot.ultrasonic[right])/2 - homeX;
    if ( abs(x) > 50 || robot.ultrasonic[left]+robot.ultrasonic[right]<130 )
      y -= 25;
    speed = defendSpeed;
    if ( y > 30 )
      direct = atan( (float)x/y)*180+180;
    else if ( x > 4 )
      direct = 270;
    else if ( x < -4 )
      direct = 90;
    else if ( y > 4 )
      direct = 180;
    else if ( y < -4 )
      direct = 0;
    else {
      direct = 0;
      speed = 0;
      for (uint8_t i=0; i<4; i++ )
        outside[i] = false;
    }
  }
  if ( speed == 0 )
    robot.motorStop();
  else
    robot.moveSmart(direct,  speed, rotation, Precision);
}


void loop() {
  static uint32_t bttimer = 0;
  static bool start = false;
  if (robot.buttTrigRead(1)) {
    start = ! start;
    if ( !start ) {
      robot.enableScanning(true, ALLSENSORS, true);
      robot.motorStop();
    }
    else
      robot.enableScanning(false, ALLSENSORS, false);
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
