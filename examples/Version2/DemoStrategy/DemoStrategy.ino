#include <PeanutKingSoccerV2.h>
static PeanutKingSoccerV2 robot = PeanutKingSoccerV2();

// Attributes =======================================
byte 
  Attack = 10,              // Sensitivity to the ball
  Defend = 9,              // Closenest to the goal
  MovingSpeed  = 4,         // Speed
  BallPossession = 5,       // Slow when close to the ball
  Precision = 5;            // Spinning speed for compass compliment


//  Do not modifiy ================================
void setup() {
  robot.init();
  robot.enableScanning(true, ALLSENSORS, true);
  robot.EYEBOUNDARY = 10 + (10-robot.btAttributes[0]) * 20;
}

static bool outside[4];

static int
  attackSpeed = 80 + robot.btAttributes[2]*16,
  defendSpeed = 50 + robot.btAttributes[2]*12,
  BallPossessionSpeed = attackSpeed-robot.btAttributes[3]*4,
  homeX = 0,
  homeY = (10-robot.btAttributes[1]) * 8;


bool zoneControl(uint16_t moveAngle) {
  uint8_t quadrant;
  if (moveAngle < 45 || moveAngle > 315)
    quadrant = front;
  else if (moveAngle < 135)
    quadrant = right;
  else if (moveAngle < 225)
    quadrant = back;
  else
    quadrant = left;

  if (quadrant==back || quadrant==front)
    robot.ultrasonicRead(quadrant);
  
  if ( robot.ultrasonic[quadrant]>31 )
    outside[quadrant] = false;
  else {
    robot.whiteLineCheck(quadrant);
    if ( robot.isWhite[quadrant] )
      outside[quadrant] = true;
  }
  return outside[quadrant];
}

void strategy(void) {
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
    speed = reading>500 ? BallPossessionSpeed : attackSpeed;

    robot.ultrasonicRead(left);
    robot.ultrasonicRead(right);
    //rotation = (robot.ultrasonic[right] - robot.ultrasonic[left])/6;
    
    if (eyeAngle<135)
      direct = eyeAngle*1.5;
    else if (eyeAngle>225)
      direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5;
    else {
      if (robot.ultrasonic[left] > robot.ultrasonic[right])
        direct = eyeAngle*1.5;
      else
        direct = eyeAngle * 1.5 - 180; //359 - (359-eyeAngle)*1.5;
    }
    
    if ( zoneControl(direct) )
      speed = 0;
    /*
    if ( outside[front] && (eyeAngle<100 || eyeAngle>260) )
      speed = 0;
    else if ( outside[back]  && (eyeAngle>270 && eyeAngle>90) )
      speed = 0;
    else 
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
    */
  }
  else {
    outside[front] = false;
    for (uint8_t i=1; i<4; i++ ) {
      robot.ultrasonicRead(i);
      if ( robot.ultrasonic[i]>30 )
        outside[i] = false;
    }

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
    }
  }
  robot.moveSmart(direct,  speed, rotation, robot.btAttributes[4]);
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

  robot.bluetoothAttributes();
  attackSpeed = 80 + robot.btAttributes[2]*16;
  defendSpeed = 50 + robot.btAttributes[2]*12;
  BallPossessionSpeed = attackSpeed-robot.btAttributes[3]*4;
  homeX = 0;
  homeY = (10-robot.btAttributes[1]) * 8;
  
  /*
  if (Serial1.available()) {
    if ( millis() - bttimer > 100 ) {
      Serial1.println(robot.compass);
      bttimer = millis();
    }
  }*/
}
