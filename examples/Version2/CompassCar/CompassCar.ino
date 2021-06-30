#include <PeanutKingSoccerV2.h>

PeanutKingSoccerV2 robot = PeanutKingSoccerV2();

void setup() {
  robot.init();
  robot.enableScanning(false, COMPASS, false);
  
  delay(1000);
}

void loop() {
  compassCarAdvance();

  int angle = robot.compassRead();
  int rotate;

  if ( angle<10 || angle>350 ) rotate = 0;
  else if ( angle<180)         rotate = -40;
  else                         rotate = 40;

  robot.motorSet(0, rotate);
  robot.motorSet(1, rotate);
  robot.motorSet(2, rotate);
  robot.motorSet(3, rotate);

  delay(50);
}


void compassCarAdvance() {
  int angle = robot.compassRead();
  int rotate;

  if ( angle<8 || angle>352 ) rotate = 0;
  else if ( angle< 45)        rotate = -25;
  else if ( angle<180)        rotate = -50;
  else if ( angle<315)        rotate = 50;
  else                        rotate = 25;

  robot.motorSet(0, rotate);
  robot.motorSet(1, rotate);
  robot.motorSet(2, rotate);
  robot.motorSet(3, rotate);

  delay(50);
}
