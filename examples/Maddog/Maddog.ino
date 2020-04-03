#include <PeanutKing_Soccer_V2.h>

PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  robot.init();
  robot.enableScanning(false, ALLSENSORS, false);
  delay(1000);
}

void loop() {
  int16_t 
    moveSpeed = 80,
    MaxEye  = robot.compoundEyeRead(13),
    reading = robot.compoundEyeRead(14);
    
  robot.compassRead();

  if ( reading>100 ) {
    switch(MaxEye) {
      case 1:
        robot.motorSet(0, 70);
        robot.motorSet(1, 70);
        robot.motorSet(2, -70);
        robot.motorSet(3, -70);
        break;
      case 2:
        robot.motorSet(0, 80);
        robot.motorSet(1, 40);
        robot.motorSet(2, -80);
        robot.motorSet(3, -40);
        break;
      case 3:
        robot.motorSet(0, 80);
        robot.motorSet(1, -40);
        robot.motorSet(2, -80);
        robot.motorSet(3, 40);
        break;
      case 4:
        robot.motorSet(0, 70);
        robot.motorSet(1, -70);
        robot.motorSet(2, -70);
        robot.motorSet(3, 70);
        break;
      case 5:
        robot.motorSet(0, 40);
        robot.motorSet(1, -80);
        robot.motorSet(2, -40);
        robot.motorSet(3, 80);
        break;
      case 6:
        robot.motorSet(0, -40);
        robot.motorSet(1, -80);
        robot.motorSet(2, 40);
        robot.motorSet(3, 80);
        break;
      case 7:
        robot.motorSet(0, -70);
        robot.motorSet(1, -70);
        robot.motorSet(2, 70);
        robot.motorSet(3, 70);
        break;
      case 8:
        robot.motorSet(0, -80);
        robot.motorSet(1, -40);
        robot.motorSet(2, 80);
        robot.motorSet(3, 40);
        break;
      case 9:
        robot.motorSet(0, -80);
        robot.motorSet(1, 40);
        robot.motorSet(2, 80);
        robot.motorSet(3, -40);
        break;
      case 10:
        robot.motorSet(0, -70);
        robot.motorSet(1, 70);
        robot.motorSet(2, 70);
        robot.motorSet(3, -70);
        break;
      case 11:
        robot.motorSet(0, -40);
        robot.motorSet(1, 80);
        robot.motorSet(2, 40);
        robot.motorSet(3, -80);
        break;
      case 12:
        robot.motorSet(0, 40);
        robot.motorSet(1, 80);
        robot.motorSet(2, -40);
        robot.motorSet(3, -80);
        break;
    }
  }
  else {
    robot.motorSet(0, 0);
    robot.motorSet(1, 0);
    robot.motorSet(2, 0);
    robot.motorSet(3, 0);
  }
}
