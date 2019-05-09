#include <Arduino.h>
#include <PeanutKing_Soccer.h>

static PeanutKing_Soccer robot = PeanutKing_Soccer(3);
static bool start = false;

void setup() {
  robot.init();
  //robot.autoScanEnabled = false;
  //robot.autoScanSensors = (ALLSENSOR & ~COMPOUNDEYE);
  robot.EYEBOUNDARY = 200;
  robot.ledShow(255, 0, 0, 0, 0);
  robot.ledUpdate();
}

void loop() {
  if ( robot.buttonRead(0) ) {
    start = !start;
    if ( !start ) {
      for (int j=0; j<4; j++) {
        robot.motorSet( j, 0 );
      }
    }
  }
  if ( !start ) {
    robot.debugging(1000, ALLSENSOR);
    return;
  }
  robot.strategy();

}
