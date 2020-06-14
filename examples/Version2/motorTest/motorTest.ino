#include <PeanutKing_Soccer_V2.h>
static PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  robot.init();
  //robot.EYEBOUNDARY = 20;
  robot.enableScanning(false, ALLSENSORS, false);
  //while (robot.compass == 400) delay(100);
}

void loop() {
  robot.setScreen(0, 0, "Motor Test +");
  for(int i = 0; i<4; i++ ) {
    robot.motorSet(i, 100);
  }

  while ( !robot.buttTrigRead(1) ) { delay(10); }
  
  robot.setScreen(0, 0, "Motor Test -");
  for(int i = 0; i<4; i++ ) {
    robot.motorSet(i, -100);
  }

  while ( !robot.buttTrigRead(1) ) { delay(10); }
}
