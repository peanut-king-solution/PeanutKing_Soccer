#include <PeanutKing_Soccer_V2.h>
static PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

int lednum = 0;
uint32_t ledtime = 0;

void setup() {
  robot.init();
  //robot.EYEBOUNDARY = 20;
  robot.enableScanning(false, ALLSENSORS, false);
  
  robot.setScreen(0, 0, "Pressure Test");
  //while (robot.compass == 400) delay(100);
}

void loop() {
  //robot.debug(COMPOUNDEYE + COLORSENSOR);
  //robot.debug(ALLSENSORS);
  //delay(1000);
  //robot.pressureTest();

  robot.setScreen(0, 0, "Button 0 Test");
  while ( !robot.buttTrigRead(0) ) { delay(10); }
  robot.setScreen(0, 0, "Button 1 Test");
  while ( !robot.buttTrigRead(1) ) { delay(10); }
  robot.setScreen(0, 0, "Button 2 Test");
  while ( !robot.buttTrigRead(2) ) { delay(10); }
  
  robot.setScreen(0, 0, "LED Test");
  robot.ledShow(255, 255, 0, 0, 0);
  robot.ledUpdate();
  while ( !robot.buttTrigRead(1) ) { delay(10); }
  robot.ledShow(255, 0, 255, 0, 0);
  robot.ledUpdate();
  while ( !robot.buttTrigRead(1) ) { delay(10); }
  robot.ledShow(255, 0, 0, 255, 0);
  robot.ledUpdate();
  while ( !robot.buttTrigRead(1) ) { delay(10); }
  robot.ledShow(255, 0, 0, 0, 255);
  robot.ledUpdate();
  while ( !robot.buttTrigRead(1) ) { delay(10); }

  robot.lcdClear();
  robot.ledShow(255, 0, 0, 0, 0);
  robot.ledUpdate();
  robot.enableScanning(true, ULTRASONIC, false);
  while ( !robot.buttTrigRead(1) ) {
    robot.setScreen(0, 0, robot.ultrasonic[0]);
    robot.setScreen(8, 0, robot.ultrasonic[1]);
    robot.setScreen(0, 1, robot.ultrasonic[2]);
    robot.setScreen(8, 1, robot.ultrasonic[3]);
    delay(80);
  }
  
  //robot.testProgram();
  //robot.btTest();
}
