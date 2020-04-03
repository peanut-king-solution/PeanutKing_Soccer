#include <PeanutKing_Soccer_V2.h>
static PeanutKing_Soccer_V2 robot = PeanutKing_Soccer_V2();

void setup() {
  robot.init();
  robot.enableScanning(true, ALLSENSORS, false);
}

void loop() {
  //robot.btTest();
//  robot.lcdMenu();
  robot.bluetoothRemote();
}
