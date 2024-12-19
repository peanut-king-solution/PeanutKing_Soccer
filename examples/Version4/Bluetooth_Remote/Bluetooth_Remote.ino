#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup(void) {
  robot.init();
}

void loop() {
  robot.bluetoothRemote();
  
}
