#include <PeanutKingSoccerV3.h>
static PeanutKingSoccerV3 robot = PeanutKingSoccerV3();

void setup(void) {
  robot.init();
}

void loop() {
  static uint32_t dataFetchTimer = 0;
  robot.bluetoothRemote();
  
  if (millis() - dataFetchTimer > 50) {
    robot.dataFetch();
    dataFetchTimer = millis();
  }
}
