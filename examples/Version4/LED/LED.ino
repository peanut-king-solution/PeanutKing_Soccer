#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Cycle through all 8 colors every 1 sec
  robot.onBoardLedSet(LEDOff);    delay(1000);
  robot.onBoardLedSet(LEDBlue);   delay(1000);
  robot.onBoardLedSet(LEDGreen);  delay(1000);
  robot.onBoardLedSet(LEDCyan);   delay(1000);
  robot.onBoardLedSet(LEDRed);    delay(1000);
  robot.onBoardLedSet(LEDPurple); delay(1000);
  robot.onBoardLedSet(LEDYellow); delay(1000);
  robot.onBoardLedSet(LEDWhite);  delay(1000);
}