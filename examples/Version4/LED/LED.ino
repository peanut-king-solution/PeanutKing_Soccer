#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Cycle through all 8 colors every 1 sec
  robot.setOnBrdLED(LEDColor::OFF);    delay(1000);
  robot.setOnBrdLED(LEDColor::BLUE);   delay(1000);
  robot.setOnBrdLED(LEDColor::GREEN);  delay(1000);
  robot.setOnBrdLED(LEDColor::CYAN);   delay(1000);
  robot.setOnBrdLED(LEDColor::RED);    delay(1000);
  robot.setOnBrdLED(LEDColor::PURPLE); delay(1000);
  robot.setOnBrdLED(LEDColor::YELLOW); delay(1000);
  robot.setOnBrdLED(LEDColor::WHITE);  delay(1000);
}