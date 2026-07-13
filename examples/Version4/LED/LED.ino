#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Cycle through all 8 colors every 1 sec
  robot.setOnBrdLED(LED_OFF);    delay(1000);
  robot.setOnBrdLED(LED_BLUE);   delay(1000);
  robot.setOnBrdLED(LED_GREEN);  delay(1000);
  robot.setOnBrdLED(LED_CYAN);   delay(1000);
  robot.setOnBrdLED(LED_RED);    delay(1000);
  robot.setOnBrdLED(LED_PURPLE); delay(1000);
  robot.setOnBrdLED(LED_YELLOW); delay(1000);
  robot.setOnBrdLED(LED_WHITE);  delay(1000);
}