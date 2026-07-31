#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Update button state machine (detects TAP, TAP2, TAP3, HOLD, etc.)
  robot.buttonUpdate();

  // Check each button's status
  for (int i = 1; i <= 4; i++) {
    ButtonStatus status = robot.button.getStatus((ButtonId) i);
    switch (status) {
      case TAP:   Serial.print("Button "); Serial.print(i); Serial.println(" - TAP"); break;
      case TAP2:  Serial.print("Button "); Serial.print(i); Serial.println(" - DOUBLE TAP"); break;
      case TAP3:  Serial.print("Button "); Serial.print(i); Serial.println(" - TRIPLE TAP"); break;
      case HOLD:  Serial.print("Button "); Serial.print(i); Serial.println(" - HOLD"); break;
      case HOLD2: Serial.print("Button "); Serial.print(i); Serial.println(" - HOLD2"); break;
      case PRESS: Serial.print("Button "); Serial.print(i); Serial.println(" - PRESS"); break;
      default: break;
    }
  }
}
