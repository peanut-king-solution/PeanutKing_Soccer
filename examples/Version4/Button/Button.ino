#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  bool noButtonPressed = true;
  // Check all buttons' state (pressed or not) and print the result to Serial Monitor
  for (int i = 1; i <=4; i++) {
    // Parameter `i` is changed to `ButtonId` enum type to read the button state
    // if call singlely, can use `robot.buttonRead(BTN_1)` or `robot.buttonRead(BTN_2)` etc.

    // Check if the button is pressed
    if (robot.buttonRead((ButtonId) i)) {
      // Print the button state to Serial Monitor
      Serial.print("Button ");
      Serial.print(i);
      Serial.println(" pressed");
      // Have at least one button pressed, so set the flag to false
      noButtonPressed = false;
    }
  }
  // No button pressed
  if (noButtonPressed) {
    Serial.println("No button pressed");
  }
}
