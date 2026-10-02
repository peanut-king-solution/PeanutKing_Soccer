/**
 * This example demonstrates how to read the state of buttons on the PeanutKingSoccerV4 robot.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup()
{
  robot.init();
}

void loop()
{
  // Update button states
  robot.buttonUpdate();

  // Check each button's state (after buttonUpdate() to get the latest state)
  for (int button = Button1; button <= Button4; button++)
  {
    // Read the current state of the button
    ButtonState state = robot.buttonStateRead(button);
    // Print the button state to the Serial Monitor
    switch (state)
    {
    case ButtonIdle:
      break;
    case ButtonPressed:
      Serial.print("Button ");
      Serial.print(button);
      Serial.println(" - PRESSED");
      break;
    case ButtonHolding:
      Serial.print("Button ");
      Serial.print(button);
      Serial.println(" - HOLDING");
      break;
    case ButtonReleased:
      Serial.print("Button ");
      Serial.print(button);
      Serial.println(" - RELEASED");
      break;
    }
  }

  delay(5);
}
