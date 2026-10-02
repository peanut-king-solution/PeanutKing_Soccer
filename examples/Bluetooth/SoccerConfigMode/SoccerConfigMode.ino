// Soccer Config Mode Example
// Library auto-builds config with built-in joystick + SoccerBtn + SoccerTog.
// Joystick, Z LED, and all legacy commands (J/P/C/Z/H/D) are handled internally.

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

// ── Single action button callback ────────────────────────────
void onClick(bool pressed) {
  if (pressed) robot.onBoardLedSet(LEDRed);
  else robot.onBoardLedSet(LEDOff);
}

void setup() {
  robot.init();
  robot.bluetoothInit(&Serial1, PILA_CONFIG);

  robot.compassCorrectEnabled = true;

  // Register callback + add widgets

  // if you want to use a single action button, use the following:
  // robot.bluetoothOnButton("Click", onClick);
  // robot.bluetoothSetSoccerButton(ButtonType);         // B,SoccerBtn

  // Toggle button should not use callbacks, but rather poll the state
  robot.bluetoothSetSoccerButton(ToggleButtonType); // TB,SoccerTog

  robot.bluetoothSetSoccerOutput("compass");
  robot.bluetoothSetSoccerOutput("ultrasoundFront");
  robot.bluetoothSetSoccerOutput("ballAngle");
}

uint32_t lastSensorSendTime = 0;

void loop() {
  robot.bluetoothRemote();
  
  // Only fetch data and send outputs if the toggle button is ON
  if (!robot.bluetoothGetToggle()) return;

  // Send sensor data every 100ms only if the config is sent
  if (robot.bluetoothIsConfig() && millis() - lastSensorSendTime >= 100) {
    robot.dataFetch();
    robot.bluetoothSendOutput("compass", (int)robot.heading);
    robot.bluetoothSendOutput("ultrasoundFront", (int)robot.distances[0]);
    robot.bluetoothSendOutput("ballAngle", (int)robot.irAngle);
    lastSensorSendTime = millis();
  }
}
