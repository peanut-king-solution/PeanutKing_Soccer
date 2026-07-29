#include <PeanutKingSoccerV4.h>

PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  delay(300);

  // CLK=D6_P(56), DAT=D3_P(59), middle pins CMD=57, ATT=58 auto assigned
  byte error = robot.ps2Init(D6_P, D3_P, false, true);
  if (error)
  {
    Serial.print("PS2 init error: ");
    Serial.println(error);
    while (1) {};
  }
  Serial.println("PS2 OK");
}

void loop() {
  // Update PS2 controller state
  robot.ps2Update();

  // Check if L1 button is being held down
  if (robot.ps2ButtonHolding(PS2Button::L1))
  {
    // Read left joystick data
    PS2JoystickData lj = robot.ps2JoystickRead(PS2Joystick::LEFT);
    Serial.print("L angle:");
    Serial.print(lj.angle);
    Serial.print(" str:");
    Serial.println(lj.strength);

    // Set vibration strength based on left joystick strength
    robot.ps2SetVibration(lj.strength);

    // Remap strength from 0-255 to 0-130 for motor speed
    int moveSpeed = lj.strength * 130 / 255;)

    // Move robot based on left joystick angle and remapped speed
    robot.moveByAnglePID(lj.angle, moveSpeed);
  }
  if (robot.ps2ButtonReleased(PS2Button::L1)) {
    // Stop vibration when L1 is released
    robot.ps2SetVibration(0);
    // Stop robot movement when L1 is released
    robot.stopAllMotors();
  }
}