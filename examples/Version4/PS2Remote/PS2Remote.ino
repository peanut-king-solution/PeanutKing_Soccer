#include <PeanutKingSoccerV4.h>

PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  delay(300);

  byte error = robot.ps2Init(D6_P, D3_P, false, true);
  if (error) {
    Serial.print("PS2 init error: ");
    Serial.println(error);
    while (1) {};
  }
  Serial.println("PS2 OK");
}

bool handleJoystick(PS2Button triggerBtn, PS2Joystick stick) {
  if (!robot.ps2ButtonHolding(triggerBtn)) { return false; }
  PS2JoystickData js = robot.ps2JoystickRead(stick);
  // Print joystick data for debugging
  Serial.print("Joystick ");
  Serial.print((stick == PS2Joystick::LEFT) ? "L" : "R");
  Serial.print(" angle:"); Serial.print(js.angle);
  Serial.print(" str:");   Serial.println(js.strength);
  // Set vibration strength based on joystick strength
  robot.ps2SetVibration(js.strength);
  // Move robot with compass correction based on joystick angle and scaled strength
  float moveSpeed = js.strength * 130 / 255; // Scale strength to speed (0-130)
  robot.moveWithCorr(js.angle, moveSpeed);
  return true;
}

bool leftControlling = false;
bool rightControlling = false;
void loop() {
  robot.ps2Update();
  // Handle left joystick if L1 is held
  leftControlling = handleJoystick(PS2Button::L1, PS2Joystick::LEFT);
  
  // If L1 is not held, check R1 and handle right joystick
  if (!leftControlling){ 
    // Handle right joystick if R1 is held
    rightControlling = handleJoystick(PS2Button::R1, PS2Joystick::RIGHT);
  }

  // If neither joystick is controlling, stop movement and vibration
  if (!leftControlling && !rightControlling) {
    // Stop movement and compass correction
    robot.moveWithCorr(0, 0);
    // Stop vibration
    robot.ps2SetVibration(0);
  }
}
