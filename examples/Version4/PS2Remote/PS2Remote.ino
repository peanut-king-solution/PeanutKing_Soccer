/**
 * PeanutKing Soccer V4 PS2 Remote Control Example
 * Demonstrates how to control the PeanutKing Soccer V4 robot using a PS2 controller.
 */

#include <PeanutKingSoccerV4.h>
PeanutKingSoccerV4 robot;

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

  // Enable compass correction and disable out-of-bounds prevention for joystick control
  robot.compassCorrectEnabled = true;
  robot.outBoundPreventEnabled = false;
}

bool handleJoystick(PS2Button triggerBtn, PS2Joystick stick) {
  if (robot.ps2ButtonStateRead(triggerBtn) != PS2Holding) { return false; }
  PS2JoystickData js = robot.ps2JoystickRead(stick);
  // Print joystick data for debugging
  Serial.print("Joystick ");
  Serial.print((stick == PS2LeftJoystick) ? "L" : "R");
  Serial.print(" angle:"); Serial.print(js.angle);
  Serial.print(" str:");   Serial.println(js.strength);
  // Set vibration strength based on joystick strength
  robot.ps2SetVibration(js.strength);
  // Move robot with compass correction based on joystick angle and scaled strength
  float moveSpeed = js.strength * 130 / 255; // Scale strength to speed (0-130)
  robot.move(js.angle, moveSpeed);
  return true;
}

bool leftControlling = false;
bool rightControlling = false;
void loop() {
  robot.ps2Update();
  // Handle left joystick if L1 is held
  leftControlling = handleJoystick(PS2L1, PS2LeftJoystick);

  // If L1 is not held, check R1 and handle right joystick
  if (!leftControlling){
    // Handle right joystick if R1 is held
    rightControlling = handleJoystick(PS2R1, PS2RightJoystick);
  }

  // If neither joystick is controlling, stop movement and vibration
  if (!leftControlling && !rightControlling) {
    // Stop movement and compass correction
    robot.move(0, 0);
    // Stop vibration
    robot.ps2SetVibration(0);
  }
}