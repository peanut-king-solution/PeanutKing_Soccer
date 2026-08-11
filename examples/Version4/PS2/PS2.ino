/**
 * PeanutKing Soccer V4 PS2 Example
 * Demonstrates how to use the PS2 controller with the PeanutKing Soccer V4 robot.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup()
{
  robot.init();
  delay(300);

  // CLK=D6_P, DAT=D3_P, middle pins CMD, ATT auto assigned
  byte error = robot.ps2Init(D6_P, D3_P, false, true);
  if (error)
  {
    Serial.print("PS2 init error: ");
    Serial.println(error);
    while (1) {};
  }
  Serial.println("PS2 OK");
}

void loop()
{
  // Update PS2 controller state
  robot.ps2Update();
  // ===== Check button states =====
  PS2ButtonState crossState = robot.ps2ButtonStateRead(PS2Cross);
  if (crossState == PS2Pressed) { Serial.println("CROSS pressed"); }

  PS2ButtonState circleState = robot.ps2ButtonStateRead(PS2Circle);
  if (circleState == PS2Pressed) { Serial.println("CIRCLE pressed"); }

  PS2ButtonState triangleState = robot.ps2ButtonStateRead(PS2Triangle);
  if (triangleState == PS2Pressed) { Serial.println("TRIANGLE pressed"); }

  PS2ButtonState squareState = robot.ps2ButtonStateRead(PS2Square);
  if (squareState == PS2Pressed) { Serial.println("SQUARE pressed"); }

  // ===== Directional buttons (holding) =====
  PS2ButtonState upState = robot.ps2ButtonStateRead(PS2Up);
  if (upState == PS2Holding) { Serial.println("UP holding"); }

  PS2ButtonState downState = robot.ps2ButtonStateRead(PS2Down);
  if (downState == PS2Holding) { Serial.println("DOWN holding"); }

  PS2ButtonState leftState = robot.ps2ButtonStateRead(PS2Left);
  if (leftState == PS2Holding) { Serial.println("LEFT holding"); }

  PS2ButtonState rightState = robot.ps2ButtonStateRead(PS2Right);
  if (rightState == PS2Holding) { Serial.println("RIGHT holding"); }

  // ===== Middle buttons =====
  PS2ButtonState startState = robot.ps2ButtonStateRead(PS2Start);
  if (startState == PS2Pressed) { Serial.println("START"); }

  PS2ButtonState selectState = robot.ps2ButtonStateRead(PS2Select);
  if (selectState == PS2Pressed) { Serial.println("SELECT"); }

  // ===== Shoulder buttons =====
  PS2ButtonState l2State = robot.ps2ButtonStateRead(PS2L2);
  if (l2State == PS2Pressed) { Serial.println("L2 pressed"); }

  PS2ButtonState r2State = robot.ps2ButtonStateRead(PS2R2);
  if (r2State == PS2Pressed) { Serial.println("R2 pressed"); }

  // ===== Joysticks =====
  PS2ButtonState l1State = robot.ps2ButtonStateRead(PS2L1);
  if (l1State == PS2Holding)
  {
    PS2JoystickData lj = robot.ps2JoystickRead(PS2LeftJoystick);
    Serial.print("L angle:");
    Serial.print(lj.angle);
    Serial.print(" str:");
    Serial.println(lj.strength);

    // Set vibration strength based on left joystick strength
    robot.ps2SetVibration(lj.strength);
  }

  PS2ButtonState r1State = robot.ps2ButtonStateRead(PS2R1);
  if (r1State == PS2Holding)
  {
    PS2JoystickData rj = robot.ps2JoystickRead(PS2RightJoystick);
    Serial.print("R angle:");
    Serial.print(rj.angle);
    Serial.print(" str:");
    Serial.println(rj.strength);
  }

  // ===== Button Release Detection =====
  if (l1State == PS2Released)
  {
    Serial.println("L1 released");
    // Stop vibration when L1 is released
    robot.ps2SetVibration(0);
  }
  if (r1State == PS2Released)
  {
    Serial.println("R1 released");
  }
  delay(50);
}