#include <PeanutKingSoccerV4.h>

PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup()
{
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

void loop()
{
  // Update PS2 controller state
  robot.ps2Update();
  // ===== Right side Buttons =====
  if (robot.ps2ButtonPressed(PS2Button::CROSS))
  {
    Serial.println("CROSS pressed");
  }
  if (robot.ps2ButtonPressed(PS2Button::CIRCLE))
  {
    Serial.println("CIRCLE pressed");
  }
  if (robot.ps2ButtonPressed(PS2Button::TRIANGLE))
  {
    Serial.println("TRIANGLE pressed");
  }
  if (robot.ps2ButtonPressed(PS2Button::SQUARE))
  {
    Serial.println("SQUARE pressed");
  }
  // ===== Left side Buttons =====
  if (robot.ps2ButtonHolding(PS2Button::UP))
  {
    Serial.println("UP holding");
  }
  if (robot.ps2ButtonHolding(PS2Button::DOWN))
  {
    Serial.println("DOWN holding");
  }
  if (robot.ps2ButtonHolding(PS2Button::LEFT))
  {
    Serial.println("LEFT holding");
  }
  if (robot.ps2ButtonHolding(PS2Button::RIGHT))
  {
    Serial.println("RIGHT holding");
  }
  // ===== Middle side Buttons =====
  if (robot.ps2ButtonPressed(PS2Button::START))
  {
    Serial.println("START");
  }
  if (robot.ps2ButtonPressed(PS2Button::SELECT))
  {
    Serial.println("SELECT");
  }
  // ===== Shoulder Buttons =====
  if (robot.ps2ButtonPressed(PS2Button::L2))
  {
    Serial.println("L2 pressed");
  }
  if (robot.ps2ButtonPressed(PS2Button::R2))
  {
    Serial.println("R2 pressed");
  }
  // ===== Joysticks =====
  if (robot.ps2ButtonHolding(PS2Button::L1))
  {
    PS2JoystickData lj = robot.ps2JoystickRead(PS2Joystick::LEFT);
    Serial.print("L angle:");
    Serial.print(lj.angle);
    Serial.print(" str:");
    Serial.println(lj.strength);

    // Set vibration strength based on left joystick strength
    robot.ps2SetVibration(lj.strength);
  }
  if (robot.ps2ButtonHolding(PS2Button::R1))
  {
    PS2JoystickData rj = robot.ps2JoystickRead(PS2Joystick::RIGHT);
    Serial.print("R angle:");
    Serial.print(rj.angle);
    Serial.print(" str:");
    Serial.println(rj.strength);
  }
  // ===== Button Release Detection =====
  if (robot.ps2ButtonReleased(PS2Button::L1))
  {
    Serial.println("L1 released");
    // Stop vibration when L1 is released
    robot.ps2SetVibration(0);
  }
  if (robot.ps2ButtonReleased(PS2Button::R1))
  {
    Serial.println("R1 released");
  }
  delay(50);
}
