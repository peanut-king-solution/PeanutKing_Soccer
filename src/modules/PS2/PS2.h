#ifndef PS2_H
#define PS2_H

#include "PS2X_lib.h"

// PS2 Joystick structure definition
struct PS2JoystickData
{
  float angle;
  float strength;
};

// PS2 button state enumeration
enum PS2ButtonState : uint8_t
{
  PS2Idle = 0,
  PS2Pressed = 1,
  PS2Holding = 2,
  PS2Released = 3,
};

enum PS2Button : uint16_t
{
  PS2Select = PSB_SELECT,
  PS2L3 = PSB_L3,  // left joystick button
  PS2R3 = PSB_R3,  // right joystick button
  PS2Start = PSB_START,
  PS2Up    = PSB_PAD_UP,
  PS2Right = PSB_PAD_RIGHT,
  PS2Down  = PSB_PAD_DOWN,
  PS2Left  = PSB_PAD_LEFT,
  PS2L2 = PSB_L2,
  PS2R2 = PSB_R2,
  PS2L1 = PSB_L1,
  PS2R1 = PSB_R1,
  PS2Triangle = PSB_TRIANGLE,
  PS2Circle   = PSB_CIRCLE,
  PS2Cross    = PSB_CROSS,
  PS2Square   = PSB_SQUARE,
};

enum PS2Joystick : uint8_t
{
  PS2LeftJoystick  = 0,  // Left joystick
  PS2RightJoystick = 1,  // Right joystick
};

#endif  // PS2_H