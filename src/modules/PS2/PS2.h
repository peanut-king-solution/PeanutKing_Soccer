#ifndef PS2_H
#define PS2_H

#include "PS2X_lib.h"

// PS2 Joystick structure definition
typedef struct
{
  float angle;
  float strength;
} PS2JoystickData;

typedef struct
{
  bool holding;
  bool pressed;
  bool released;
} PS2ButtonState;

enum class PS2Button: uint16_t
{
  SELECT = PSB_SELECT,
  L3 = PSB_L3,  // left joystick button
  R3 = PSB_R3,  // right joystick button
  START = PSB_START,
  UP    = PSB_PAD_UP,
  RIGHT = PSB_PAD_RIGHT,
  DOWN  = PSB_PAD_DOWN,
  LEFT  = PSB_PAD_LEFT,
  L2 = PSB_L2,
  R2 = PSB_R2,
  L1 = PSB_L1,
  R1 = PSB_R1,
  TRIANGLE = PSB_TRIANGLE,
  CIRCLE   = PSB_CIRCLE,
  CROSS    = PSB_CROSS,
  SQUARE   = PSB_SQUARE,
};

enum class PS2Joystick: uint8_t
{
  LEFT = 0,  // Left joystick
  RIGHT = 1, // Right joystick
};

#endif  // PS2_H