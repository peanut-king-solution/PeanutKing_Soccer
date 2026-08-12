#include "Motor.h"

Motor::Motor() :
  _in1Pin{9, 7, 5, 3},       // initialize all motors' ch1 pins
  _in2Pin{8, 6, 4, 2},       // initialize all motors' ch2 pins
  _motorMap{M1, M2, M3, M4}, // default motor mapping (no mapping applied)
  _motorflip{false, false, false, false} // default motor flip state (no flipping)
{
}

void Motor::init(void) {
  // Initialize motor pins as OUTPUT
  for (uint8_t i=0; i<4; i++) {
    pinMode(_in1Pin[i], OUTPUT);
    pinMode(_in2Pin[i], OUTPUT);
  }
}

bool Motor::portValidCheck(MotorId mi) {
  return (mi >= M1 && mi <= M4);
}

MotorId Motor::getPortFromPos(MotorPos pos)
{
  if (pos < LeftFront || pos > LeftBack) { return MotorMaxCount; } // Invalid position
  // Return the corresponding motor port based on the current mapping
  return _motorMap[pos];
}

void Motor::mapPort(MotorId LeftFront, MotorId RightFront, MotorId RightBack, MotorId LeftBack)
{
  // Check if all motor ports are valid
  if (!portValidCheck(LeftFront) || !portValidCheck(RightFront) || !portValidCheck(RightBack) || !portValidCheck(LeftBack)) {
    return; // Invalid motor port, do nothing
  }
  // Remap motor ports to physical positions
  _motorMap[0] = LeftFront;
  _motorMap[1] = RightFront;
  _motorMap[2] = RightBack;
  _motorMap[3] = LeftBack;
}

void Motor::flipDirection(MotorId mi, bool flip)
{
  // invalid motor port, do nothing
  if (!portValidCheck(mi)) { return; }

  // Set the flip state of the specified motor
  _motorflip[mi] = flip;
}

void Motor::setSpeed(MotorId mi, int16_t speed)
{
  // invalid motor port, do nothing
  if (!portValidCheck(mi)) { return; }

  // apply motor flip if needed
  if (_motorflip[mi]) { speed = -speed; }

  // constrain speed to be within -255 to 255
  speed = constrain(speed, -255, 255);

  // H bridge control logic, directly on port `mi`
  if (speed == 0) {
    digitalWrite(_in1Pin[mi], HIGH);
    digitalWrite(_in2Pin[mi], HIGH);
  }
  else if (speed > 0) {
    analogWrite(_in1Pin[mi], speed);
    digitalWrite(_in2Pin[mi], LOW);
  }
  else {
    digitalWrite(_in1Pin[mi], LOW);
    analogWrite(_in2Pin[mi], -speed);
  }
}

void Motor::stop(MotorId mi)
{
  // invalid motor port, do nothing
  if (!portValidCheck(mi)) { return; }
  // Brake a single motor directly on port `mi`
  digitalWrite(_in1Pin[mi], HIGH);
  digitalWrite(_in2Pin[mi], HIGH);
}