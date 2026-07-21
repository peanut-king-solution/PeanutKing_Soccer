#include "Movement.h"

Movement::Movement(Motor &m) :
  motor(m),                 // initialize the motor reference
  converter(),              // initialize the converter instance
  motorPID(300.0, 1.0, 2.0) // initialize the PID controller
{
}

void Movement::byAngle(float mAngle, float mSpeed, float rotate)
{
  // constrain speed to valid range (0-255)
  mSpeed = constrain(mSpeed, 0.0f, 255.0f);

  // convert the angle to the robot's coordinate system
  mAngle = converter.convert(mAngle);

  // vector decomposition for mecanum wheels
  float mc[4];  // motor speeds for each wheel
  mc[0] =  mSpeed * sin((mAngle + 45.0) * pi / 180.0);  // LF
  mc[1] = -mSpeed * cos((mAngle + 45.0) * pi / 180.0);  // RF
  mc[2] = -mc[0];                                       // RB
  mc[3] = -mc[1];                                       // LB

  // Apply the speed and rotation to each motor
  for (int8_t i = 3; i >= 0; i--)
  {
    motor.setSpeed((MOTOR_ID)i, (int16_t)(mc[i] + rotate));
  }
}

void Movement::byAnglePID(float mAngle, float mSpeed, float compassReading)
{
  // constrain speed to valid range (0-255)
  mSpeed = constrain(mSpeed, 0.0f, 255.0f);

  // convert the angle to the robot's coordinate system
  mAngle = converter.convert(mAngle);

  // vector decomposition for mecanum wheels
  float rad = (mAngle + 45.0f) * (pi / 180.0f);
  float a = cos(rad), b = sin(rad);
  float scaleFactor = max(fabsf(a), fabsf(b));

  float m[4]; // motor speeds for each wheel
  m[0] =  b / scaleFactor; // LF
  m[1] = -a / scaleFactor; // RF
  m[2] = -m[0];            // RB
  m[3] = -m[1];            // LB

  // normalize the compass reading to -180 to 180
  compassReading = converter.normalize(compassReading + 180.0f) - 180.0f;

  float rotationScale, rotation;
  rotationScale = -(compassReading / 180.0f); // scale the rotation to [-1, 1]

  // Apply a dead zone for small rotation errors to prevent oscillation
  if (fabsf(rotationScale) < 0.05f) {
    rotation = 0.0f;  // no rotation needed
  }
  // if the compass reading is within the range of -90 to 90 degrees,
  // apply a rotation correction based on the PID controller output
  else if (fabsf(compassReading) < 90.0f) {
    rotation = this->motorPID.update(-rotationScale);
    rotation = constrain(rotation, -255.0f, 255.0f);
    if (fabsf(rotation) < 60.0f) {
      rotation = 60.0f * ((rotation > 0.0f) ? 1.0f : -1.0f);
    }
  }
  // if the compass reading is outside the range of -90 to 90 degrees,
  // apply a maximum rotation speed in the appropriate direction
  // and set the movement speed to zero
  else {
    float sign = (rotationScale > 0.0f) ? 1.0f : ((rotationScale < 0.0f) ? -1.0f : 0.0f);
    rotation = 255.0f * sign;
    mSpeed = 0.0f;
  }

  // scale the motor speeds based on the desired speed and rotation
  float factor = (mSpeed / 255.0f) * (255.0f - fabsf(rotation));
  
  for (uint8_t i = 0; i < 4; i++)
  {
    // apply the scaling factor and rotation to each motor speed
    m[i] = m[i] * factor + rotation;
    // constrain the motor speed to be within the valid range of -255 to 255
    m[i] = constrain(m[i], -255.0f, 255.0f);
    // set the motor speed for each motor
    motor.setSpeed((MOTOR_ID)i, (int16_t)m[i]);
  }
}

void Movement::test(float speed)
{
  this->byAngle(0, speed, 0);
  delay(1000);
  this->byAngle(45, speed, 0);
  delay(1000);
  this->byAngle(90, speed, 0);
  delay(1000);

  this->motor.stopAll();
  delay(500);
}