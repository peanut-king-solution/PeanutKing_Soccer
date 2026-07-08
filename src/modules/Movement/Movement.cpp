#include "Movement.h"

// Constructor
Movement::Movement(Motor &m) : 
  motor(m), // initialize the motor reference
  converter(), // initialize the converter instance
  motorPID(2.0, 0.0, 0.0) // initialize the PID controller with default coefficients
{

}

// robot movement based on angle, speed, and rotation
void Movement::byAngle(float mAngle, float mSpeed, float rotate)
{
  float mc[4];

  // convert the angle to the robot's coordinate system
  mAngle = converter.convert(mAngle);

  // vector decomposition for mecanum wheels
  mc[0] =  mSpeed * sin((mAngle + 45.0) * pi / 180.0);  // LF
  mc[1] = -mSpeed * cos((mAngle + 45.0) * pi / 180.0);  // RF
  mc[2] = -mc[0];                                       // RB
  mc[3] = -mc[1];                                       // LB

  // Apply the speed and rotation to each motor
  for (int8_t i = 3; i >= 0; i--)
  {
    motor.SetSpeed((MOTOR_ID)i, (int16_t)(mc[i] + rotate));
  }
}

// robot movement based on X and Y speed components
void Movement::bySpeedVector(int16_t speed_X, int16_t speed_Y)
{
  // Calculate the angle of movement based on the speed vector
  float mAngle = atan2(speed_Y, speed_X) * 180.0 / PI;
  if (mAngle < 0) mAngle += 360.0;

  // Calculate the magnitude of the speed vector
  uint16_t mSpeed = sqrt(speed_X * speed_X + speed_Y * speed_Y);

  Smart(mAngle, mSpeed);
}

// robot movement based on angle, speed, and compass correction with PID control
void Movement::byAngleWithSmart(float mAngle, float mSpeed, float compassReading)
{
  // read the current compass value and normalize it to be [-180, 180] degrees
  float c = converter.normalize(compassReading + 180.0f) - 180.0f;

  // calculate the rotation correction based on the current compass reading
  float rotation = this->motorPID.update(c);

  byAngle(mAngle, mSpeed, rotation);
}

void Movement::byAngleWithJason(float mAngle, float mSpeed, float compassReading)
{
  // convert the angle to the robot's coordinate system
  mAngle = converter.convert(mAngle);

  float rad = (mAngle + 45.0f) * (pi / 180.0f); // convert angle to radians and adjust for mecanum wheel orientation
  float a = cos(rad), b = sin(rad);             // calculate the X and Y components of the movement vector
  float scaleFactor = max(fabsf(a), fabsf(b));  // help normalize the motor speeds to -1 to 1

  float m[4]; // motor speeds for each wheel
  m[0] =  b / scaleFactor; // LF (without -ve, follow the motor position)
  m[1] = -a / scaleFactor; // RF
  m[2] = -m[0];            // RB
  m[3] = -m[1];            // LB

  // normalize the compass reading to -180 to 180
  compassReading = converter.normalize(compassReading + 180.0f) - 180.0f;

  float rotationScale, rotation;
  rotationScale = -(compassReading / 180.0f); // scale to -1 to 1

  // if the compass reading is within the range of -90 to 90 degrees,
  // apply a rotation correction based on the PID controller output
  if (fabsf(compassReading) < 90.0f) {
    rotation = this->motorPID.update(-rotationScale);
    rotation = constrain(rotation, -255.0f, 255.0f);
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
    m[i] = m[i] * factor + rotation;          // combine the movement and rotation components
    m[i] = constrain(m[i], -255.0f, 255.0f);  // ensure the motor speeds are within the valid range
    motor.SetSpeed((MOTOR_ID)i, (int16_t)m[i]);     // set the speed for each motor
  }
}

// motor move + compass as reference
void Movement::Smart(uint16_t angular_direction, int16_t speed, int16_t angle, uint8_t precision)
{
  // // calculate the difference between the current compass reading and the desired angle
  // int16_t c = compassRef.read() - angle;
  // // normalize the difference to be within -180 to 180 degrees
  // int16_t rotation = c < 180 ? -c : 360 - c; // need to fix?

  // // speed - 50
  // // rotation = abs(speed) < 120 ? rotation : rotation * 1.5;
  // rotation = rotation * (precision + 3) / 12;
  // // if ( speed==0 && abs(rotation)>10 ) rotation = rotation < 35 ? 35 : rotation;
  // if (speed == 0 && abs(rotation) < 12) rotation = 0;

  // // call moveByAngle with the calculated rotation
  // moveByAngle(angular_direction, speed, rotation);
}