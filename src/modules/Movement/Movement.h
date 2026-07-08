#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "utils/Converter.h"
#include "utils/PIDController.h"
#include "modules/Motor/Motor.h"

class Movement
{
private:
  // Motor instance for controlling the robot's movement
  Motor &motor;

public:
  // Constructor
  Movement(Motor &m);
  
  // robot movement based on angle, speed, and rotation
  void byAngle(float mAngle, float mSpeed, float rotate);
  // robot movement based on X and Y speed components
  void bySpeedVector(int16_t speed_X, int16_t speed_Y);
  // robot movement based on angle, speed, and compass correction with PID control
  void byAngleWithSmart(float mAngle, float mSpeed, float compassReading);
  void byAngleWithJason(float mAngle, float mSpeed, float compassReading);
  // motor move + compass as reference
  void Smart(uint16_t angular_direction, int16_t speed, int16_t angle = 0, uint8_t precision = 5);

  // Converters' instances
  Converter converter;
  // PID controllers' instances
  PIDController motorPID;
};

#endif // MOVEMENT_H