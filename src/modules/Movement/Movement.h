#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "PeanutKingDef.h"
#include "utils/Converter.h"
#include "utils/PIDController.h"
#include "modules/Motor/Motor.h"

class Movement
{
private:
  // Motor instance for controlling the robot's movement
  Motor &motor;

public:
  /**
   * Constructor with Motor reference injection
   * `m` - Motor instance reference
   */
  Movement(Motor &m);

  /**
   * Move robot at angle with speed and rotation
   * `mAngle`  - Movement angle `(0-360°)`
   * `mSpeed`  - Movement speed `(0-255)`
   * `rotate`  - Rotation speed `(-255 to +255)`, positive=`CW`, negative=`CCW`
   */
  void byAngle(float mAngle, float mSpeed, float rotate);

  /**
   * Move robot with compass correction
   * `mAngle`          - Movement angle `(0-360°)`
   * `mSpeed`          - Movement speed `(0-255)`
   * `compassReading`  - Current compass heading `(0-360°)`
   */
  void withCorr(float mAngle, float mSpeed, float compassReading);

  /**
   * Test movement patterns ( `forward` -> `right front` -> `rightward` )
   * `speed` - Test speed `(0-255)`
   */
  void test(float speed);

  // Converters' instances
  Converter converter;
  // PID controllers' instances
  PIDController motorPID;
};

#endif // MOVEMENT_H