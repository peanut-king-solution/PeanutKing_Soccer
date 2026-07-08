#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include "PeanutKingDef.h"

// #define  MOTOR     0xf0    // 4 motors * 2byte
// #define  ENCODER   0xf8    // 4 encoder* byte

/*  ================== Motor Mapping ==================
 *  All motors +ve speed -> rotate clockwise
 *  Default Motor Mapping | index in in1Pin/in2Pin arrays
 *  Left Front  -> M1     |              0
 *  Right Front -> M2     |              1
 *  Right Back  -> M3     |              2
 *  Left Back   -> M4     |              3
 *  =================================================== */

typedef enum
{
  M1 = 0, // Left Front
  M2 = 1, // Right Front
  M3 = 2, // Right Back
  M4 = 3, // Left Back
} MOTOR_ID;

class Motor
{
private:
  const uint8_t in1Pin[4]; // motors' ch1 pins
  const uint8_t in2Pin[4]; // motors' ch2 pins

  // Motor mapping array to determine which motor is connected to which port (M1/M2/M3/M4)
  // Order: Left Front, Right Front, Right Back, Left Back
  // Default mapping: M1, M2, M3, M4 (no mapping applied)
  MOTOR_ID motorMap[4];

public:
  // Constructor
  Motor();

  // init motor pins
  void init(void);
  /* Check which motor is connected to which port (M1/M2/M3/M4),
   * then allocate the motor port to the correct motor position in void setup() function.
   * e.g. motor.mapMotors(M3, M1, M4, M2); */
  void mapMotors(MOTOR_ID LeftFront, MOTOR_ID RightFront, MOTOR_ID RightBack, MOTOR_ID LeftBack);

  /* Set single motor speed, mi: motor index (0-3), speed: -255 to 255
   *
   * All speed are < 0 -> robot rotates clockwise */
  void SetSpeed(MOTOR_ID mi, int16_t speed);
  // stop all motors
  void StopAll(void);
  void motorTest(void);
};

#endif // MOTOR_H