#include "Motor.h"

// Constructor
Motor::Motor() :
  in1Pin{9, 7, 5, 3},      // initialize all motors' ch1 pins
  in2Pin{8, 6, 4, 2},      // initialize all motors' ch2 pins
  motorMap{M1, M2, M3, M4} // default motor mapping (no mapping applied)
{
}

// init motor pins
void Motor::init(void) {
  for (uint8_t i=0; i<4; i++) {
    pinMode(in1Pin[i], OUTPUT);
    pinMode(in2Pin[i], OUTPUT);
  }
}

/* Check which motor is connected to which port (M1/M2/M3/M4),
 * then allocate the motor port to the correct motor position in void setup() function.
 * e.g. robot.mapMotors(M3, M1, M4, M2); */
void Motor::mapMotors(MOTOR_ID LeftFront, MOTOR_ID RightFront, MOTOR_ID RightBack, MOTOR_ID LeftBack)
{
  // update the motor mapping in the motorMap array
  motorMap[0] = LeftFront;
  motorMap[1] = RightFront;
  motorMap[2] = RightBack;
  motorMap[3] = LeftBack;
}

/* Set single motor speed
 * mi: motor index (0-3)
 * speed: -255 to 255
 *
 * All speed are < 0 -> robot rotates anti-clockwise */
void Motor::SetSpeed(MOTOR_ID mi, int16_t speed)
{
  MOTOR_ID motorIndex = motorMap[mi];  // use motorMap to get the actual motor index
  speed = constrain(speed, -255, 255); // constrain speed to be within -255 to 255

  // H brigdge control logic
  if (speed == 0) {
    digitalWrite(in1Pin[motorIndex], HIGH);
    digitalWrite(in2Pin[motorIndex], HIGH);
  }
  else if (speed > 0) {
    analogWrite(in1Pin[motorIndex], speed);
    digitalWrite(in2Pin[motorIndex], LOW);
  }
  else {
    digitalWrite(in1Pin[motorIndex], LOW);
    analogWrite(in2Pin[motorIndex], -speed);
  }
}

// stop all motors
void Motor::StopAll(void)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    digitalWrite(in1Pin[i], HIGH);
    digitalWrite(in2Pin[i], HIGH);
  }
}

// motor test ------------------------------------------------------
void Motor::motorTest(void)
{
  // static uint32_t motorTimer = 0;
  // static uint8_t i = 0;
  // uint32_t timeNow = millis();

  // if (timeNow - motorTimer > 1000)
  // {
  //   motorTimer = timeNow;
  //   for (uint8_t j = 0; j < 4; j++)
  //   {
  //     if (i < 4)
  //       SetSpeed((MOTOR_ID)j, i == j ? 100 : 0);
  //     else
  //       SetSpeed((MOTOR_ID)j, (i - 4) == j ? -100 : 0);
  //   }
  //   i++;
  //   if (i == 9)
  //     i = 0;
  // }
}