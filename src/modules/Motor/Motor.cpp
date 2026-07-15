#include "Motor.h"

Motor::Motor() :
  in1Pin{9, 7, 5, 3},       // initialize all motors' ch1 pins
  in2Pin{8, 6, 4, 2},       // initialize all motors' ch2 pins
  motorMap{M1, M2, M3, M4}, // default motor mapping (no mapping applied)
  motorflip{false, false, false, false} // default motor flip state (no flipping)
{
}

void Motor::init(void) {
  // Initialize motor pins as OUTPUT
  for (uint8_t i=0; i<4; i++) {
    pinMode(in1Pin[i], OUTPUT);
    pinMode(in2Pin[i], OUTPUT);
  }
}

void Motor::configuration(MOTOR_ID LeftFront, MOTOR_ID RightFront, MOTOR_ID RightBack, MOTOR_ID LeftBack)
{
  // Remap motor ports to physical positions
  motorMap[0] = LeftFront;
  motorMap[1] = RightFront;
  motorMap[2] = RightBack;
  motorMap[3] = LeftBack;
}

void Motor::flipMotor(MOTOR_ID mi, bool flip)
{
  // Set the flip state of the specified motor
  motorflip[mi] = flip;
}

void Motor::flipMotors(bool m1, bool m2, bool m3, bool m4)
{
  // Set flip state for all motors
  motorflip[0] = m1;
  motorflip[1] = m2;
  motorflip[2] = m3;
  motorflip[3] = m4;
}

void Motor::setSpeed(MOTOR_ID mi, int16_t speed)
{
  // use motorMap to get the actual motor index
  MOTOR_ID motorIndex = motorMap[mi];

  // apply motor flip if needed
  if (motorflip[mi]) { speed = -speed; }

  // constrain speed to be within -255 to 255
  speed = constrain(speed, -255, 255);

  // H bridge control logic
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

void Motor::stopAll(void)
{
  // Stop all motors (brake mode)
  for (uint8_t i = 0; i < 4; i++)
  {
    digitalWrite(in1Pin[i], HIGH);
    digitalWrite(in2Pin[i], HIGH);
  }
}

void Motor::testAll(int16_t speed)
{
  // Test motors sequentially (M1->M2->M3->M4)
  for (uint8_t i = 0; i < 4; i++) {
    setSpeed((MOTOR_ID)i, speed);
    delay(1000);
    stopAll();
    delay(500);
  }
}