#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
  // configure motor ports to match the physical wiring of the robot
  // robot.motorConfiguration(M3, M1, M2, M4);

  // Flip the coordinate system of the compound eye module to match the robot's orientation
  robot.coordinateFlip(robot.compoundEye); // Flip the coordinate system of the compound eye module

  // movement configuration: enable compass correction and disable out-of-bounds prevention
  robot.compassCorrectEnabled = true;
  robot.outBoundPreventEnabled = false;
}
void loop() {
  uint16_t ballAngle = robot.compoundEyeAngleRead(); // Read the angle of the detected object
  float moveAngle = ballAngle;

  if (ballAngle >= 350 || ballAngle <= 10) {
    robot.move(0, 160); // Move forward
  } else {
    float angleDrift = 0.0f;
    angleDrift = atan2(60, 180-robot.compoundMaxEyeValueRead()) * 180.0f / 3.14159f; // Calculate angle drift based on the maximum IR sensor value
    if (ballAngle < 180) {
      moveAngle = ballAngle + angleDrift; // Adjust movement angle to the right
    } else {
      moveAngle = ballAngle - angleDrift; // Adjust movement angle to the left
    }
    Serial.print(robot.compoundMaxEyeValueRead());
    Serial.print(", Move Angle: ");
    Serial.println(moveAngle);
    robot.move(moveAngle, 160); // Move towards the ball
  }
}