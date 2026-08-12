/**
 * This example demonstrates how to read the compass heading
 * and reading raw sensor data (accelerometer, gyroscope, magnetometer) 
 * from the PeanutKingSoccerV4 robot.
 */

#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Read compass heading
  uint16_t heading = robot.compassRead();

  // or use the public variable directly
  // robot.dataFetch(); // Update all sensor data
  // uint16_t heading = robot.heading;

  // Print compass heading
  Serial.print("Heading: ");
  Serial.print(heading);

  // Read and print raw accelerometer data
  int16_t* accelData = robot.compassReadRawAccel();
  Serial.print(", Accel: X= ");
  Serial.print(accelData[0]);
  Serial.print(", Y= ");
  Serial.print(accelData[1]);
  Serial.print(", Z= ");
  Serial.print(accelData[2]);
  
  // Read and print raw gyroscope data
  int16_t* gyroData = robot.compassReadRawGyro();
  Serial.print(", Gyro: X= ");
  Serial.print(gyroData[0]);
  Serial.print(", Y= ");
  Serial.print(gyroData[1]);
  Serial.print(", Z= ");
  Serial.println(gyroData[2]);

  // Read and print raw magnetometer data
  int16_t* magData = robot.compassReadRawMag();
  Serial.print(", Magnet: X= ");
  Serial.print(magData[0]);
  Serial.print(", Y= ");
  Serial.print(magData[1]);
  Serial.print(", Z= ");
  Serial.println(magData[2]);

  delay(100);
}