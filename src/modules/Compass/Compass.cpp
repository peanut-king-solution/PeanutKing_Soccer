#include "Compass.h"

Compass::Compass()
  : _handle(BusIndex::HW, COMPASS_I2C_ADDRESS, 400000)  // I2C speed: 400kHz
{
}

bool Compass::init(void) {
  // Return false if the handle is invalid
  if (!_handle.isValid()) { return false; }

  // Compass Reset Heading
  // delay(10); // wait for compass calibration
  // // Read multiple samples to calculate the average compass reading
  // uint16_t sum = 0; int8_t sampleCount = 10;
  // for (int i = 0; i < sampleCount; i++) {
  //   sum += this->read();
  //   delay(50); // wait for the next sample
  // }

  // Set 0° as the direction of the robot facing at starting
  // converter.reset().shift(-(int16_t)(sum / sampleCount));

  return true; // Return true if initialization is successful
}

uint16_t Compass::readHeading() {
  // Clear the receive buffer before reading
  // clearBuffer();

  // Read the compass value from the compass module using the I2C manager
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, GET_YAW, rxBuff, 2);
  compass = (uint16_t)rxBuff[0] | ((uint16_t)rxBuff[1] << 8);
  compass = compass / 100;

  // Apply the conversion using the compassConverter
  // compass = converter.convert(compass);
  return compass;
}

int16_t* Compass::readRaw6(uint8_t reg, int16_t* dataArr) {
  // Clear the receive buffer before reading
  // clearBuffer();

  // Read 6 bytes of raw sensor data from the specified register
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, reg, rxBuff, 6);

  // Combine the received bytes into 16-bit signed integers
  dataArr[0] = (int16_t)((uint16_t)rxBuff[0] | ((uint16_t)rxBuff[1] << 8));
  dataArr[1] = (int16_t)((uint16_t)rxBuff[2] | ((uint16_t)rxBuff[3] << 8));
  dataArr[2] = (int16_t)((uint16_t)rxBuff[4] | ((uint16_t)rxBuff[5] << 8));
  return dataArr;
}

int16_t* Compass::readAccelerometerRaw(void) {
  return readRaw6(ACC_RAW, accelData);
}

int16_t* Compass::readGyroscopeRaw(void) {
  return readRaw6(GYR_RAW, gyroData);
}

int16_t* Compass::readMagnetometerRaw(void) {
  return readRaw6(MAG_RAW, magData);
}

void Compass::clearBuffer(void) {
  // Clear the receive buffer by setting all bytes to zero
  for (uint8_t i = 0; i < COMPASS_RX_BUFFER_SIZE; i++) {
    rxBuff[i] = 0;
  }
}