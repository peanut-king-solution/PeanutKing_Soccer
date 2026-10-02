#include "hwI2CMaster.h"

hwI2CMaster::hwI2CMaster() {}

// =============================================================================
//                              Wire.h Backend
// =============================================================================

#ifdef HW_I2C_USE_WIRE_H

bool hwI2CMaster::init(uint32_t speed) {
  if (speed < 100000) speed = 100000; // Minimum speed is 100kHz
  if (speed > 400000) speed = 400000; // Maximum speed is 400kHz
  defaultSpeed = speed;

  Wire.begin();
  // Wire.setClock(speed);
  Wire.setClock(100000); // Set to 100kHz first (compass has issues with 400kHz)

  return true;
}

bool hwI2CMaster::read(uint8_t deviceAddress, uint8_t *rxBuffer, uint8_t length) {
  uint8_t received = Wire.requestFrom(deviceAddress, length);
  if (received < length) return false;
  for (uint8_t i = 0; i < length; i++) {
    rxBuffer[i] = Wire.read();
  }
  return true;
}

bool hwI2CMaster::write(uint8_t deviceAddress, const uint8_t *txBuffer, uint8_t length) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(txBuffer, length);
  return (Wire.endTransmission(true) == 0);
}

bool hwI2CMaster::readReg(uint8_t deviceAddress, uint8_t reg, uint8_t *rxBuffer, uint8_t length) {
  // First transaction: write register address (with full STOP)
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) {  // true = send full STOP
    return false;
  }

  // Second transaction: read data from device
  uint8_t received = Wire.requestFrom(deviceAddress, length);
  if (received < length) return false;
  for (uint8_t i = 0; i < length; i++) {
    rxBuffer[i] = Wire.read();
  }
  return true;
}

bool hwI2CMaster::writeReg(uint8_t deviceAddress, uint8_t reg, const uint8_t *txBuffer, uint8_t length) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(reg);
  Wire.write(txBuffer, length);
  return (Wire.endTransmission(true) == 0);
}

// =============================================================================
//                              IICIT Backend
// =============================================================================

#else

bool hwI2CMaster::init(uint32_t speed) {
  defaultSpeed = speed;
  gIIC->SetSpeed(speedToIICIT(speed));
  return true;
}

bool hwI2CMaster::read(uint8_t deviceAddress, uint8_t *rxBuffer, uint8_t length) {
  // Register the temp device with IICIT
  IICIT::Handle handle = gIIC->RegisterDevice(deviceAddress, 1, speedToIICIT(defaultSpeed));

  if (handle.device_address == 0) {
    return false; // failed to register device
  }

  IICIT::status_t status = gIIC->Read(handle, rxBuffer, length);
  return (status == IICIT::STATUS_OK);
}

bool hwI2CMaster::write(uint8_t deviceAddress, const uint8_t *txBuffer, uint8_t length) {
  // Register the temp device with IICIT
  IICIT::Handle handle = gIIC->RegisterDevice(deviceAddress, 1, speedToIICIT(defaultSpeed));

  if (handle.device_address == 0) {
    return false; // failed to register device
  }

  IICIT::status_t status = gIIC->Write(handle, txBuffer, length);
  return (status == IICIT::STATUS_OK);
}

bool hwI2CMaster::readReg(uint8_t deviceAddress, uint8_t reg, uint8_t *rxBuffer, uint8_t length) {
  // First, write the register address to the device
  if (!write(deviceAddress, &reg, 1)) return false;
  // Then, read the data from the device
  return read(deviceAddress, rxBuffer, length);
}

bool hwI2CMaster::writeReg(uint8_t deviceAddress, uint8_t reg, const uint8_t *txBuffer, uint8_t length) {
  // Create a temporary buffer to hold the register address and data
  uint8_t tempBuffer[I2C_MAX_BUFFER_SIZE];

  // Check if the total length exceeds the maximum buffer size
  if ((length + 1) > I2C_MAX_BUFFER_SIZE) return false;

  // Copy the register address and data into the temporary buffer
  tempBuffer[0] = reg;
  memcpy(tempBuffer + 1, txBuffer, length);

  // Write the combined buffer to the device
  return write(deviceAddress, tempBuffer, length + 1);
}

IICIT::Speed hwI2CMaster::speedToIICIT(uint32_t speed) const {
  return (speed >= 400000) ? IICIT::Speed::FAST : IICIT::Speed::SLOW;
}

IICIT::status_t hwI2CMaster::rxCpltCallback(const IICIT::status_t status) {
  return status;
}

#endif