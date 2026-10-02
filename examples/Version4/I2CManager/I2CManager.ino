/**
 * It is a simple example demonstrating how to use the I2CManager
 * This example shows how to register an I2C device, read from it, and handle errors.
 * 
 * This example try to read the compass sensor of PeanutKingSoccerV4 robot
 * by software i2c without using the PeanutKingSoccerV4
 */

#include <PeanutKingSoccerV4.h>

I2CManager& i2c = I2CManager::getInstance();

BusIndex bus = BusIndex::SW7;   // Software I2C bus index to use for the compass sensor
uint8_t deviceAddress = 0x08;   // I2C slave 7-bit address of the compass sensor
uint32_t speed = 0;             // 0 as software I2C not related to speed set
I2C_Handle compass = I2C_Handle(bus, deviceAddress, speed); // Create an I2C handle for the compass sensor

void setup() {
  /*
  i2c.init(); is call in robot.init();
  if you have called robot.init(); already, 
  you don't need to call i2c.init() again.
  and SW0-SW3 will be used by default for color sensors, 
  so you may need to change the bus to SW4-SW7 
  if you want to use those software I2C for other devices.
  */
  // i2c.init();

  Serial.begin(115200);

  // Check if the registration was successful
  if (compass.isValid()) {
    Serial.print("Successfully registered device at address 0x");
    Serial.println(compass.deviceAddress, HEX);
  } else {
    Serial.print("Failed to register device at address 0x");
    Serial.print(deviceAddress, HEX);
    Serial.print(" on ");
    if (bus >= BusIndex::SW0 && bus <= BusIndex::SW7) {
      Serial.print("SWIIC bus ");
      Serial.println((int)bus, DEC);
    } else if (bus == BusIndex::HW) {
      Serial.println("HW I2C bus");
    } else {
      Serial.println("Invalid bus index");
    }
  }
}

void loop() {
  // Example: Read from a register (e.g., 0x56) of the compass
  uint8_t reg = 0x56;   // Example register address
  uint8_t rxBuffer[2]; // Buffer to hold received data
  uint16_t heading = 65535; // Default value if read fails

  int readCount = 0;
  uint32_t startTime = micros(); // Start timing the read operation
  while (micros() - startTime < 1000000) { // Timeout after 1 second
    // Attempt to read from the compass sensor (true if successful, false otherwise)
    if (i2c.SensorRead(compass, reg, rxBuffer, sizeof(rxBuffer))) {
      // Combine the two bytes into a 16-bit heading value
      heading = (uint16_t)(rxBuffer[0] | (rxBuffer[1] << 8));
      heading = heading / 100; // need to divide by 100 to get the heading in degrees

      // Print the heading to the Serial Monitor
      Serial.print("Compass Heading: ");
      Serial.println(heading);
      
      // Update the read count and break the loop if successful
      readCount++;
    } else {
      Serial.println("Failed to read from device. Retrying...");
      delay(100); // Wait before retrying
    } 
  }
  Serial.print("Read attempts: ");
  Serial.println(readCount);

  delay(1000); // Wait for 1 second before the next read
}