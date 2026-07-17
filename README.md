# PeanutKing Soccer

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-00979D)](https://www.arduino.cc/reference/en/libraries/)
[![Version](https://img.shields.io/badge/version-4.2.0-blue)](https://github.com/peanut-king-solution/PeanutKing_Soccer)

Arduino library for controlling **PeanutKing Soccer Robots** (V2 / V3 / V4 compatible).

---

## Table of Contents

- [PeanutKing Soccer](#peanutking-soccer)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
    - [Supported Versions](#supported-versions)
  - [Installation](#installation)
    - [Via Arduino IDE](#via-arduino-ide)
    - [Manual Installation](#manual-installation)
    - [Dependencies](#dependencies)
  - [Quick Start](#quick-start)
  - [Module Overview](#module-overview)
  - [Module Documentation](#module-documentation)
    - [Motor — Motor Control](#motor--motor-control)
      - [Methods](#methods)
      - [Example](#example)
      - [Compatibility Wrappers](#compatibility-wrappers)
      - [Related Examples](#related-examples)
    - [Movement — Omnidirectional Movement](#movement--omnidirectional-movement)
      - [Methods](#methods-1)
      - [Subclasses](#subclasses)
      - [Example](#example-1)
      - [Compatibility Wrappers](#compatibility-wrappers-1)
      - [Related Examples](#related-examples-1)
    - [ColorSensor — Color Sensor](#colorsensor--color-sensor)
      - [Data Structures](#data-structures)
      - [Methods](#methods-2)
      - [Example](#example-2)
      - [Compatibility Wrappers](#compatibility-wrappers-2)
      - [Related Examples](#related-examples-2)
    - [CompoundEye — IR Compound Eye](#compoundeye--ir-compound-eye)
      - [Register Map](#register-map)
      - [Methods](#methods-3)
      - [Example](#example-3)
      - [Compatibility Wrappers](#compatibility-wrappers-3)
      - [Related Examples](#related-examples-3)
    - [ButtonManager — Button Management](#buttonmanager--button-management)
      - [Methods](#methods-4)
      - [Example](#example-4)
      - [Compatibility Wrappers](#compatibility-wrappers-4)
      - [Related Examples](#related-examples-4)
    - [LedController — LED Control](#ledcontroller--led-control)
      - [Methods](#methods-5)
      - [Example](#example-5)
      - [Compatibility Wrappers](#compatibility-wrappers-5)
      - [Related Examples](#related-examples-5)
    - [Ultrasonic — Ultrasonic Sensor](#ultrasonic--ultrasonic-sensor)
      - [Methods](#methods-6)
      - [Example](#example-6)
      - [Compatibility Wrappers](#compatibility-wrappers-6)
      - [Related Examples](#related-examples-6)
    - [Compass — Compass & IMU](#compass--compass--imu)
      - [Methods](#methods-7)
      - [Subclasses](#subclasses-1)
      - [Example](#example-7)
      - [Compatibility Wrappers](#compatibility-wrappers-7)
      - [Related Examples](#related-examples-7)
    - [I2C — I2C Bus Management](#i2c--i2c-bus-management)
      - [Methods](#methods-8)
      - [Example](#example-8)
      - [Related Tests](#related-tests)
    - [TFT Display](#tft-display)
      - [Methods](#methods-9)
      - [Color Constants](#color-constants)
      - [Direct Drawing](#direct-drawing)
      - [Example](#example-9)
      - [Related Examples](#related-examples-8)
    - [Utility Classes](#utility-classes)
      - [Converter](#converter)
      - [PIDController](#pidcontroller)
  - [Examples](#examples)
  - [Hardware Configuration](#hardware-configuration)
    - [Default Pin Mapping](#default-pin-mapping)
    - [I2C Device Addresses](#i2c-device-addresses)

---

## Introduction

**PeanutKing Soccer** is a complete Arduino library supporting sensor reading, motor control, omnidirectional movement, and wireless remote control for PeanutKing series soccer robots.

### Supported Versions

- V2 (legacy compatibility)
- V3 (legacy compatibility)
- **V4** (main development version)

---

## Installation

### Via Arduino IDE

1. Download the ZIP archive of this library
2. In Arduino IDE, select **Sketch → Include Library → Add .ZIP Library...**
3. Choose the downloaded ZIP file to complete installation

### Manual Installation

Copy the entire `PeanutKing_Soccer` folder to the Arduino `libraries` directory:

| Platform | Path |
|----------|------|
| Windows | `Documents\Arduino\libraries\` |
| macOS | `~/Documents/Arduino/libraries/` |
| Linux | `~/Arduino/libraries/` |

### Dependencies

This library includes the following dependencies (built-in, no additional installation required):

- `PDQ_GFX` / `PDQ_ST7735` — TFT display driver
- `SlowSoftI2CMaster` — Software I2C implementation
- `pcint` — Pin Change Interrupt handler

---

## Quick Start

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();          // Initialize all modules
}

void loop() {
  // Read compass heading
  uint16_t heading = robot.compass.read();
  Serial.print("Heading: ");
  Serial.println(heading);

  // Read ultrasonic distance
  uint16_t dist = robot.ultrasonicRead(U1);
  Serial.print("Front distance: ");
  Serial.println(dist);

  // Set LED color
  robot.setOnBrdLED(LED_CYAN);

  delay(100);
}
```

---

## Module Overview

| Module | Instance | Description |
|--------|----------|-------------|
| Motor | `robot.motor` | 4 DC motor control |
| Movement | `robot.move` | 45° omni wheel omnidirectional movement |
| ColorSensor | `robot.colorSensor` | Up to 8 color sensors (I2C) |
| CompoundEye | `robot.compoundEye` | 12-channel IR sensor array |
| ButtonManager | `robot.buttonMgr` | 4-button state management |
| LedController | `robot.ledCtrl` | On-board RGB LED control |
| Ultrasonic | `robot.xsound` | 4 ultrasonic distance sensors |
| Compass | `robot.compass` | Compass heading & IMU data |
| TFT Display | `robot.tft` | ST7735 TFT display |
| I2C | `I2CManager::getInstance()` | I2C bus management (singleton) |

---

## Module Documentation

### Motor — Motor Control

Controls 4 DC motors with speed setting, direction flipping, and physical position mapping.

**Default motor ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `M1` | `0` | Left Front |
| `M2` | `1` | Right Front |
| `M3` | `2` | Right Back |
| `M4` | `3` | Left Back |

**Default direction rules:**
- Positive motor speed (`+`) → Counter-clockwise rotation (CCW)
- All motors positive (`+`) → Robot rotates clockwise (CW)

#### Methods

| Method | Description |
|--------|-------------|
| `setSpeed(MOTOR_ID mi, int16_t speed)` | Set motor speed, range `-255` (CCW) ~ `+255` (CW), `0` = brake |
| `stopAll()` | Stop all motors (brake mode) |
| `flipMotor(MOTOR_ID mi, bool flip = true)` | Flip single motor rotation direction |
| `flipMotors(bool m1, bool m2, bool m3, bool m4)` | Set rotation direction for each motor individually |
| `configuration(MOTOR_ID LF, MOTOR_ID RF, MOTOR_ID RB, MOTOR_ID LB)` | Remap motor ports to physical positions |
| `testAll(int16_t speed)` | Test motors sequentially (M1→M2→M3→M4) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // If needed, remap motor positions (swap Left Front and Right Front)
  robot.motor.configuration(M2, M1, M3, M4);

  // Flip motor rotation direction
  robot.motor.flipMotor(M2, true);
}

void loop() {
  // Set motor speed (positive = CCW, negative = CW)
  robot.motor.setSpeed(M1, 150);   // Left Front CCW
  robot.motor.setSpeed(M2, 150);   // Right Front CW — direction flipped
  robot.motor.setSpeed(M3, -100);  // Right Back CW
  robot.motor.setSpeed(M4, -100);  // Left Back CW
  delay(2000);
  robot.motor.stopAll();           // Stop all motors (brake mode)
  delay(1000);
}
```

#### Compatibility Wrappers

```cpp
robot.setMotorSpeed(M1, 150);   // Same as motor.setSpeed()
robot.stopAllMotors();          // Same as motor.stopAll()
```

#### Related Examples

[examples/Version4/Motor/Motor.ino](examples/Version4/Motor/Motor.ino)

---

### Movement — Omnidirectional Movement

Enables omnidirectional movement using 45° omni wheels, with angle control and compass PID correction.

#### Methods

| Method | Description |
|--------|-------------|
| `byAngle(float mAngle, float mSpeed, float rotate)` | Move at specified angle, `mAngle` = `0-360°`, `mSpeed` = `0-255`, `rotate` = `-255~+255` |
| `byAnglePID(float mAngle, float mSpeed, float compassReading)` | Move with compass PID correction, auto-maintains heading |
| `test(float speed)` | Test movement patterns (forward → right-front → rightward) |

#### Subclasses

| Member | Class | Description |
|--------|-------|-------------|
| `robot.move.converter` | `Converter` | Angle conversion tool (see [#Converter](#converter)) |
| `robot.move.motorPID` | `PIDController` | PID controller (default `Kp=300.0`, `Ki=1.0`, `Kd=2.0`) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // Adjust coordinate system
  robot.move.converter.reset().shift(0);  // Reset, then offset 0 degrees
  robot.move.converter.flip();            // Flip 180 degrees (CW<->CCW)
}

void loop() {
  // Basic movement
  robot.move.byAngle(0, 100, 0);  // Forward
  robot.move.byAngle(90, 100, 0); // Left (flipped)
  robot.move.byAngle(0, 0, 100);  // Rotate clockwise

  // Movement with compass correction
  robot.move.byAnglePID(0, 100, robot.compass.read());
}
```

#### Compatibility Wrappers

```cpp
robot.moveByAngle(0, 100, 0);   // Same as move.byAngle()
robot.moveByAnglePID(0, 100);   // Same as move.byAnglePID() — reads compass internally
```

#### Related Examples

[examples/Version4/Movement/Movement.ino](examples/Version4/Movement/Movement.ino)

---

### ColorSensor — Color Sensor

Supports up to `8` color sensors (`CL1`–`CL8`), communicating via software I2C (address `0x11`, each sensor uses a different software I2C Master). Reads color index, RGB, HSL, and raw RGBC values.

**Default sensor ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `CL1` | `0` | Sensor 1 |
| `CL2` | `1` | Sensor 2 |
| `CL3` | `2` | Sensor 3 |
| `CL4` | `3` | Sensor 4 |
| `CL5` | `4` | Sensor 5 |
| `CL6` | `5` | Sensor 6 |
| `CL7` | `6` | Sensor 7 |
| `CL8` | `7` | Sensor 8 |

**Color index mapping:**

| Name | Actual ID | Color |
|------|-----------|-------|
| `CLR_BLACK` | `0` | Black |
| `CLR_WHITE` | `1` | White |
| `CLR_GREY` | `2` | Grey |
| `CLR_RED` | `3` | Red |
| `CLR_GREEN` | `4` | Green |
| `CLR_BLUE` | `5` | Blue |
| `CLR_YELLOW` | `6` | Yellow |
| `CLR_CYAN` | `7` | Cyan |

#### Data Structures

| Structure | Fields | Description |
|-----------|--------|-------------|
| `rgbc_t` | `r, g, b, c` | RGBC raw values (each `0-65535`, `uint16_t`) |
| `rgb_t` | `r, g, b` | RGB values (each `0-255`, `uint16_t`) |
| `hsl_t` | `h, s, l` | HSL values, `h -> uint16_t`, `s, l -> uint8_t` |

#### Methods

| Method | Description |
|--------|-------------|
| `readColor(CLR_SENSOR_ID)` | Read `color` index (`0-7`) |
| `readRGB(CLR_SENSOR_ID)` | Read `RGB` values (each `0-255`) |
| `readHSL(CLR_SENSOR_ID)` | Read `HSL` values |
| `readRGBRaw(CLR_SENSOR_ID)` | Read `raw RGBC` values (each `0-65535`) |
| `setEnabled(uint8_t mask)` | Set enable mask (default `0x0F` = `CL1-CL4`) |
| `enableSensor(CLR_SENSOR_ID, bool)` | `Enable` / `disable` a single sensor |
| `isEnabled(CLR_SENSOR_ID)` | Check if sensor is enabled |
| `whiteLedOn(CLR_SENSOR_ID)` | `Turn on` bottom white LED |
| `whiteLedOff(CLR_SENSOR_ID)` | `Turn off` bottom white LED |
| `rgbwLedOn(CLR_SENSOR_ID)` | `Turn on` top RGBW LED (shows detected color) |
| `rgbwLedOff(CLR_SENSOR_ID)` | `Turn off` top RGBW LED |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // Enable/disable sensors
  robot.colorSensor.setEnabled(0b00001111);   // Enable CL1-CL4
  robot.colorSensor.enableSensor(CL5, false); // Disable CL5
}

void loop() {
  // Read CL1 sensor color information
  uint8_t colorIdx = robot.colorSensor.readColor(CL1);
  rgb_t   rgb      = robot.colorSensor.readRGB(CL1);
  hsl_t   hsl      = robot.colorSensor.readHSL(CL1);
  rgbc_t  raw      = robot.colorSensor.readRGBRaw(CL1);

  // Control CL1 sensor LEDs
  robot.colorSensor.whiteLedOn(CL1);
  robot.colorSensor.rgbwLedOff(CL1);
}
```

#### Compatibility Wrappers

```cpp
uint8_t colorIdx = robot.getColorSensor(CL1);     // Same as readColor()
rgb_t   rgb      = robot.getColorSensorRGB(CL1);  // Same as readRGB()
hsl_t   hsl      = robot.getColorSensorHSL(CL1);  // Same as readHSL()
```

#### Related Examples

[examples/Version4/Colour_Sensor/Colour_Sensor.ino](examples/Version4/Colour_Sensor/Colour_Sensor.ino)

[examples/Version4/ScreenColor/ScreenColor.ino](examples/Version4/ScreenColor/ScreenColor.ino)

---

### CompoundEye — IR Compound Eye

12-channel infrared sensor array for detecting the soccer ball position. Communicates via hardware I2C (address `0x13`).

#### Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| `0x00` | 12 bytes | `IR1–IR12` raw readings (`0-255`) |
| `0x0C` | 1 byte | Maximum IR value |
| `0x0D` | 1 byte | Maximum IR index (`1-12`) |
| `0x0E` | 1 byte | Angle (multiply by 2 for `0-360°`) |
| `0x0F` | 1 byte | Mode (`0`=single IR, `1`=dual IR) |

#### Methods

| Method | Description |
|--------|-------------|
| `readAll()` | Read all 12 IR sensor values, returns `uint8_t*` array |
| `getMaxEye()` | Get the index of the sensor with maximum value (`0-11`) |
| `getMaxEyeVal()` | Get the maximum sensor reading |
| `getEyeVal(uint8_t n)` | Get the value of a specific sensor index (`0-11`) |
| `getAngle()` | Calculate ball direction angle (`0-360°`) |
| `getMode()` | Get detection mode (`0` = single IR, `1` = dual IR) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Read all IR values
  uint8_t* ir = robot.compoundEye.readAll();
  for (int i = 0; i < 12; i++) {
    Serial.print(ir[i]);
    Serial.print(" ");
  }

  // Read a single IR sensor value (index 0-11)
  uint8_t val = robot.compoundEye.getEyeVal(5);
  Serial.print("IR5: ");
  Serial.println(val);

  // Get ball position
  uint8_t maxEye = robot.compoundEye.getMaxEye();
  uint8_t maxVal = robot.compoundEye.getMaxEyeVal();
  uint16_t angle = robot.compoundEye.getAngle();
}
```

#### Compatibility Wrappers

```cpp
uint8_t* ir = robot.compoundEyeRead();        // Same as readAll()
uint8_t val = robot.compoundEyeVal(5);        // Same as getEyeVal()
uint8_t maxEye = robot.compoundMaxEye();      // Same as getMaxEye()
uint8_t maxVal = robot.compoundMaxEyeVal();   // Same as getMaxEyeVal()
uint16_t angle = robot.compoundEyeAngle();    // Same as getAngle()
```

#### Related Examples

[examples/Version4/CompoundEye/CompoundEye.ino](examples/Version4/CompoundEye/CompoundEye.ino)

[examples/Version4/ScreenIR/ScreenIR.ino](examples/Version4/ScreenIR/ScreenIR.ino)

---

### ButtonManager — Button Management

Manages 4 buttons (`BTN_1`–`BTN_4`) with press, release, hold, and double-tap gesture detection.

**Button ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `BTN_1` | `1` | Button 1 |
| `BTN_2` | `2` | Button 2 |
| `BTN_3` | `3` | Button 3 |
| `BTN_4` | `4` | Button 4 |

**Gesture state mapping:**

| Name | Actual ID | Description |
|------|-----------|-------------|
| `NONE` | `0` | No event |
| `TAP` | `1` | Single tap |
| `PRESS` | `2` | Press |
| `HOLD` | `3` | Long press (1000ms) |
| `TAP2` | `4` | Double tap |
| `TAP3` | `5` | Triple tap |
| `RELEASE` | `6` | Release |
| `HOLD2` | `13` | Extra long press |

#### Methods

| Method | Description |
|--------|-------------|
| `read(BUTTON_ID btn)` | Read whether button is pressed (`true` / `false`) |
| `update()` | Update button state machine (must be called regularly in `loop()`) |
| `getStatus(BUTTON_ID btn)` | Get button status |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Basic read
  if (robot.buttonMgr.read(BTN_1)) {
    Serial.println("Button 1 pressed");
  }

  // Gesture detection
  robot.buttonMgr.update();
  buttonStatus_t status = robot.buttonMgr.getStatus(BTN_1);
  switch (status) {
    case TAP:  Serial.println("TAP");  break;
    case TAP2: Serial.println("DOUBLE TAP"); break;
    case HOLD: Serial.println("HOLD"); break;
  }
}
```

#### Compatibility Wrappers

```cpp
bool pressed = robot.buttonRead(BTN_1);          // Same as read()
robot.buttonUpdate();                            // Same as update()
buttonStatus_t s = robot.buttonGetStatus(BTN_1); // Same as getStatus()
```

#### Related Examples

[examples/Version4/Button/Button.ino](examples/Version4/Button/Button.ino)

[examples/Version4/Button_StateMachine/Button_StateMachine.ino](examples/Version4/Button_StateMachine/Button_StateMachine.ino)

---

### LedController — LED Control

Controls the on-board RGB LED with 8 color modes.

**LED color mapping:**

| Name | Actual ID | Color |
|------|-----------|-------|
| `LED_OFF` | `0` | Off |
| `LED_BLUE` | `1` | Blue |
| `LED_GREEN` | `2` | Green |
| `LED_CYAN` | `3` | Cyan |
| `LED_RED` | `4` | Red |
| `LED_PURPLE` | `5` | Purple |
| `LED_YELLOW` | `6` | Yellow |
| `LED_WHITE` | `7` | White |

#### Methods

| Method | Description |
|--------|-------------|
| `setLED(obBrdLEDCL color)` | Set LED color directly |
| `setLED(uint8_t LED, uint8_t status)` | Set individual LED (`0=Red`, `1=Green`, `2=Blue`) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Set color
  robot.ledCtrl.setLED(LED_RED);
  robot.ledCtrl.setLED(LED_CYAN);

  // Individual control
  robot.ledCtrl.setLED(0, HIGH);  // Red LED on
  robot.ledCtrl.setLED(1, LOW);   // Green LED off
}
```

#### Compatibility Wrappers

```cpp
robot.setOnBrdLED(LED_CYAN);
robot.setOnBrdLED(0, HIGH);
```

#### Related Examples

[examples/Version4/LED/LED.ino](examples/Version4/LED/LED.ino)

---

### Ultrasonic — Ultrasonic Sensor

Manages 4 ultrasonic distance sensors (U1–U4), using Pin Change Interrupt for non-blocking distance measurement and round-robin triggering to avoid interference.

**Default sensor ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `U1` | `0` | Front |
| `U2` | `1` | Right |
| `U3` | `2` | Back |
| `U4` | `3` | Left |

#### Methods

| Method | Description |
|--------|-------------|
| `read(ULTR_SENSOR sensor)` | Read distance (`mm`, range `0–4500`mm) |
| `configuration(ULTR_SENSOR Front, Right, Back, Left)` | Remap sensor physical positions |
| `setEnabled(bool u1, bool u2, bool u3, bool u4)` | Set enable state |
| `enableSensor(ULTR_SENSOR sensor, bool enabled)` | Enable/disable a `single` sensor |
| `enableAll(bool enabled)` | Enable/disable `all` sensors |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // If needed, remap sensor positions (swap U1 and U2, i.e., front and right)
  robot.xsound.configuration(U2, U1, U3, U4);
  
  // Enable/disable
  robot.xsound.enableSensor(U1, true);
  robot.xsound.enableAll(true);
}

void loop() {
  // Read distances from each direction
  uint16_t front = robot.xsound.read(U1);
  uint16_t right = robot.xsound.read(U2);
}
```

#### Compatibility Wrappers

```cpp
uint16_t dist = robot.ultrasonicRead(U1);
```

#### Related Examples

[examples/Version4/Ultrasonic/Ultrasonic.ino](examples/Version4/Ultrasonic/Ultrasonic.ino)

[examples/Version4/ScreenXsound/ScreenXsound.ino](examples/Version4/ScreenXsound/ScreenXsound.ino)

---

### Compass — Compass & IMU

Reads compass heading (0–360°) and 9-axis IMU raw data (accelerometer, gyroscope, magnetometer) via hardware I2C.

#### Methods

| Method | Description |
|--------|-------------|
| `read()` | Read compass heading (`0–360°`), clockwise |
| `getAccelerometerRaw()` | Get raw accelerometer data `int16_t[3]` (X, Y, Z) |
| `getGyroscopeRaw()` | Get raw gyroscope data `int16_t[3]` (X, Y, Z) |
| `getMagnetometerRaw()` | Get raw magnetometer data `int16_t[3]` (X, Y, Z) |
| `clearBuffer()` | Clear receive buffer |

#### Subclasses

| Member | Description |
|--------|-------------|
| `robot.compass.converter` | Angle conversion tool (see [#Converter](#converter)) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // Set coordinate system
  robot.compass.converter.reset().shift(0);  // Reset, then offset 0 degrees
  robot.compass.converter.flip();            // Flip 180 degrees (CW<->CCW)
}

void loop() {
  // Read heading
  uint16_t heading = robot.compass.read();
  Serial.print("Heading: ");
  Serial.println(heading);

  // Read IMU raw data
  int16_t* accel = robot.compass.getAccelerometerRaw();
  int16_t* gyro  = robot.compass.getGyroscopeRaw();
  int16_t* mag   = robot.compass.getMagnetometerRaw();
}
```

#### Compatibility Wrappers

```cpp
uint16_t heading = robot.compassRead();
int16_t* accel = robot.getAccelerometerRaw();
int16_t* gyro  = robot.getGyroscopeRaw();
int16_t* mag   = robot.getMagnetometerRaw();
```

#### Related Examples

[examples/Version4/Compass/Compass.ino](examples/Version4/Compass/Compass.ino)

[examples/Version4/CompassCar/CompassCar.ino](examples/Version4/CompassCar/CompassCar.ino)

[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)

---

### I2C — I2C Bus Management

Manages 8 software I2C buses (SW0–SW7) and 1 hardware I2C bus (HW) using the singleton pattern.

**Bus index mapping:**

| Name | Actual ID | Type |
|------|-----------|------|
| `SW0`–`SW7` | `0`–`7` | Software I2C |
| `HW` | `8` | Hardware I2C |

**I2C_Handle structure:**

| Field | Description |
|-------|-------------|
| `busIndex` | Bus index (`BusIndex`) |
| `deviceAddress` | I2C device address (`0x00`–`0x7F`) |
| `speed` | I2C speed (`Hz`) |
| `isValid()` | Check if the Handle is valid |

#### Methods

| Method | Description |
|--------|-------------|
| `I2CManager::getInstance()` | Get singleton instance |
| `init()` | Initialize all I2C buses |
| `RegisterDevice(BusIndex, address, speed)` | Register I2C device, returns `I2C_Handle` |
| `SensorRead(handle, reg, buffer, length)` | Read I2C register data |
| `SensorSend(handle, buffer, length)` | Send data to I2C device |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
I2CManager& i2c = I2CManager::getInstance();

void setup() {
  robot.init();

  // Register device
  I2C_Handle device = i2c.RegisterDevice(BusIndex::HW, 0x08, 400000);
}

void loop() {
  // Read data
  uint8_t buffer[8];
  i2c.SensorRead(device, 0x55, buffer, 3);
}
```

#### Related Tests

[test/test_i2c_manager/test_i2c_manager.ino](test/test_i2c_manager/test_i2c_manager.ino)

---

### TFT Display

ST7735 TFT display (128×160 pixels), communicates via SPI, inherits from the PDQ_GFX graphics library.

#### Methods

| Method | Description |
|--------|-------------|
| `clearScreen()` | Clear screen (fill black) |
| `setTextColor(uint16_t color)` | Set text foreground color |
| `setTextColor(uint16_t fg, uint16_t bg)` | Set foreground and background color |
| `setTextSize(uint8_t size)` | Set text size (1-3) |
| `setScreen(uint8_t col, uint8_t row, char string[])` | Display text at specified position |
| `setScreen(uint8_t col, uint8_t row, int16_t number)` | Display number at specified position |
| `drawAnglePointer(x, y, radius, angle, color)` | Draw angle pointer (with N/S/E/W markers) |

#### Color Constants

```cpp
ST7735_BLACK   // 0x0000
ST7735_WHITE   // 0xFFFF
ST7735_RED     // 0x001F
ST7735_GREEN   // 0x07E0
ST7735_BLUE    // 0xF800
ST7735_YELLOW  // 0x07FF
ST7735_MAGENTA // 0xF81F
ST7735_CYAN    // 0xFFE0
```

#### Direct Drawing

Use `robot.tft` to access PDQ_ST7735 drawing methods directly:

```cpp
robot.tft.fillCircle(x, y, r, color);
robot.tft.drawRect(x, y, w, h, color);
robot.tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
robot.tft.drawLine(x0, y0, x1, y1, color);
// More methods available — refer to PDQ_GFX documentation
```

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  robot.clearScreen();
  robot.setTextSize(2);
  robot.setTextColor(ST7735_YELLOW);
  robot.setScreen(0, 0, "Hello");
}

void loop() {
  // Clear old value
  robot.setTextColor(ST7735_BLACK);
  robot.setScreen(0, 1, (int16_t)oldValue);
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 1, (int16_t)newValue);

  // Draw angle pointer
  robot.drawAnglePointer(64, 120, 25, heading);
}
```

#### Related Examples

[examples/Version4/LCDScreen/LCDScreen.ino](examples/Version4/LCDScreen/LCDScreen.ino)

[examples/Version4/ScreenColor/ScreenColor.ino](examples/Version4/ScreenColor/ScreenColor.ino)

[examples/Version4/ScreenIR/ScreenIR.ino](examples/Version4/ScreenIR/ScreenIR.ino)

[examples/Version4/ScreenXsound/ScreenXsound.ino](examples/Version4/ScreenXsound/ScreenXsound.ino)

[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)

---

### Utility Classes

#### Converter

Angle conversion tool supporting flip, shift, and normalization. Can be used for compass calibration or movement coordinate adjustment.

```cpp
class Converter {
public:
  Converter& flip();            // Flip direction (+/- invert)
  Converter& shift(float deg);  // Shift angle offset
  float normalize(float angle); // Normalize to [0, 360)
  float convert(float angle);   // Apply transformation
  void reset();                 // Reset to defaults
};
```

Usage example:

```cpp
// Chained method calls (reset then offset)
robot.compass.converter.reset().shift(90).flip();

// Or set individually
robot.compass.converter.reset();
robot.compass.converter.shift(90);
robot.compass.converter.flip();

// Movement coordinate adjustment
robot.move.converter.reset().shift(180);
```

#### PIDController

PID control algorithm used for compass heading correction in the Movement module.

```cpp
class PIDController {
public:
  PIDController(double kp, double ki, double kd);
  double update(double currentValue);  // Calculate PID output

  double kp, ki, kd;        // PID coefficients
  double setPoint;          // Target value
  double integral;          // Integral term
  double previousError;     // Previous error
};
```

Usage example:

```cpp
// Custom PID parameters (defaults: Kp=300.0, Ki=1.0, Kd=2.0)
robot.move.motorPID.kp = 200.0;
robot.move.motorPID.ki = 0.5;
robot.move.motorPID.kd = 1.0;
```

---

## Examples

| Example | Description |
|---------|-------------|
| [Bluetooth_Remote](examples/Version4/Bluetooth_Remote/Bluetooth_Remote.ino) | Bluetooth remote control |
| [Button](examples/Version4/Button/Button.ino) | Basic button reading |
| [Button_StateMachine](examples/Version4/Button_StateMachine/Button_StateMachine.ino) | Button state machine (TAP/HOLD) |
| [Colour_Sensor](examples/Version4/Colour_Sensor/Colour_Sensor.ino) | Color sensor reading |
| [Compass](examples/Version4/Compass/Compass.ino) | Compass & IMU data |
| [CompassCar](examples/Version4/CompassCar/CompassCar.ino) | Compass navigation |
| [CompoundEye](examples/Version4/CompoundEye/CompoundEye.ino) | IR compound eye |
| [Digital_Analog](examples/Version4/Digital_Analog/Digital_Analog.ino) | GPIO digital/analog I/O |
| [Goalkeeper](examples/Version4/Goalkeeper/Goalkeeper.ino) | Goalkeeper behavior strategy |
| [LCDScreen](examples/Version4/LCDScreen/LCDScreen.ino) | TFT display |
| [LED](examples/Version4/LED/LED.ino) | RGB LED control |
| [Motor](examples/Version4/Motor/Motor.ino) | Motor test & configuration |
| [Movement](examples/Version4/Movement/Movement.ino) | Omnidirectional movement |
| [ScreenColor](examples/Version4/ScreenColor/ScreenColor.ino) | Screen + color sensor integration |
| [ScreenCompass](examples/Version4/ScreenCompass/ScreenCompass.ino) | Screen + compass integration |
| [ScreenIR](examples/Version4/ScreenIR/ScreenIR.ino) | Screen + compound eye integration |
| [ScreenXsound](examples/Version4/ScreenXsound/ScreenXsound.ino) | Screen + ultrasonic integration |
| [Striker](examples/Version4/Striker/Striker.ino) | Striker behavior strategy |
| [Ultrasonic](examples/Version4/Ultrasonic/Ultrasonic.ino) | Ultrasonic sensor |

---

## Hardware Configuration

### Default Pin Mapping

| Function | Pin | Description |
|----------|-----|-------------|
| TFT CS | 0 | TFT chip select |
| TFT DC | 53 | TFT data/command |
| TFT RST | 50 | TFT reset |
| TFT SCL | 52 | TFT SPI clock |
| TFT SDA | 51 | TFT SPI data |
| S1–S4 | 10–13 | Servo/PWM output |
| D1–D3 | 53–55 | Digital input |
| D4–D6 | 56–58 | Digital output |
| A1–A4 | 59–62 | Analog input |

### I2C Device Addresses

| Device | Address | Bus |
|--------|---------|-----|
| Compass module | `0x08` | Hardware I2C (HW) |
| Compound eye module | `0x13` | Hardware I2C (HW) |
| Color sensor CL1–CL8 | `0x11` | Software I2C (SW0–SW7) |