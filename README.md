# PeanutKing Soccer

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-00979D)](https://www.arduino.cc/reference/en/libraries/)
[![Version](https://img.shields.io/badge/version-4.2.0-blue)](https://github.com/peanut-king-solution/PeanutKing_Soccer)

Arduino library for controlling **PeanutKing Soccer Robots** (V2 / V3 / V4 compatible).

---

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Module Overview](#module-overview)
- [Module Documentation](#module-documentation)
  - [Motor — Motor Control](#motor--motor-control)
  - [Movement — Omnidirectional Movement](#movement--omnidirectional-movement)
  - [ColorSensor — Color Sensor](#colorsensor--color-sensor)
  - [CompoundEye — IR Compound Eye](#compoundeye--ir-compound-eye)
  - [ButtonManager — Button Management](#buttonmanager--button-management)
  - [LedController — LED Control](#ledcontroller--led-control)
  - [Ultrasonic — Ultrasonic Sensor](#ultrasonic--ultrasonic-sensor)
  - [Compass — Compass & IMU](#compass--compass--imu)
  - [I2C — I2C Bus Management](#i2c--i2c-bus-management)
  - [TFT Display](#tft-display)
  - [PS2 Controller — PS2 Remote](#ps2-controller--ps2-remote)
  - [Utility Classes](#utility-classes)
- [Examples](#examples)
- [Hardware Configuration](#hardware-configuration)
- [Known Issues](#known-issues)
- [Version History](#version-history)

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

- `PDQ_GFX` / `PDQ_ST7735` — TFT display driver (128×160, SPI)
- `SoftI2cMaster` / `FastI2cMaster` — Software I2C implementation (8 independent SW I2C buses)
- `pcint` — Pin Change Interrupt handler (for non-blocking ultrasonic echo detection)

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
| Motor | `robot.motor` | 4 DC motor control with mapping and direction flipping |
| Movement | `robot.move` | 45° omni wheel omnidirectional movement |
| ColorSensor | `robot.colorSensor` | Up to 8 color sensors (software I2C, address `0x11`) |
| CompoundEye | `robot.compoundEye` | 12-channel IR sensor array for ball detection (hardware I2C, address `0x13`) |
| ButtonManager | `robot.buttonMgr` | 4-button state management (TAP, HOLD, etc.) |
| LedController | `robot.ledCtrl` | On-board RGB LED control (8 colors) |
| Ultrasonic | `robot.xsound` | 4 ultrasonic distance sensors (PCINT-based, round-robin) |
| Compass | `robot.compass` | Compass heading (0–360°) & 9-axis IMU raw data (hardware I2C, address `0x08`) |
| TFT Display | `robot.tft` | ST7735 TFT display (128×160, SPI) |
| I2C | `I2CManager::getInstance()` | I2C bus management singleton (HW + 8×SW) |
| PS2 Controller | `robot.ps2x` | PS2 wireless controller (via PS2X_lib) |

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
| `flipMotor(MOTOR_ID mi, bool flip = true)` | Flip single motor rotation direction (`false` to cancel) |
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
| `moveWithCorr(float mAngle, float mSpeed, float compassReading)` | Move with compass PID correction, auto-maintains heading |
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
  robot.move.WithCorr(0, 100, robot.compass.read());
}
```

#### Compatibility Wrappers

```cpp
robot.moveByAngle(0, 100, 0);  // Same as move.byAngle()
robot.moveWithCorr(0, 100);    // Same as move.withCorr() — reads compass internally
```

#### Related Examples

[examples/Version4/Movement/Movement.ino](examples/Version4/Movement/Movement.ino)

---

### ColorSensor — Color Sensor

Supports up to `8` color sensors (`CL1`–`CL8`), communicating via software I2C (address `0x11`, each sensor uses a dedicated software I2C Master). Reads color index, RGB, HSL, and raw RGBC values.

**Default sensor ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `CL1` | `0` | Front (default) |
| `CL2` | `1` | Right (default) |
| `CL3` | `2` | Back (default) |
| `CL4` | `3` | Left (default) |
| `CL5`–`CL8` | `4–7` | Extra sensors (disabled by default) |

**Color index mapping (readColor register `0x01`):**

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
| `hsl_t` | `h, s, l` | HSL values (`h` → `uint16_t`, `s`&`l` → `uint8_t`) |
| `GreenBaseLine` | `greenHue, greenLight, greenSat, done` | Calibrated green field baseline |

#### Methods

| Method | Description |
|--------|-------------|
| `readColor(CLR_SENSOR_ID)` | Read color index (`0-7`) |
| `readRGB(CLR_SENSOR_ID)` | Read RGB values |
| `readHSL(CLR_SENSOR_ID)` | Read HSL values |
| `readRGBRaw(CLR_SENSOR_ID)` | Read raw RGBC values |
| `setEnabled(uint8_t mask)` | Set enable mask (default `0x0F` = `CL1`–`CL4` enabled) |
| `enableSensor(CLR_SENSOR_ID, bool)` | Enable/disable a single sensor |
| `isEnabled(CLR_SENSOR_ID)` | Check if sensor is enabled |
| `whiteLedOn(CLR_SENSOR_ID)` | Turn on bottom white LED |
| `whiteLedOff(CLR_SENSOR_ID)` | Turn off bottom white LED |
| `rgbwLedOn(CLR_SENSOR_ID)` | Turn on top RGBW LED (which shows detected color) |
| `rgbwLedOff(CLR_SENSOR_ID)` | Turn off top RGBW LED (which shows detected color) |
| `configuration(CLR_SENSOR_ID F, R, B, L)` | Map sensors to front/right/back/left positions |
| `calBaseline(CLR_SENSOR_ID, samples=10)` | Calibrate baseline of green field (HSL averaging) |
| `isCalibrated(CLR_SENSOR_ID)` | Check if calibration is complete |
| `getBaseline(CLR_SENSOR_ID)` | Get calibrated `GreenBaseLine` struct |
| `isWhiteLine(CLR_SENSOR_ID)` | Check if sensor detects white line (3D HSL check: Light + Sat + Hue, 2/3 vote) |

#### White Line Detection (Plan A: Baseline)

The white line detection uses a **calibrated baseline** approach:

1. **Calibration** (`calBaseline()`): Call once on green field. Averages 10 HSL samples to establish the green baseline (Hue, Saturation, Lightness).

2. **Detection** (`isWhiteLine()`): Compares current reading against the baseline using three dimensions:
   - `lightCheck`: 20%+ brighter than green baseline
   - `satCheck`: saturation below 80% of green baseline
   - `hueCheck`: hue differs by more than 30°

   White is detected when **2 out of 3** conditions are met, improving reliability in varying lighting conditions.

3. **Thresholds**: Dynamic — computed from the calibrated baseline values:
   - `lightThreshold = greenLight / 5` (adapts to ambient brightness)
   - `satThreshold = greenSat * 80%` (adapts to sensor saturation range)

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // Calibrate all 4 sensors (ensure robot is on green field)
  for (uint8_t i = 0; i < 4; i++) {
    robot.colorSensor.calBaseline((CLR_SENSOR_ID)i);
  }

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

  // White line detection (3D HSL check, 2/3 vote)
  bool isFrontWhite = robot.colorSensor.isWhiteLine(CL1);

  // Get calibrated baseline values
  if (robot.colorSensor.isCalibrated(CL1)) {
    GreenBaseLine bl = robot.colorSensor.getBaseline(CL1);
    // bl.greenHue, bl.greenSat, bl.greenLight
  }

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
bool isFrontWhite = robot.whiteLineCheck(CL1);    // Same as isWhiteLine()
```

#### Related Examples

[examples/Version4/Colour_Sensor/Colour_Sensor.ino](examples/Version4/Colour_Sensor/Colour_Sensor.ino)

[examples/Version4/ScreenColor/ScreenColor.ino](examples/Version4/ScreenColor/ScreenColor.ino)

[examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino](examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino)

[examples/Version4/OutOfBound/OutOfBound.ino](examples/Version4/OutOfBound/OutOfBound.ino)

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

Manages 4 buttons (`BTN_1`–`BTN_4`) with basic read support. Advanced gesture detection (TAP, HOLD, double-tap) is declared but **not yet implemented** — `update()` is a placeholder.

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
| `read(BUTTON_ID btn)` | Read whether button is physically pressed (`true`/`false`) |
| `update()` | Update button state machine (**not yet implemented** — see [Known Issues](#known-issues)) |
| `getStatus(BUTTON_ID btn)` | Get current button gesture status |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // Basic read (works)
  if (robot.buttonMgr.read(BTN_1)) {
    Serial.println("Button 1 pressed");
  }

  // Gesture detection (update() is a placeholder — see Known Issues)
  // robot.buttonMgr.update();
  // buttonStatus_t status = robot.buttonMgr.getStatus(BTN_1);
}
```

#### Compatibility Wrappers

```cpp
bool pressed = robot.buttonRead(BTN_1);          // Same as read()
robot.buttonUpdate();                            // Same as update() — placeholder
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
| `setLED(obBrdLEDCL color)` | Set LED to a predefined color (uses bitmask: `R=bit0`, `G=bit1`, `B=bit2`) |
| `setLED(uint8_t LED, uint8_t status)` | Set individual LED channel (`0`=Red, `1`=Green, `2`=Blue), `0`=off, `1`=on |

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

| Name | Actual ID | Position | Trig Pin | Echo Pin |
|------|-----------|----------|----------|----------|
| `U1` | `0` | Front | 49 | A15 |
| `U2` | `1` | Right | 48 | A14 |
| `U3` | `2` | Back | 47 | A13 |
| `U4` | `3` | Left | 46 | A12 |

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

  // If needed, remap sensor positions (swap U1 and U2)
  robot.xsound.configuration(U2, U1, U3, U4);
  
  // Enable/disable
  robot.xsound.enableSensor(U1, true);
  robot.xsound.enableAll(true);
}

void loop() {
  // Read distances from each direction
  uint16_t front = robot.xsound.read(U1);
  uint16_t right = robot.xsound.read(U2);
  uint16_t back  = robot.xsound.read(U3);
  uint16_t left  = robot.xsound.read(U4);
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

Reads compass heading (0–360°) and 9-axis IMU raw data (accelerometer, gyroscope, magnetometer) via hardware I2C (address `0x08`).

#### Methods

| Method | Description |
|--------|-------------|
| `read()` | Read compass heading (`0–360°`, clockwise) |
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

Manages 8 software I2C buses (`SW0`–`SW7`) and 1 hardware I2C bus (`HW`) using a singleton pattern.

**Bus index mapping:**

| Name | Actual ID | Type | Typical Use |
|------|-------|------|-------------|
| `SW0`–`SW7` | `0`–`7` | Software I2C (bit-bang) | Color sensors CL1–CL8 |
| `HW` | `8` | Hardware I2C (TWI) | Compass, CompoundEye |

**Software I2C pin mapping:**

| Bus | SCL | SDA |
|-----|-----|-----|
| SW0 | 30 | 29 |
| SW1 | 32 | 31 |
| SW2 | 34 | 33 |
| SW3 | 36 | 35 |
| SW4 | 38 | 37 |
| SW5 | 40 | 39 |
| SW6 | 42 | 41 |
| SW7 | 44 | 43 |

**Hardware I2C:** Uses Arduino Mega default pins (SCL=21, SDA=20) or Wire.h auto-detect.

**I2C_Handle structure:**

| Field | Description |
|-------|-------------|
| `busIndex` | Bus index (`BusIndex`) |
| `deviceAddress` | I2C device address (`0x00`–`0x7F`) |
| `speed` | I2C speed in `Hz` |
| `isValid()` | Check if the handle is valid |

#### Methods

| Method | Description |
|--------|-------------|
| `I2CManager::getInstance()` | Get singleton instance |
| `init()` | Initialize all I2C buses (HW + SW) |
| `RegisterDevice(BusIndex, address, speed)` | Register I2C device, returns `I2C_Handle` |
| `SensorRead(handle, reg, buffer, length)` | Read I2C register data |
| `SensorSend(handle, buffer, length)` | Send data to I2C device |

#### Wire.h Compatibility

The hardware I2C backend supports two modes, auto-detected at compile time:

1. **Arduino Wire.h** (preferred) — auto-detected if `<Wire.h>` is available
2. **IICIT** (fallback) — custom hardware I2C implementation for AVR

To force Wire.h usage, define `USE_WIRE_H` before including the library:
```cpp
#define USE_WIRE_H
#include <PeanutKingSoccerV4.h>
```

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
I2CManager& i2c = I2CManager::getInstance();

void setup() {
  robot.init();

  // Register custom device on hardware I2C
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
| `setScreen(uint8_t col, uint8_t row, char string[])` | Display text at grid position (col×6, row×10) |
| `setScreen(uint8_t col, uint8_t row, int16_t number)` | Display number at grid position |
| `drawAnglePointer(x, y, radius, angle, color)` | Draw angle pointer with N/S/E/W markers |

#### Color Constants

| Constant | Value | Color |
|----------|-------|-------|
| `ST7735_BLACK` | `0x0000` | Black |
| `ST7735_WHITE` | `0xFFFF` | White |
| `ST7735_RED` | `0x001F` | Red |
| `ST7735_GREEN` | `0x07E0` | Green |
| `ST7735_BLUE` | `0xF800` | Blue |
| `ST7735_YELLOW` | `0x07FF` | Yellow |
| `ST7735_MAGENTA` | `0xF81F` | Magenta |
| `ST7735_CYAN` | `0xFFE0` | Cyan |

#### Direct Drawing

Access the underlying `PDQ_ST7735` instance directly via `robot.tft`:

```cpp
robot.tft.fillCircle(x, y, r, color);
robot.tft.drawRect(x, y, w, h, color);
robot.tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
robot.tft.drawLine(x0, y0, x1, y1, color);
// More methods — refer to PDQ_GFX documentation
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
  // Clear old value, display new value
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

### PS2 Controller — PS2 Remote

Wireless PS2 controller interface using the PS2X_lib library. Supports button states (pressed/holding/released), joystick angle + strength readings, and vibration feedback.

**Pin assignment:**
Only `CLK` and `DAT` pins need to be specified; `CMD` and `ATT` are automatically assigned based on the gap between them.

| Pin | Role | Description |
|-----|------|-------------|
| `CLK` | Clock | Digital pin (`D0_P`–`D5_P`) |
| `DAT` | Data | Digital pin (`D0_P`–`D5_P`) |
| `CMD` | Command | Auto-assigned (between CLK and DAT) |
| `ATT` | Attention | Auto-assigned (between CLK and DAT) |

**Button ID mapping:**

| Name | Description |
|------|-------------|
| `PS2Button::SELECT` | Select button |
| `PS2Button::L3` | Left joystick button |
| `PS2Button::R3` | Right joystick button |
| `PS2Button::START` | Start button |
| `PS2Button::UP` / `DOWN` / `LEFT` / `RIGHT` | D-pad directions |
| `PS2Button::L1` / `L2` | Left shoulder buttons |
| `PS2Button::R1` / `R2` | Right shoulder buttons |
| `PS2Button::TRIANGLE` / `CIRCLE` / `CROSS` / `SQUARE` | Right side action buttons |

**Joystick data structure (`PS2JoystickData`):**

| Field | Type | Description |
|-------|------|-------------|
| `angle` | `float` | Direction angle (0–360°) |
| `strength` | `float` | Push strength (0–255) |

#### Methods

| Method | Description |
|--------|-------------|
| `ps2Init(CLK, DAT, pressure, vibration)` | Initialize PS2 controller — auto-assigns CMD/ATT pins, returns `0` on success, error code otherwise |
| `ps2Update()` | Read latest controller state (call once per loop) |
| `ps2SetVibration(byte strength)` | Set vibration motor strength (0–255) |
| `ps2ButtonPressed(PS2Button btn)` | Check if button was just pressed (edge-triggered) |
| `ps2ButtonHolding(PS2Button btn)` | Check if button is being held down (level-triggered) |
| `ps2ButtonReleased(PS2Button btn)` | Check if button was just released (edge-triggered) |
| `ps2ButtonRead(PS2Button btn)` | Read button state struct (⚠️ **not yet implemented** — returns empty struct) |
| `ps2JoystickRead(PS2Joystick js)` | Read joystick angle + strength (square boundary scaling) |

#### Example — Basic Reading

```cpp
#include <PeanutKingSoccerV4.h>

PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  delay(300);

  // CLK=D6_P(56), DAT=D3_P(59), middle pins CMD=57, ATT=58 auto assigned
  byte error = robot.ps2Init(D6_P, D3_P, false, true);
  if (error) {
    Serial.print("PS2 init error: ");
    Serial.println(error);
    while (1) {};
  }
  Serial.println("PS2 OK");
}

void loop() {
  robot.ps2Update();

  // Edge-triggered button detection
  if (robot.ps2ButtonPressed(PS2Button::CROSS)) {
    Serial.println("CROSS pressed");
  }
  if (robot.ps2ButtonReleased(PS2Button::L1)) {
    Serial.println("L1 released");
  }

  // Level-triggered button detection
  if (robot.ps2ButtonHolding(PS2Button::UP)) {
    Serial.println("UP holding");
  }

  // Joystick reading (angle + strength)
  if (robot.ps2ButtonHolding(PS2Button::L1)) {
    PS2JoystickData lj = robot.ps2JoystickRead(PS2Joystick::LEFT);
    Serial.print("L angle:"); Serial.print(lj.angle);
    Serial.print(" str:"); Serial.println(lj.strength);
    robot.ps2SetVibration(lj.strength);
  }

  delay(50);
}
```

#### Example — PS2 Remote Control

```cpp
#include <PeanutKingSoccerV4.h>

PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  delay(300);
  byte error = robot.ps2Init(D6_P, D3_P, false, true);
  if (error) { while (1) {}; }
}

void loop() {
  robot.ps2Update();

  // Hold L1 + move left joystick to drive
  if (robot.ps2ButtonHolding(PS2Button::L1)) {
    PS2JoystickData lj = robot.ps2JoystickRead(PS2Joystick::LEFT);
    int moveSpeed = lj.strength * 130 / 255;
    robot.moveWithCorr(lj.angle, moveSpeed);
    robot.ps2SetVibration(lj.strength);
  }
  if (robot.ps2ButtonReleased(PS2Button::L1)) {
    robot.ps2SetVibration(0);
    robot.stopAllMotors();
  }
}
```

#### Compatibility Wrappers

None

#### Related Examples

[examples/Version4/PS2/PS2.ino](examples/Version4/PS2/PS2.ino)

[examples/Version4/PS2Remote/PS2Remote.ino](examples/Version4/PS2Remote/PS2Remote.ino)

---

### Utility Classes

#### Converter

Angle conversion tool supporting flip, shift, and normalization. Used for compass calibration and movement coordinate adjustment.

```cpp
class Converter {
public:
  Converter& flip();            // Flip direction (+/- invert)
  Converter& shift(float deg);  // Shift angle offset
  float normalize(float angle); // Normalize to [0, 360)
  float convert(float angle);   // Apply transformation
  void reset();                 // Reset to defaults (multiplier=1, offset=0)
};
```

Usage example:

```cpp
// Chained method calls (reset then offset)
robot.compass.converter.reset().shift(90).flip();

// Movement coordinate adjustment
robot.move.converter.reset().shift(180);
```

#### PIDController

PID control algorithm used for compass heading correction in the Movement module.

```cpp
class PIDController {
public:
  PIDController(double kp, double ki, double kd);
  double update(double currentValue);  // Calculate PID output (error = setPoint - currentValue)

  double kp, ki, kd;        // PID coefficients
  double setPoint;          // Target value (default: 0.0)
  double integral;          // Integral term accumulator
  double previousError;     // Previous error value
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

### Version 4 (22 examples)

| Example | Description |
|---------|-------------|
| [Bluetooth_Remote](examples/Version4/Bluetooth_Remote/Bluetooth_Remote.ino) | Bluetooth remote control (skeleton — functions are empty) |
| [Button](examples/Version4/Button/Button.ino) | Basic button reading |
| [Button_StateMachine](examples/Version4/Button_StateMachine/Button_StateMachine.ino) | Button state machine (TAP/HOLD — requires ButtonManager::update() implementation) |
| [Colour_Sensor](examples/Version4/Colour_Sensor/Colour_Sensor.ino) | Color sensor reading (color index, RGB, HSL, RGBC raw, white line check) |
| [Compass](examples/Version4/Compass/Compass.ino) | Compass & IMU data (heading, accelerometer, gyroscope, magnetometer) |
| [CompassCar](examples/Version4/CompassCar/CompassCar.ino) | Compass navigation (heading-based motor control) |
| [CompoundEye](examples/Version4/CompoundEye/CompoundEye.ino) | IR compound eye (12 sensors, max eye, ball angle) |
| [Digital_Analog](examples/Version4/Digital_Analog/Digital_Analog.ino) | GPIO digital/analog I/O (uses S_PIN, D_PIN, A_PIN enums) |
| [Goalkeeper](examples/Version4/Goalkeeper/Goalkeeper.ino) | Goalkeeper behavior strategy (⚠️ uses deprecated `motorSet()`/`motorStop()` — see [Known Issues](#known-issues)) |
| [LCDScreen](examples/Version4/LCDScreen/LCDScreen.ino) | TFT display basics (text, shapes, tick counter) |
| [LED](examples/Version4/LED/LED.ino) | RGB LED control (cycles through all 8 colors) |
| [Motor](examples/Version4/Motor/Motor.ino) | Motor test & configuration (mapping, direction, speed) |
| [Movement](examples/Version4/Movement/Movement.ino) | Omnidirectional movement (byAngle, withCorr, rotation) |
| [OutOfBound](examples/Version4/OutOfBound/OutOfBound.ino) | Square movement with white line detection |
| [PS2](examples/Version4/PS2/PS2.ino) | PS2 controller basic reading (button states, joystick angle + strength) |
| [PS2Remote](examples/Version4/PS2Remote/PS2Remote.ino) | PS2 controller remote control (joystick-driven movement) |
| [ScreenColor](examples/Version4/ScreenColor/ScreenColor.ino) | Screen + color sensor integration (color name, RGB display) |
| [ScreenCompass](examples/Version4/ScreenCompass/ScreenCompass.ino) | Screen + compass integration (heading display + pointer) |
| [ScreenIR](examples/Version4/ScreenIR/ScreenIR.ino) | Screen + compound eye integration (6×2 grid display + angle pointer) |
| [ScreenWhiteLine](examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino) | Screen + color sensor white line detection (HSL + baseline display) |
| [ScreenXsound](examples/Version4/ScreenXsound/ScreenXsound.ino) | Screen + ultrasonic integration (4 distances with labels) |
| [Striker](examples/Version4/Striker/Striker.ino) | Striker behavior strategy (⚠️ uses deprecated `motorSet()`/`motorStop()` — see [Known Issues](#known-issues)) |
| [Ultrasonic](examples/Version4/Ultrasonic/Ultrasonic.ino) | Ultrasonic sensor (configuration, enable/disable, distance reading) |

---

## Hardware Configuration

### Default Pin Mapping (Arduino Mega)

| Function | Pin(s) | Description |
|----------|--------|-------------|
| TFT CS | 0 | TFT chip select |
| TFT DC | 53 | TFT data/command |
| TFT RST | 50 | TFT reset |
| TFT SCL | 52 | TFT SPI clock |
| TFT SDA | 51 | TFT SPI data |
| Motor IN1 | 9, 7, 5, 3 | Motor channel 1 (PWM) |
| Motor IN2 | 8, 6, 4, 2 | Motor channel 2 (PWM) |
| Ultrasonic Trig | 49, 48, 47, 46 | U1–U4 trigger pins |
| Ultrasonic Echo | A15, A14, A13, A12 | U1–U4 echo pins (PCINT) |
| Button | 22, 23, 24, 25 | BTN_1–BTN_4 (INPUT_PULLUP) |
| LED RGB | 26, 28, 27 | Red, Green, Blue |
| SW I2C (×8) | 29–44 | SCL=30/32/34/36/38/40/42/44, SDA=29/31/33/35/37/39/41/43 |
| Servo/PWM | 10–13 | S1–S4 |
| Digital Input | 53–55 | D1–D3 |
| Digital Output | 56–58 | D4–D6 |
| Analog Input | 59–62 | A1–A4 |

### I2C Device Addresses

| Device | Address | Bus |
|--------|---------|-----|
| Compass module | `0x08` | Hardware I2C (HW) |
| Compound eye module | `0x13` | Hardware I2C (HW) |
| Color sensor CL1–CL8 | `0x11` | Software I2C (SW0–SW7) |

### Timer Usage

| Timer | Width | PWM Pins | Used By | Notes |
|-------|-------|----------|---------|-------|
| Timer0 | 8-bit | 13, 4 | Arduino core (`millis`/`delay`) | ⚠️ Do not modify prescaler |
| Timer1 | 16-bit | 12, 11 | Motor PWM | Default 3921 Hz |
| Timer2 | 8-bit | 9, 10 | Motor PWM | Default 980 Hz |
| Timer3 | 16-bit | 5, 3, 2 | — | Available |
| Timer4 | 16-bit | 8, 7, 6 | — | Available |
| Timer5 | 16-bit | 46, 45, 44 | — | Available |

---

## Known Issues

### 🔴 Compile Errors (Examples)

- **Goalkeeper.ino** and **Striker.ino** use deprecated `motorSet()` and `motorStop()` functions that do not exist in V4. These examples will **fail to compile**. Use `setMotorSpeed()` / `stopAllMotors()` instead.

### 🟡 Not Yet Implemented

| Feature | Location | Status |
|---------|----------|--------|
| `ButtonManager::update()` | `ButtonManager.cpp` | Placeholder — gesture detection (TAP, HOLD, TAP2) not implemented |
| `bluetoothRemote()` | `PeanutKingSoccerV4.cpp` | Empty function — Bluetooth not implemented |
| `bluetoothAttributes()` | `PeanutKingSoccerV4.cpp` | Empty function — Bluetooth not implemented |
| `Chase()` / `Back()` | `PeanutKingSoccerV4.cpp` | Empty functions — strategy not implemented |
| `ps2ButtonRead()` | `PeanutKingSoccerV4.cpp` | Returns empty struct — full button state reading not implemented |

### 🟢 Minor Issues

- `pwmPin[4]` declared in `PeanutKingSoccerV4.h` but never used
- `BUTTON_ID` enum starts at 1, requiring `-1` conversion for array indexing
- Private member naming is inconsistent across modules (`_` prefix vs no prefix)
- `Compass::converter` and `Movement::converter`/`motorPID` are public members (encapsulation)

---

## Version History
