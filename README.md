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
- [API Reference](#api-reference)
  - [Motor — Motor Control](#motor--motor-control)
  - [Movement — Omnidirectional Movement](#movement--omnidirectional-movement)
  - [ColorSensor — Color Sensor](#colorsensor--color-sensor)
  - [CompoundEye — IR Compound Eye](#compoundeye--ir-compound-eye)
  - [Button — Button Control](#button--button-control)
  - [LED — on-board RGB LED Control](#led--on-board-rgb-led-control)
  - [Ultrasound — Ultrasonic Sensor](#ultrasound--ultrasonic-sensor)
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

Remind: V2, V3, V4 are not compatible to each other, each version has it corresponding hardward kit set. For example, version V4 is expected to use with Peanut King Mega shield V4.

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

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init(); // Initialize all modules
}

void loop() {
  // Read compass heading
  uint16_t heading = robot.compassRead();         // (or robot.compass.read())
  Serial.print("Heading: ");
  Serial.println(heading);

  // Read ultrasonic distance
  uint16_t dist = robot.ultrasoundGetDist(Front);
  Serial.print("Front distance: ");
  Serial.println(dist);

  // Set LED color (enum values: LEDOff, LEDRed, LEDGreen, ... — no `LEDColor::` prefix)
  robot.onBoardLedSet(LEDCyan);

  delay(100);
}
```

> ⚠️ **Single-Robot Constraint**
>
> `PeanutKingSoccerV4` is designed for **one physical robot per Arduino board**. All hardware (motors, sensors, I2C bus, serial) is shared, so you **must not** create more than one `PeanutKingSoccerV4` instance in a sketch. Because every module pins/registers map to the same physical hardware, a second instance would cause pin conflicts, duplicated I2C registration, and unpredictable sensor readings.
>
> The module classes (`Motor`, `Button`, `LED`, `Ultrasound`, `ColorSensor`, `Compass`, `CompoundEye`, `Bluetooth`, `Movement`) have **private constructors** accessible only to `PeanutKingSoccerV4`. Users should **only** access them through the public instances exposed on the robot object, e.g. `robot.motor`, `robot.compass`.

---

## Module Overview

| Module | Instance | Description |
|--------|----------|-------------|
| Motor | `robot.motor` | 4 DC motor control with mapping and direction flipping |
| Movement | `robot.movement` | 45° omni wheel omnidirectional movement |
| ColorSensor | `robot.colorSensor` | Up to 8 color sensors (software I2C, address `0x11`) |
| CompoundEye | `robot.compoundEye` | 12-channel IR sensor array for ball detection (hardware I2C, address `0x13`) |
| Button | `robot.button` | button control |
| LED | `robot.led` | On-board RGB LED control (8 colors) |
| Ultrasound | `robot.ultrasound` | 4 ultrasonic distance sensors (PCINT-based, round-robin) |
| Compass | `robot.compass` | Compass heading (0–360°) & 9-axis IMU raw data (hardware I2C, address `0x08`) |
| TFT Display | `robot.tft` | ST7735 TFT display (128×160, SPI) |
| I2C | `I2CManager::getInstance()` | I2C bus management singleton (HW + 8×SW) |
| PS2 Controller | `robot.ps2x` | PS2 wireless controller (via PS2X_lib) |
| Bluetooth | `robot.bluetooth` | BLE module (HM-10) with AT commands, connection management, and data send |

---

## Module Documentation

### Motor — Motor Control

Controls 4 DC motors with speed setting, direction flipping, and physical position mapping.

**Default motor port mapping:**

| Name | Value | Position |
|------|-------|----------|
| `M1` | `0`   | Left Front |
| `M2` | `1`   | Right Front |
| `M3` | `2`   | Right Back |
| `M4` | `3`   | Left Back |

> **Default direction rules:**
> - Positive motor speed (`+`) → Counter-clockwise rotation (CCW)
> - All motors positive (`+`) → Robot rotates clockwise (CW)

**MotorId enum** — `getPortFromPos()` maps a `MotorPos` to the corresponding `MotorId` (`M1`–`M4`). Invalid positions are handled gracefully: the wrapper methods validate via `portValidCheck()` and return early for invalid ports.

**MotorPos enum (physical position):**

| Name            | Value |
|-----------------|-------|
| `LeftFront`     | `0`   |
| `RightFront`    | `1`   |
| `RightBack`     | `2`   |
| `LeftBack`      | `3`   |

> V4 high-level API takes a **physical position** (`MotorPos`) as input, resolves it to the actual port (`M1`-`M4`) via `getPortFromPos()`, then drives the corresponding motor.

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `motorConfiguration(MotorId LF, MotorId RF, MotorId RB, MotorId LB)` | Remap motor ports to physical positions |
| `motorFlipDirection(MotorPos pos, bool flip = true)` | Flip rotation direction of the motor at a position |
| `motorSetSpeed(MotorPos pos, int16_t speed)` | Set motor speed, range `-255` ~ `+255`, `0` = brake (positive = CCW, negative = CW) |
| `motorStop(MotorPos pos)` | Stop a single motor (brake mode) |
| `motorStopAll()` | Stop all motors (brake mode) |
| `motorTestAll(int16_t speed, int duration = 1000)` | Test all motors sequentially (LF→RF→RB→LB), each held for `duration` ms |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // If needed, remap motor positions (swap Left Front and Right Front)
  robot.motorConfiguration(M2, M1, M3, M4);

  // Flip motor rotation direction at a position
  robot.motorFlipDirection(RightFront, true);
}

void loop() {
  // Set motor speed (positive = CCW, negative = CW)
  robot.motorSetSpeed(LeftFront, 150);   // Left Front CCW
  robot.motorSetSpeed(RightFront, 150);  // Right Front CW — direction flipped
  robot.motorSetSpeed(RightBack, -100);  // Right Back CW
  robot.motorSetSpeed(LeftBack, -100);   // Left Back CW
  delay(2000);
  robot.motorStopAll(); // Stop all motors (brake mode)
  delay(1000);
}
```

#### Low-level Module Methods (`robot.motor.`)

For direct port-based control. Ports take `MotorId` (`M1`-`M4`), not positions.

| Method | Description |
|--------|-------------|
| `init()` | Initialize all motor pins as OUTPUT |
| `getPortFromPos(MotorPos pos)` | Convert a physical position to the mapped motor port (`M1`-`M4`) |
| `mapPort(MotorId LF, MotorId RF, MotorId RB, MotorId LB)` | Remap motor ports to physical positions |
| `flipDirection(MotorId mi, bool flip = true)` | Flip rotation direction of a single motor port |
| `setSpeed(MotorId mi, int16_t speed)` | Set a single motor speed by port, range `-255` ~ `+255`, `0` = brake |
| `stop(MotorId mi)` | Brake a single motor port |

```cpp
// Low-level equivalent: drive port M1 directly
MotorId mi = robot.motor.getPortFromPos(LeftFront);
robot.motor.flipDirection(mi, true);
robot.motor.setSpeed(mi, 150);
robot.motor.stop(mi);
```

#### Related Examples

[examples/Version4/Motor/Motor.ino](examples/Version4/Motor/Motor.ino)

---

### Movement — Omnidirectional Movement

Enables omnidirectional movement using 45° omni wheels, with angle control and compass PID correction.

> **Architecture** — `Movement` is a **pure computation layer**: it computes the four wheel speeds and returns a `WheelSpeeds` struct, but does **not** drive motors directly. Mapping to physical positions and driving is handled by the V4 high-level `move()` wrapper.

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `move(float mAngle, float mSpeed, float rotate = 0)` | Move and drive motors using the enabled correction mode |
| `movementCoordinateReset()` | Reset the high-level movement coordinate system (→ `movement.converter.reset()`) |
| `movementCoordinateRotate(uint16_t rotateAngle, RotationDir dir = CW)` | Rotate the high-level movement coordinate system by a non-negative angle. Direction is `CW` by default; use `CCW` for counter-clockwise rotation (→ `movement.converter.rotate()`) |
| `movementCoordinateFlip()` | Flip the high-level movement coordinate direction (→ `movement.converter.flip()`) |

> **Example**: After `movementCoordinateRotate(90, CCW)`, the angles will be rotated as follows:
> ```
>     Before                                    After
>        0°                                      90°
>   315° ↑  45°                              45°  ↑  135°
>      \ | /          rotate(90, CCW)           \ | /
> 270°←-   -→ 90°           ->              0° ←-   -→ 180°
>      / | \      rotate counter-clockwise      / | \
>   215° ↓  135°                            315°  ↓  225°
>       180°                                     270°
> ```

#### Movement mode selection (`move()`)

The V4 wrapper `robot.move()` reads compass + 4 white-line sensors, selects the movement method based on two enable flags, computes `WheelSpeeds`, maps physical positions to motor ports (`getPortFromPos()`), and drives the motors.

| Enable Flag | Default | Description |
|-------------|---------|-------------|
| `compassCorrectEnabled` | `true` | Enable compass correction |
| `outBoundPreventEnabled` | `false` | Enable out-of-bounds prevention (⚠️ requires `outBoundPrevent`/`correctedMove` to be implemented) |

```cpp
// Both features off -> plain omnidirectional movement
robot.compassCorrectEnabled = false;
robot.outBoundPreventEnabled = false;
robot.move(45, 100);

// Compass correction only (out-of-bounds prevention not yet implemented)
robot.compassCorrectEnabled = true;
robot.outBoundPreventEnabled = false;
robot.move(45, 100);
```

> **Note:** `outBoundPreventEnabled` defaults to `false` until `outBoundPrevent()`/`correctedMove()` logic is implemented. Enabling both flags currently calls the *zeroed* stub methods.

#### Data Structure

| Type |                 Fields                  |                                            Description                                            |
|------|-----------------------------------------|---------------------------------------------------------------------------------------------------|
| `WheelSpeeds` | `leftFront, rightFront, rightBack, leftBack` (`int16_t`) | Speeds for LeftFront / RightFront / RightBack / LeftBack (position-based) |

#### Low-level Module Methods (`robot.movement.`)

| Method | Description |
|--------|-------------|
| `byAngle(float mAngle, float mSpeed, float rotate)` | Compute wheel speeds at angle, `mAngle` = `0-360°`, `mSpeed` = `0-255`, `rotate` = `-255~+255` |
| `withCorr(float mAngle, float mSpeed, float compassReading)` | Compute wheel speeds with compass PID correction, maintains heading |
| `outBoundPrevent(float mAngle, float mSpeed, bool isOutBound[4])` | Compute wheel speeds to prevent moving out of bounds — ⚠️ TODO (returns zeroed speeds) |
| `correctedMove(float mAngle, float mSpeed, float compassReading, bool isOutBound[4])` | Combined compass + out-of-bounds prevention — ⚠️ TODO (returns zeroed speeds) |

This class computes wheel speeds and returns a `WheelSpeeds` struct; it does **not** drive motors directly. You can use the `robot.move()` wrapper to read sensors and drive motors automatically. Or you can directly call the low-level methods to compute wheel speeds and then drive motors by the functions in Motor (see [Motor — Motor Control](#motor--motor-control)).

The coordinate system is adjusted directly through the public `robot.movement.converter` (`reset()` / `rotate()` / `flip()`, see [Converter — Angle Conversion](#converter--angle-conversion)).

```cpp
// Compute wheel speeds without compass correction (rotate is optional)
WheelSpeeds ws = robot.movement.byAngle(90, 100, 50);   // Move 90° at speed 100, rotate 50

// Compute wheel speeds with compass heading correction (no rotate parameter)
WheelSpeeds ws2 = robot.movement.withCorr(90, 100, 180);// Move 90° at speed 100, hold heading 180°

// Adjust the coordinate system via the public converter member
robot.movement.converter.reset();     // Reset to default
robot.movement.converter.rotate(90);  // Rotate coordinate system 90° clockwise (default CW)
robot.movement.converter.flip();      // Flip 180° (CW <-> CCW)

// The result is applied to motors by the top-level move() wrapper
robot.move(90, 100, 50);            // Move 90° at speed 100, rotate 50
```

> `outBoundPrevent()` and `correctedMove()` are **not yet implemented** — they return zeroed wheel speeds. Do not rely on out-of-bounds prevention.

#### Tuning Parameters

| Member | Type | Default | Description |
|--------|------|---------|-------------|
| `compassDeadZone` | `float` | `0.05` | Compass rotation dead zone — errors below this are ignored to prevent oscillation |
| `minRotateSpeed` | `float` | `60.0` | Minimum rotation speed — PID correction smaller than this is clamped up so the robot still turns |
| `motorPID` | `PIDController` | `Kp=300.0, Ki=1.0, Kd=2.0` | Compass correction PID controller — tune via `motorPID.setKp()` etc. |
| `converter` | `Converter` | reset | Movement coordinate system — via `converter.rotate()`/`converter.flip()`/`converter.reset()` |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // movement configuration: enable compass correction and out-of-bounds prevention
  robot.compassCorrectEnabled = true;
  robot.outBoundPreventEnabled = false; // not yet implemented

  // motor configuration: assign which motor port controls which wheel position
  // robot.motorConfiguration(M1, M2, M3, M4);
  
  // Adjust coordinate system (high-level wrapper)
  robot.movementCoordinateReset();        // Reset to default coordinate system
  robot.movementCoordinateRotate(90);     // Rotate coordinate system 90° clockwise (default CW)
  robot.movementCoordinateRotate(90, CCW);// Rotate coordinate system 90° counter-clockwise
  robot.movementCoordinateFlip();         // Flip 180 degrees (CW<->CCW)
}

void loop() {
  // or use the high-level wrapper which reads sensors and drives motors:
  robot.move(0, 100, 0);  // Forward
  robot.move(90, 100, 0); // Right

  robot.compassCorrectEnabled = false; // Disable compass correction
  robot.move(0, 0, 100);  // Rotate clockwise

  robot.motorStopAll();   // Stop all motors
}
```

#### Related Examples

[examples/Version4/Movement/Movement.ino](examples/Version4/Movement/Movement.ino)

[examples/Version4/MoveSquare/MoveSquare.ino](examples/Version4/MoveSquare/MoveSquare.ino)

---

### ColorSensor — Color Sensor

Supports up to `8` color sensors (`CL1`–`CL8`), communicating via software I2C (address `0x11`, each sensor uses a dedicated software I2C Master). Reads RGB, HSL, and raw RGBC values, with white-line detection.

**Default sensor ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `CL1` | `0` | Front (default) |
| `CL2` | `1` | Right (default) |
| `CL3` | `2` | Back (default) |
| `CL4` | `3` | Left (default) |
| `CL5`–`CL8` | `4–7` | Extra sensors (disabled by default) |

> **Note**: The color-index reading (`readColor`, register `0x01`) and the `CLR_BLACK`…`CLR_CYAN` constants have been **removed** in V4. Data is read via the struct-returning methods (`readRGB` / `readHSL` / `readRGBRaw`) instead.

#### Methods (V4 high-level wrapper)

**V4 high-level API takes a physical position (`SensorPos`) as input**, e.g. `Front`/`Right`/`Back`/`Left`, resolves it to the actual sensor port (`CL1`–`CL8`) via `getPortFromPos()`, then reads / configures the corresponding sensor.

| Method | Description |
|--------|-------------|
| `colorSensorConfiguration(F, R, B, L)` | Map sensors to positions |
| `colorSensorReadRGBC(pos)` | Read raw RGBC by position |
| `colorSensorReadRGB(pos)` | Read RGB by position |
| `colorSensorReadHSL(pos)` | Read HSL by position |
| `isWhiteLine(pos)` | White line detection by position |
| `colorSensorCalBaseline(pos, samples=10)` | Calibrate baseline by position |
| `colorSensorGetBaseline(pos)` | Get baseline by position |

#### Data Structures

##### RGBC

| Name | Type | Description |
|------|------|-------------|
| `r` | `uint32_t` | Red raw value (0~65535) |
| `g` | `uint32_t` | Green raw value (0~65535) |
| `b` | `uint32_t` | Blue raw value (0~65535) |
| `c` | `uint32_t` | Clear raw value (0~65535) |

##### RGB

| Name | Type | Description |
|------|------|-------------|
| `r` | `uint16_t` | Red value (0~255) |
| `g` | `uint16_t` | Green value (0~255) |
| `b` | `uint16_t` | Blue value (0~255) |

##### HSL

| Name | Type | Description |
|------|------|-------------|
| `h` | `uint16_t` | Hue value (0~360) |
| `s` | `uint8_t` | Saturation value (0~100) |
| `l` | `uint8_t` | Lightness value (0~100) |

#### Low-level Module Methods (`robot.colorSensor.`)

For direct port-based control. Ports take `ColorSensorId` (`CL1`–`CL8`), not positions.

| Method | Description |
|--------|-------------|
| `readRGBRaw(ColorSensorId)` | Read raw RGBC values (returns `RGBC` with `uint32_t` fields) |
| `readRGB(ColorSensorId)` | Read RGB values |
| `readHSL(ColorSensorId)` | Read HSL values |
| `setEnableMask(uint8_t mask)` | Set enable mask (default `0x0F` = `CL1`–`CL4` enabled) |
| `enable(ColorSensorId, bool)` | Enable/disable a single color sensor |
| `isEnabled(ColorSensorId)` | Check if sensor is enabled |
| `whiteLedOn(ColorSensorId)` | Turn on bottom white LED |
| `whiteLedOff(ColorSensorId)` | Turn off bottom white LED |
| `rgbwLedOn(ColorSensorId)` | Turn on top RGBW LED (which shows detected color) |
| `rgbwLedOff(ColorSensorId)` | Turn off top RGBW LED (which shows detected color) |
| `mapPort(ColorSensorId F, R, B, L)` | Map sensors to front/right/back/left positions |
| `calBaseline(ColorSensorId, samples=10)` | Calibrate baseline of green field (HSL averaging) |
| `getBaseline(ColorSensorId)` | Get calibrated `GreenBaseline` struct |
| `isWhiteLine(ColorSensorId)` | Check if sensor detects white line (hue deviation from green baseline) |

#### White Line Detection (Plan A: Baseline)

The white line detection uses a **calibrated baseline** approach:

1. **Calibration** (`calBaseline()`): Call once on green field. Averages 10 HSL samples to establish the green baseline (Hue, Saturation, Lightness).

2. **Detection** (`isWhiteLine()`): Compares the current hue reading against the calibrated green baseline hue:
   - `hueThreshold = greenHue / 8` (dynamic, adapts to the green field hue)
   - White is detected when the current hue is **more than one threshold above the baseline hue** (`hsl.h > greenHue + hueThreshold`)

   The comparison uses a single hue-deviation check against the calibrated baseline, keeping the threshold adaptive to lighting conditions.

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // Configure color sensor ports to their default positions
  //  - front color sensor is connected to CL1
  //  - right color sensor is connected to CL2
  //  - back color sensor is connected to CL3
  //  - left color sensor is connected to CL4
  // robot.colorSensorConfiguration(CL1, CL2, CL3, CL4);
}

void loop() {
  // Read RGBC raw values from the front color sensor
  RGBC rgbc = robot.colorSensorReadRGBC(Front);
  Serial.print("RGBC: R="); Serial.print(rgbc.r);
  Serial.print(", G="); Serial.print(rgbc.g);
  Serial.print(", B="); Serial.print(rgbc.b);
  Serial.print(", C="); Serial.println(rgbc.c);

  // Read RGB values from the front color sensor
  RGB rgb = robot.colorSensorReadRGB(Front);
  Serial.print("RGB: R="); Serial.print(rgb.r);
  Serial.print(", G="); Serial.print(rgb.g);
  Serial.print(", B="); Serial.println(rgb.b);

  // Read HSL values from the front color sensor
  HSL hsl = robot.colorSensorReadHSL(Front);
  Serial.print("HSL: H="); Serial.print(hsl.h);
  Serial.print(", S="); Serial.print(hsl.s);
  Serial.print(", L="); Serial.println(hsl.l);

  // Check if the front color sensor detects a white line
  if (robot.isWhiteLine(Front)) {
    Serial.println("White line detected!");
  }

  delay(100);
}
```

#### Related Examples

[examples/Version4/ColorSensor/ColorSensor.ino](examples/Version4/ColorSensor/ColorSensor.ino)

[examples/Version4/ScreenColorSensor/ScreenColorSensor.ino](examples/Version4/ScreenColorSensor/ScreenColorSensor.ino)

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
| `0x0D` | 1 byte | Maximum IR index (`0-11`) |
| `0x0E` | 1 byte | Angle (multiply by 2 for `0-360°`) |
| `0x0F` | 1 byte | Mode (`0`=single IR, `1`=dual IR) |

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `compoundEyeReadAll()` | Read all 12 IR sensor values |
| `compoundMaxEyeRead()` | Get the index of the eye with maximum reading |
| `compoundMaxEyeValueRead()` | Get the maximum IR sensor value |
| `compoundEyeValueRead(EyeId eyeIndex)` | Get the value of a specific eye |
| `compoundEyeAngleRead()` | Get the angle of the detected object (`0-360°`) |
| `compoundEyeModeRead()` | Get detection mode (`0` = single IR, `1` = dual IR) |
| `compoundEyeCoordinateReset()` | Reset the compound eye coordinate system to default |
| `compoundEyeCoordinateRotate(uint16_t rotateAngle, RotationDir dir = CW)` | Rotate the compound eye coordinate system by a non-negative angle |
| `compoundEyeCoordinateFlip()` | Flip the compound eye coordinate system direction (CW ↔ CCW) |

#### Low-level Module Methods (`robot.compoundEye.`)

These are the **CompoundEye module methods** (low-level, access via `robot.compoundEye`):

| Method | Description |
|--------|-------------|
| `readAll()` | Read all 12 IR sensor values, returns `uint8_t*` array |
| `readMaxEye()` | Get the index of the sensor with maximum value (`0-11`) |
| `readMaxEyeVal()` | Get the maximum sensor reading |
| `readEyeVal(EyeId n)` | Get the value of a specific sensor index (`0-11`) |
| `readAngle()` | Calculate ball direction angle (`0-360°`) — applies `converter.convert()` |
| `readMode()` | Get detection mode (`0` = single IR, `1` = dual IR) |
| `converter` | Coordinate system adjustment (see [Converter](#converter)) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // Adjust compound eye coordinate system (high-level wrapper)
  robot.compoundEyeCoordinateReset();        // Reset to default
  robot.compoundEyeCoordinateRotate(90);     // Rotate coordinate system 90° clockwise
  robot.compoundEyeCoordinateFlip();         // Flip direction (CW ↔ CCW)
}

void loop() {
  // Read all IR values
  uint8_t* ir = robot.compoundEyeReadAll();
  for (int i = 0; i < 12; i++) {
    Serial.print(ir[i]);
    Serial.print(" ");
  }

  // Get ball position (angle is converted via converter)
  uint16_t angle = robot.compoundEyeAngleRead();
  EyeId maxEye = robot.compoundMaxEyeRead();
  uint8_t maxVal = robot.compoundMaxEyeValueRead();
}
```

#### Related Examples

[examples/Version4/CompoundEye/CompoundEye.ino](examples/Version4/CompoundEye/CompoundEye.ino)

[examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino](examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino)

---

### Button — Button Control

Controls 4 buttons (`Button1`–`Button4`) with a state machine that detects `ButtonPressed`, `ButtonHolding`, and `ButtonReleased`.

**Button ID mapping:**

| Name | Actual ID | Position |
|------|-----------|----------|
| `Button1` | `0` | Button 1 |
| `Button2` | `1` | Button 2 |
| `Button3` | `2` | Button 3 |
| `Button4` | `3` | Button 4 |

**Button state mapping:**

| Name | Actual ID | Description |
|------|-----------|-------------|
| `ButtonIdle` | `0` | Button is idle |
| `ButtonPressed` | `1` | Button just pressed |
| `ButtonHolding` | `2` | Button is being held down |
| `ButtonReleased` | `3` | Button just released |

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `robot.buttonUpdate()` | Update button state machine; call once per loop before reading |
| `robot.buttonStateRead(btn)` | Read current `ButtonState` after `buttonUpdate()` |

#### Low-level Module Methods (`robot.button.`)

| Method | Description |
|--------|-------------|
| `robot.button.init()` | Initialize button pins as `INPUT_PULLUP` |
| `robot.button.update()` | Update button state machine |
| `robot.button.readState(btn)` | Read current `ButtonState` |
| `robot.button.setDebounceTime(ms)` | Set debounce time (default 50ms) |
| `robot.button.setHoldTime(ms)` | Set hold time (default 1000ms) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Update button state machine first
  robot.buttonUpdate();

  // Then read the state of each button
  ButtonState state = robot.buttonStateRead(Button1);
  switch (state) {
    case ButtonIdle:    break;
    case ButtonPressed: Serial.println("Button1 pressed");  break;
    case ButtonHolding: Serial.println("Button1 holding");  break;
    case ButtonReleased: Serial.println("Button1 released"); break;
  }
}
```

#### Related Examples

[examples/Version4/Button/Button.ino](examples/Version4/Button/Button.ino)

---

### LED — on-board RGB LED Control

Controls the on-board RGB LED with 8 color modes.

**LED color mapping:**

| Name | Color |
|------|-------|
| `LEDOff` | Off |
| `LEDBlue` | Blue |
| `LEDGreen` | Green |
| `LEDCyan` | Cyan |
| `LEDRed` | Red |
| `LEDPurple` | Purple |
| `LEDYellow` | Yellow |
| `LEDWhite` | White |

> **Note**: `LEDColor` is a plain `enum`, so values are used **without** the `LEDColor::` prefix (e.g. `LEDRed`, not `LEDColor::RED`).

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `robot.onBoardLedSet(color)` | Set the on-board LED to a predefined `LEDColor` |

#### Low-level Module Methods (`robot.led.`)

| Method | Description |
|--------|-------------|
| `robot.led.setLED(LEDColor color)` | Set all channels to a predefined color |
| `robot.led.setLED(uint8_t channel, uint8_t status)` | Set one channel; header order `0=Blue, 1=Green, 2=Red` |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Set color (high-level wrapper)
  robot.onBoardLedSet(LEDRed);
  robot.onBoardLedSet(LEDCyan);

  // Individual control (low-level, via robot.led)
  robot.led.setLED(0, HIGH);  // Channel 0 (header: Blue) on
  robot.led.setLED(1, LOW);   // Green LED off
}
```

#### Related Examples

[examples/Version4/LED/LED.ino](examples/Version4/LED/LED.ino)

---

### Ultrasound — Ultrasonic Sensor

Manages 4 ultrasonic distance sensors (U1–U4), using Pin Change Interrupt for non-blocking distance measurement and round-robin triggering to avoid interference.

**Default sensor ID mapping:**

| Name | Actual ID | Position | Trig Pin | Echo Pin |
|------|-----------|----------|----------|----------|
| `U1` | `0` | Front | 49 | A15 |
| `U2` | `1` | Right | 48 | A14 |
| `U3` | `2` | Back | 47 | A13 |
| `U4` | `3` | Left | 46 | A12 |

#### Methods (V4 high-level wrapper)

**V4 high-level API takes a physical position (`SensorPos`) as input**, e.g. `Front`/`Right`/`Back`/`Left`, resolves it to the actual sensor port (`U1`–`U4`) via `getPortFromPos()`, then reads / configures the corresponding sensor.

```cpp
uint16_t dist = robot.ultrasoundGetDist(Front);            // Read by position
robot.ultrasoundConfiguration(U2, U1, U3, U4);             // Remap sensor ports
robot.ultrasoundSetEnabled(true, false, true, false);      // Enable Front, Back only
robot.ultrasoundEnableAll(true);                           // Enable all sensors
```


#### Low-level Module Methods (`robot.ultrasound.`)

For direct port-based control. Ports take `UltrasoundId` (`U1`–`U4`), not positions.

| Method | Description |
|--------|-------------|
| `read(UltrasoundId port)` | Read distance from a sensor port (`mm`); invalid/disabled returns `65535` |
| `enable(UltrasoundId port, bool enabled)` | Enable/disable a single sensor port |
| `setEnableMask(uint8_t mask)` | Enable/disable all sensors at once (bit0=U1 … bit3=U4) |
| `isEnabled(UltrasoundId port)` | Check if a sensor port is enabled |
| `mapPort(UltrasoundId Front, Right, Back, Left)` | Remap sensor ports to physical positions |
| `getPortFromPos(SensorPos pos)` | Convert position (`Front/Right/Back/Left`) to sensor port |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // If needed, remap sensor positions (swap U1 and U2)
  robot.ultrasound.mapPort(U2, U1, U3, U4);

  // Enable/disable individual ports
  robot.ultrasound.enable(U1, true);
}

void loop() {
  // Read distances by port (low-level API)
  uint16_t front = robot.ultrasound.read(U1);
  uint16_t right = robot.ultrasound.read(U2);
  uint16_t back  = robot.ultrasound.read(U3);
  uint16_t left  = robot.ultrasound.read(U4);
}
```

#### Related Examples

[examples/Version4/Ultrasound/Ultrasound.ino](examples/Version4/Ultrasound/Ultrasound.ino)

[examples/Version4/ScreenUltrasound/ScreenUltrasound.ino](examples/Version4/ScreenUltrasound/ScreenUltrasound.ino)

---

### Compass — Compass & IMU

Reads compass heading (0–360°) and 9-axis IMU raw data (accelerometer, gyroscope, magnetometer) via hardware I2C (address `0x08`).

#### Methods (V4 high-level wrapper)

```cpp
uint16_t heading = robot.compassRead();
int16_t* accel = robot.compassReadRawAccel();
int16_t* gyro  = robot.compassReadRawGyro();
int16_t* mag   = robot.compassReadRawMag();
```

High-level coordinate wrappers (act on `robot.compass.converter`):

| Method | Description |
|--------|-------------|
| `compassCoordinateReset()` | Reset the compass coordinate system (→ `compass.converter.reset()`) |
| `compassCoordinateRotate(uint16_t, RotationDir dir = CW)` | Rotate compass coordinate system by non-negative angle. Direction default `CW`; use `CCW` for counter-clockwise (→ `compass.converter.rotate()`) |
| `compassCoordinateFlip()` | Flip compass coordinate direction (→ `compass.converter.flip()`) |

> **Example**: After `compassCoordinateRotate(90, CCW)`, the angles will be rotated as follows:
> ```
>     Before                                    After
>        0°                                      90°
>   315° ↑  45°                              45°  ↑  135°
>      \ | /          rotate(90, CCW)           \ | /
> 270°←-   -→ 90°           ->              0° ←-   -→ 180°
>      / | \      rotate counter-clockwise      / | \
>   215° ↓  135°                            315°  ↓  225°
>       180°                                     270°
> ```

#### Low-level Module Methods (`robot.compass.`)

These are the **Compass module methods** (access via `robot.compass`):

| Method | Description |
|--------|-------------|
| `read()` | Read compass heading (`0–360°`, clockwise) |
| `readRawAccel()` | Get raw accelerometer data `int16_t[3]` (X, Y, Z) |
| `readRawGyro()` | Get raw gyroscope data `int16_t[3]` (X, Y, Z) |
| `readRawMag()` | Get raw magnetometer data `int16_t[3]` (X, Y, Z) |

#### Subclasses

| Member | Description |
|--------|-------------|
| `robot.compass.converter` | Angle conversion tool (see [#Converter](#converter)) |

#### Example

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // Set coordinate system (high-level wrappers on robot.compass.converter)
  robot.compassCoordinateReset();       // Reset coordinate system
  robot.compassCoordinateRotate(0);     // 0° rotate = identity (default CW, see Converter docs)
  robot.compassCoordinateRotate(90, CCW); // Rotate 90° counter-clockwise
  robot.compassCoordinateFlip();        // Flip 180 degrees (CW<->CCW)
}

void loop() {
  // Read heading
  uint16_t heading = robot.compass.read();
  Serial.print("Heading: ");
  Serial.println(heading);

  // Read IMU raw data
  int16_t* accel = robot.compass.readRawAccel();
  int16_t* gyro  = robot.compass.readRawGyro();
  int16_t* mag   = robot.compass.readRawMag();
}
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
| `deviceAddress` | I2C device address (`0x01`–`0x7F`; `0x00` is invalid) |
| `speed` | Requested I2C speed; current backends do not apply per-handle speed |
| `isValid()` | Check if the handle is valid |

#### Low-level Module Methods (`I2CManager`)

| Method | Description |
|--------|-------------|
| `I2CManager::getInstance()` | Get singleton instance |
| `init()` | Initialize all I2C buses (HW + SW) |
| `SensorRead(handle, reg, buffer, length)` | Read I2C register data |
| `SensorSend(handle, buffer, length)` | Send data to I2C device |

Handles are value-constructed with `I2C_Handle(BusIndex, address, speed)`. The manager does not provide `RegisterDevice()` or duplicate-address registration.

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

static PeanutKingSoccerV4 robot;
I2CManager& i2c = I2CManager::getInstance();

void setup() {
  robot.init();

  // Construct a handle for a custom hardware-I2C device.
  // Address 0x08 is already used by the compass in the standard robot setup.
  I2C_Handle device(BusIndex::HW, 0x20, 100000);
}

void loop() {
  // Read data
  uint8_t buffer[8];
  i2c.SensorRead(device, 0x55, buffer, 3);
}
```

#### Related Tests

[tests/i2c_test/i2c_test.ino](tests/i2c_test/i2c_test.ino)

---

### TFT Display

ST7735 TFT display (128×160 pixels), communicates via SPI, inherits from the PDQ_GFX graphics library.

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `screenClear()` | Clear screen (fill black) |
| `screenSetTextColor(uint16_t color)` | Set text foreground color |
| `screenSetTextColor(uint16_t fg, uint16_t bg)` | Set foreground and background color |
| `screenSetTextSize(uint8_t size)` | Set text size (1-3) |
| `screenPrintText(uint8_t col, uint8_t row, const char* string)` | Display text at grid position (col×6, row×10) |
| `screenPrintNumber(uint8_t col, uint8_t row, int16_t number)` | Display number at grid position |
| `screenDrawAnglePointer(x, y, radius, angle, arrowColor)` | Draw angle pointer with N/S/E/W markers |

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

#### Low-level Module Methods (`robot.tft.`)

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

static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
  robot.screenClear();
  robot.screenSetTextSize(2);
  robot.screenSetTextColor(ST7735_YELLOW);
  robot.screenPrintText(0, 0, "Hello");
}

void loop() {
  // Clear old value, display new value
  robot.screenSetTextColor(ST7735_BLACK);
  robot.screenPrintNumber(0, 1, (int16_t)oldValue);
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenPrintNumber(0, 1, (int16_t)newValue);

  // Draw angle pointer
  robot.screenDrawAnglePointer(64, 120, 25, heading);
}
```

#### Related Examples

[examples/Version4/LCDScreen/LCDScreen.ino](examples/Version4/LCDScreen/LCDScreen.ino)

[examples/Version4/ScreenColorSensor/ScreenColorSensor.ino](examples/Version4/ScreenColorSensor/ScreenColorSensor.ino)

[examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino](examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino)

[examples/Version4/ScreenUltrasound/ScreenUltrasound.ino](examples/Version4/ScreenUltrasound/ScreenUltrasound.ino)

[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)

---

### PS2 Controller — PS2 Remote

Wireless PS2 controller interface using the PS2X_lib library. Supports button states (pressed/holding/released), joystick angle + strength readings, and vibration feedback.

**Pin assignment:**
Only `CLK` and `DAT` pins need to be specified; `CMD` and `ATT` are automatically assigned based on the gap between them.

| Pin | Role | Description |
|-----|------|-------------|
| `CLK` | Clock | Digital pin (`D1_P`–`D6_P`) |
| `DAT` | Data | Digital pin (`D1_P`–`D6_P`) |
| `CMD` | Command | Auto-assigned (between CLK and DAT) |
| `ATT` | Attention | Auto-assigned (between CLK and DAT) |

**Button ID mapping:**

| Name | Description |
|------|-------------|
| `PS2Select` | Select button |
| `PS2L3` | Left joystick button |
| `PS2R3` | Right joystick button |
| `PS2Start` | Start button |
| `PS2Up` / `PS2Down` / `PS2Left` / `PS2Right` | D-pad directions |
| `PS2L1` / `PS2L2` | Left shoulder buttons |
| `PS2R1` / `PS2R2` | Right shoulder buttons |
| `PS2Triangle` / `PS2Circle` / `PS2Cross` / `PS2Square` | Right side action buttons |

> **Note**: `PS2Button` is a plain `enum`, so values are used **without** the `PS2Button::` prefix (e.g. `PS2Cross`, not `PS2Button::CROSS`).

**Joystick data structure (`PS2JoystickData`):**

| Field | Type | Description |
|-------|------|-------------|
| `angle` | `float` | Direction angle (0–360°) |
| `strength` | `float` | Push strength (0–255) |

#### Methods (V4 high-level wrapper)

| Method | Description |
|--------|-------------|
| `ps2Init(CLK, DAT, pressure, vibration)` | Initialize PS2 controller — auto-assigns CMD/ATT pins, returns `0` on success, error code otherwise |
| `ps2Update()` | Read latest controller state (call once per loop) |
| `ps2SetVibration(byte strength)` | Set vibration motor strength (0–255) |
| `ps2ButtonStateRead(PS2Button btn)` | Read current button state (`PS2Idle`/`PS2Pressed`/`PS2Holding`/`PS2Released`) |
| `ps2JoystickRead(PS2Joystick js)` | Read joystick angle + strength (square boundary scaling) |

#### Example — Basic Reading

```cpp
#include <PeanutKingSoccerV4.h>

PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
  delay(300);

  // CLK=D6_P, DAT=D3_P, middle pins CMD, ATT auto assigned
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

  // Read button state (PS2Idle / PS2Pressed / PS2Holding / PS2Released)
  PS2ButtonState crossState = robot.ps2ButtonStateRead(PS2Cross);
  if (crossState == PS2Pressed) {
    Serial.println("CROSS pressed");
  }
  if (crossState == PS2Released) {
    Serial.println("CROSS released");
  }

  // Holding detection
  PS2ButtonState upState = robot.ps2ButtonStateRead(PS2Up);
  if (upState == PS2Holding) {
    Serial.println("UP holding");
  }

  // Joystick reading (angle + strength)
  PS2ButtonState l1State = robot.ps2ButtonStateRead(PS2L1);
  if (l1State == PS2Holding) {
    PS2JoystickData lj = robot.ps2JoystickRead(PS2LeftJoystick);
    Serial.print("L angle:"); Serial.print(lj.angle);
    Serial.print(" str:"); Serial.println(lj.strength);
    robot.ps2SetVibration(lj.strength);
  }
  if (l1State == PS2Released) {
    robot.ps2SetVibration(0);
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
  PS2ButtonState l1State = robot.ps2ButtonStateRead(PS2L1);
  if (l1State == PS2Holding) {
    PS2JoystickData lj = robot.ps2JoystickRead(PS2LeftJoystick);
    int moveSpeed = lj.strength * 130 / 255;
    robot.move(lj.angle, moveSpeed);
    robot.ps2SetVibration(lj.strength);
  }
  if (l1State == PS2Released) {
    robot.ps2SetVibration(0);
    robot.motorStopAll();
  }
}
```

#### Low-level Module Methods (`robot.ps2x.`)

The bundled `PS2X` driver is available as `robot.ps2x`; its methods are lower-level than the wrappers above. Use `ps2Update()` and the wrapper reads unless direct driver access is required.

#### Related Examples

[examples/Version4/PS2/PS2.ino](examples/Version4/PS2/PS2.ino)

[examples/Version4/PS2Remote/PS2Remote.ino](examples/Version4/PS2Remote/PS2Remote.ino)

---

### Utility Classes

#### Converter

Angle conversion tool supporting flip, rotate, and normalization. Used for compass calibration and movement coordinate adjustment.

```cpp
enum RotationDir { CW, CCW };           // Rotation direction for rotate()

class Converter {
public:
  Converter& flip();                     // Flip direction (+/- invert)
  Converter& rotate(uint16_t angle, RotationDir dir = CW); // Rotate coordinate by non-negative angle; direction default CW
  float normalize(float angle);          // Normalize to [0, 360)
  float convert(float angle);            // Apply transformation
  void reset();                          // Reset to defaults (multiplier=1, offset=0)
};
```

Usage example:

```cpp
// Chained method calls (reset then rotate: 90° CW rotates the coordinate 90° CW; flip inverts direction CW<->CCW)
robot.compass.converter.reset();
robot.compass.converter.rotate(90).flip();       // 90° clockwise (default), then flip

// Movement coordinate adjustment (rotate 180° CW by default)
robot.movement.converter.rotate(180);            // 180° clockwise (default CW)

// Counter-clockwise rotation via explicit direction
robot.movement.converter.rotate(90, CCW);        // 90° counter-clockwise
```

#### PIDController

PID control algorithm used for compass heading correction in the Movement module.

```cpp
class PIDController {
public:
  PIDController(double kp, double ki, double kd);
  double update(double currentValue);  // Calculate PID output (error = targetPoint - currentValue)

  void setPID(double kp, double ki, double kd);  // Set all gains (non-negative, resets state if changed)
  void setKp(double v);
  void setKi(double v);
  void setKd(double v);
  void setTargetPoint(double v);       // Target value (default: 0.0)
  void reset();                        // Clear integral + previousError

  double getKp() const;
  double getKi() const;
  double getKd() const;
  double getTargetPoint() const;
};
```

Usage example:

```cpp
// Custom PID parameters (defaults: Kp=300.0, Ki=1.0, Kd=2.0)
robot.movement.motorPID.setKp(200.0);   // Tune proportional gain
robot.movement.motorPID.setKi(0.5);     // Tune integral gain
robot.movement.motorPID.setKd(1.0);     // Tune derivative gain
```

---

## Examples

### Version 4 examples

| Example | Description |
|---------|-------------|
| [Button](examples/Version4/Button/Button.ino) | Button state machine (PRESSED/HOLDING/RELEASED) |
| [ColorSensor](examples/Version4/ColorSensor/ColorSensor.ino) | Color sensor reading (RGB, HSL, RGBC raw, white line check) |
| [Compass](examples/Version4/Compass/Compass.ino) | Compass & IMU data (heading, accelerometer, gyroscope, magnetometer) |
| [CompassCar](examples/Version4/CompassCar/CompassCar.ino) | Compass navigation (heading-based motor control) |
| [CompoundEye](examples/Version4/CompoundEye/CompoundEye.ino) | IR compound eye (12 sensors, max eye, ball angle) |
| [Digital_Analog](examples/Version4/Digital_Analog/Digital_Analog.ino) | GPIO digital/analog I/O (uses `S_PIN`, `DigitalPinId`, `AnalogPinId` enums) |
| [LCDScreen](examples/Version4/LCDScreen/LCDScreen.ino) | TFT display basics (text, shapes, tick counter) |
| [LED](examples/Version4/LED/LED.ino) | on-board RGB LED control (cycles through all 8 colors) |
| [Motor](examples/Version4/Motor/Motor.ino) | Motor test & configuration (mapping, direction, speed) |
| [Movement](examples/Version4/Movement/Movement.ino) | Omnidirectional movement (byAngle, withCorr, rotation, coordinate calibration) |
| [MoveSquare](examples/Version4/MoveSquare/MoveSquare.ino) | Square-path movement with compass correction on/off + rotation |
| [OutOfBound](examples/Version4/OutOfBound/OutOfBound.ino) | Square movement with white line detection |
| [PS2](examples/Version4/PS2/PS2.ino) | PS2 controller basic reading (button states, joystick angle + strength) |
| [PS2Remote](examples/Version4/PS2Remote/PS2Remote.ino) | PS2 controller remote control (joystick-driven movement) |
| [ScreenColorSensor](examples/Version4/ScreenColorSensor/ScreenColorSensor.ino) | Screen + color sensor integration (color name, RGB display) |
| [ScreenCompass](examples/Version4/ScreenCompass/ScreenCompass.ino) | Screen + compass integration (heading display + pointer) |
| [ScreenCompoundEye](examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino) | Screen + compound eye integration (6×2 grid display + angle pointer) |
| [ScreenUltrasound](examples/Version4/ScreenUltrasound/ScreenUltrasound.ino) | Screen + ultrasonic integration (4 distances with labels) |
| [ScreenWhiteLine](examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino) | Screen + color sensor white line detection (HSL + baseline display) |
| [Ultrasound](examples/Version4/Ultrasound/Ultrasound.ino) | Ultrasound sensor (configuration, enable/disable, distance reading) |

---

## Hardware Configuration

### Default Pin Mapping (Arduino Mega)

| Function | Pin(s) | Description |
|----------|--------|-------------|
| TFT CS | 0 | TFT chip select |
| TFT DC | 53 | TFT data/command |
| TFT RST | 50 | TFT reset |
| TFT SCK | 52 | TFT SPI clock |
| TFT MOSI | 51 | TFT SPI data |
| Motor IN1 | 9, 7, 5, 3 | Motor channel 1 (PWM) |
| Motor IN2 | 8, 6, 4, 2 | Motor channel 2 (PWM) |
| Ultrasound Trig | 49, 48, 47, 46 | U1–U4 trigger pins |
| Ultrasound Echo | A15, A14, A13, A12 | U1–U4 echo pins (PCINT) |
| Button | 22, 23, 24, 25 | Button1–Button4 (INPUT_PULLUP) |
| LED RGB | 26, 28, 27 | Implementation pins R/G/B; header channel order is 0=Blue, 1=Green, 2=Red |
| SW I2C (×8) | 29–44 | SCL=30/32/34/36/38/40/42/44, SDA=29/31/33/35/37/39/41/43 |
| Servo/PWM | 10–13 | S1–S4 |
| Digital | 56–61 | D1_P–D6_P (`D6_P=A2(56)` … `D1_P=A7(61)`) |
| Analog | 62–65 | A1_P–A4_P (`A1_P=A11(65)` … `A4_P=A8(62)`) |

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

### 🟡 Not Yet Implemented

| Feature | Location | Status |
|---------|----------|--------|
| `Movement::outBoundPrevent()` / `correctedMove()` | `Movement.cpp` | Stubs returning zeroed speeds — out-of-bounds prevention not implemented |
| `bluetoothRemote()` | `PeanutKingSoccerV4.cpp` | Partial — connection check + handling only; full remote strategy not implemented |

### 🟢 Minor Issues

- `pwmPin[4]` declared in `PeanutKingSoccerV4.h` but never used
- Private member naming is inconsistent across modules (`_` prefix vs no prefix)
- `Compass::converter` and `Movement::converter`/`motorPID` are intentionally public, so users can tune/rotate the coordinate system directly (no high-level wrapper)

## Version History
