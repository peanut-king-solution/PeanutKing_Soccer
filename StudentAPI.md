# PeanutKingSoccerV4 Student API Reference

## Contents

- [PeanutKingSoccerV4 Student API Reference](#peanutkingsoccerv4-student-api-reference)
  - [Contents](#contents)
  - [Robot - PeanutKingSoccerV4](#robot---peanutkingsoccerv4)
    - [Overview](#overview)
    - [Public Sensor Data](#public-sensor-data)
    - [Data Fetch](#data-fetch)
    - [Common Positions Type (SensorPos)](#common-positions-type-sensorpos)
  - [Coordinate System](#coordinate-system)
    - [Functions of Coordinate System](#functions-of-coordinate-system)
      - [coordinateReset](#coordinatereset)
      - [coordinateRotate](#coordinaterotate)
      - [coordinateFlip](#coordinateflip)
    - [Example Usage of Coordinate System](#example-usage-of-coordinate-system)
  - [Motor - Single motor control](#motor---single-motor-control)
    - [Available Ports of Motor](#available-ports-of-motor)
    - [Physical Positions of Motor](#physical-positions-of-motor)
    - [Functions of Motor control](#functions-of-motor-control)
      - [motorConfiguration](#motorconfiguration)
      - [motorFlipDirection](#motorflipdirection)
      - [motorSetSpeed](#motorsetspeed)
      - [motorStop](#motorstop)
      - [motorStopAll](#motorstopall)
    - [Example Usage of Motor Control](#example-usage-of-motor-control)
    - [Motor Related Example ino](#motor-related-example-ino)
  - [Movement - Robot advanced movement control](#movement---robot-advanced-movement-control)
    - [Movement Default Coordinate System](#movement-default-coordinate-system)
    - [Parameters of movement mode switch](#parameters-of-movement-mode-switch)
    - [Functions of Movement control](#functions-of-movement-control)
      - [move](#move)
    - [Example Usage of Movement Control](#example-usage-of-movement-control)
    - [Movement Related Example ino](#movement-related-example-ino)
  - [Color Sensor - Color sensor data reading](#color-sensor---color-sensor-data-reading)
    - [Available Ports of Color Sensor](#available-ports-of-color-sensor)
    - [Physical Positions of Color Sensor](#physical-positions-of-color-sensor)
    - [Data Structures of Color Sensor](#data-structures-of-color-sensor)
      - [RGBC](#rgbc)
      - [RGB](#rgb)
      - [HSL](#hsl)
    - [Functions of Color Sensor](#functions-of-color-sensor)
      - [colorSensorConfiguration](#colorsensorconfiguration)
      - [colorSensorReadRGBC](#colorsensorreadrgbc)
      - [colorSensorReadRGB](#colorsensorreadrgb)
      - [colorSensorReadHSL](#colorsensorreadhsl)
      - [isWhiteLine](#iswhiteline)
      - [colorSensorCalBaseline](#colorsensorcalbaseline)
      - [colorSensorGetBaseline](#colorsensorgetbaseline)
    - [Example Usage of Color Sensor](#example-usage-of-color-sensor)
    - [Color Sensor Related Examples ino](#color-sensor-related-examples-ino)
  - [CompoundEye — Infrared Ray Compound Eye](#compoundeye--infrared-ray-compound-eye)
    - [Port Connection of CompoundEye Module](#port-connection-of-compoundeye-module)
    - [CompoundEye layout](#compoundeye-layout)
    - [Functions of CompoundEye](#functions-of-compoundeye)
      - [compoundEyeReadAll](#compoundeyereadall)
      - [compoundEyeValueRead](#compoundeyevalueread)
      - [compoundMaxEyeRead](#compoundmaxeyeread)
      - [compoundMaxEyeValueRead](#compoundmaxeyevalueread)
      - [compoundEyeAngleRead](#compoundeyeangleread)
      - [compoundEyeModeRead](#compoundeyemoderead)
    - [Example Usage of CompoundEye](#example-usage-of-compoundeye)
    - [CompoundEye Related Example ino](#compoundeye-related-example-ino)
  - [Button — Button Control](#button--button-control)
    - [Available Buttons](#available-buttons)
    - [Button State](#button-state)
    - [Functions of Button control](#functions-of-button-control)
      - [buttonUpdate](#buttonupdate)
      - [buttonStateRead](#buttonstateread)
    - [Example Usage of Button Control](#example-usage-of-button-control)
    - [Button Related Example ino](#button-related-example-ino)
  - [Led — On board RGB LED Control](#led--on-board-rgb-led-control)
    - [Available LED](#available-led)
    - [LED Color Type](#led-color-type)
    - [Functions of LED control](#functions-of-led-control)
      - [onBoardLedSet](#onboardledset)
    - [Example Usage of LED Control](#example-usage-of-led-control)
    - [LED Related Example ino](#led-related-example-ino)
  - [Ultrasound — Ultrasonic Sensor](#ultrasound--ultrasonic-sensor)
    - [Available Ports of Ultrasound Sensor](#available-ports-of-ultrasound-sensor)
    - [Physical Positions of Ultrasound Sensor](#physical-positions-of-ultrasound-sensor)
    - [Functions of Ultrasound Sensor](#functions-of-ultrasound-sensor)
      - [ultrasoundConfiguration](#ultrasoundconfiguration)
      - [ultrasoundGetDist](#ultrasoundgetdist)
      - [Example Usage of Ultrasound Sensor](#example-usage-of-ultrasound-sensor)
  - [Compass — Compass \& IMU](#compass--compass--imu)
    - [Compass Module Connection](#compass-module-connection)
    - [Coordinate System of Compass](#coordinate-system-of-compass)
    - [Functions of Compass reading](#functions-of-compass-reading)
      - [compassRead](#compassread)
      - [compassReadRawAccel](#compassreadrawaccel)
      - [compassReadRawGyro](#compassreadrawgyro)
      - [compassReadRawMag](#compassreadrawmag)
    - [Example Usage of Compass](#example-usage-of-compass)
    - [Compass Related Example ino](#compass-related-example-ino)
  - [LCD Screen - TFT module](#lcd-screen---tft-module)
    - [TFT Display Color Types](#tft-display-color-types)
    - [Basic Functions of TFT Display](#basic-functions-of-tft-display)
      - [screenSetTextColor](#screensettextcolor)
      - [screenSetTextSize](#screensettextsize)
      - [screenPrintText / screenPrintNumber](#screenprinttext--screenprintnumber)
      - [screenClear](#screenclear)
      - [screenDrawAnglePointer](#screendrawanglepointer)
    - [Example Usage of TFT Display](#example-usage-of-tft-display)
    - [TFT Related Example ino](#tft-related-example-ino)
  - [PS2 Controller](#ps2-controller)
    - [PS2 Controller Connection](#ps2-controller-connection)
    - [PS2 Button Types](#ps2-button-types)
    - [PS2 Joystick Types](#ps2-joystick-types)
    - [PS2 Button State](#ps2-button-state)
    - [PS2 Data Structures](#ps2-data-structures)
      - [PS2JoystickData](#ps2joystickdata)
    - [Functions of PS2 Controller](#functions-of-ps2-controller)
      - [ps2Init](#ps2init)
      - [ps2Update](#ps2update)
      - [ps2ButtonStateRead](#ps2buttonstateread)
      - [ps2JoystickRead](#ps2joystickread)
      - [ps2SetVibration](#ps2setvibration)
    - [Example Usage of PS2 Controller](#example-usage-of-ps2-controller)
    - [PS2 Related Example ino](#ps2-related-example-ino)
  - [Bluetooth Module](#bluetooth-module)

## Robot - PeanutKingSoccerV4

### Overview

**Initialization**: Before using any module, you must call `init()` in the `setup()` function.

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}
```

### Public Sensor Data

| Type | Name | Size | Description |
| ---- | ---- | ---- | ----------- |
| `RGBC` array | `colorRGBC` | 4 | Color sensor RGBC raw data (0=Front, 1=Right, 2=Back, 3=Left) |
| `RGB` array | `colorRGB` | 4 | Color sensor RGB data (0=Front, 1=Right, 2=Back, 3=Left) |
| `HSL` array | `colorHSL` | 4 | Color sensor HSL data (0=Front, 1=Right, 2=Back, 3=Left) |
| `bool` array | `isWhite` | 4 | White line detection status (0=Front, 1=Right, 2=Back, 3=Left), `true` if white line is detected |
| `uint8_t` array | `eyes` | 12 | Compound eye (Eye0~Eye11) values |
| `uint16_t` | `irAngle` | 1 | Compound eye IR angle (0-360°) |
| `uint8_t` | `maxEye` | 1 | Compound eye max eye index ((Eye0~Eye11)) |
| `uint8_t` | `maxEyeVal` | 1 | Compound eye max eye value |
| `uint16_t` array | `distances` | 4 | Ultrasound distances in mm (default: 0=Front, 1=Right, 2=Back, 3=Left) |
| `uint16_t` | `heading` | 1 | Compass heading (0-360°)

> **Note**: If the type of the public sensor data is an array, add `*` after the variable name to get the pointer of the array. For example, `uint8_t* eyeValues = robot.eyes;` will get the pointer of the compound eye values array.

### Data Fetch

**Description**: `dataFetch()` polls all four logical sensor positions and updates the public arrays, so you can access sensor data directly from the variables without calling individual read functions. It fills the 4 color-sensor slots (Front/Right/Back/Left), the 12 compound-eye values, the 4 ultrasound distances, and the compass heading. Disabled or unavailable sensors return their documented default/error values.

```cpp
void dataFetch(void);
```

**Usage**:

```cpp
void loop() {
  // Read all sensors at once
  robot.dataFetch();

  // Access color sensor data
  RGBC frontRGBC = robot.colorRGBC[0];  // Front sensor RGBC raw
  uint32_t frontRedRaw = frontRGBC.r;   // Front sensor raw red
  int frontRed = robot.colorRGB[0].r;   // Front sensor RGB
  int frontHue = robot.colorHSL[0].h;   // Front sensor HSL
  bool frontWhite = robot.isWhite[0];   // Front white line detection

  // Access compound eye data
  uint8_t* eyeValues = robot.eyes;  // All 12 eye values
  int irAngle = robot.irAngle;      // IR angle
  int maxEyeIdx = robot.maxEye;     // Max eye index
  int maxEyeVal = robot.maxEyeVal;  // Max eye value

  // Access ultrasound data
  int frontDist = robot.distances[0];  // Front distance
  int rightDist = robot.distances[1];  // Right distance
  int backDist = robot.distances[2];   // Back distance
  int leftDist = robot.distances[3];   // Left distance

  // Access compass heading
  int heading = robot.heading;  // Compass heading
}
```

### Common Positions Type (SensorPos)

**Description**: The `SensorPos` type is used to represent the physical positions of various sensors on the robot. It allows you to specify which sensor you want to interact with based on its location on the robot.

| Type | Position | Description |
| ---- | -------- | ----------- |
| `SensorPos` | `Front` | Front sensor position |
| `SensorPos` | `Right` | Right sensor position |
| `SensorPos` | `Back` | Back sensor position |
| `SensorPos` | `Left` | Left sensor position |

```
 (Robot's head)
     Front
       ↑
Left ←   → Right
       ↓
      Back
```

Now the `SensorPos` type is used in the following modules:

1. Color Sensor (see [Physical Positions of Color Sensor](#physical-positions-of-color-sensor))
2. Ultrasound Sensor (see [Physical Positions of Ultrasound Sensor](#physical-positions-of-ultrasound-sensor))

> **Note**: The `SensorPos` type is not used for motor control. For motor control, use the `MotorPos` type instead. (see [Physical Positions of Motor](#physical-positions-of-motor))


## Coordinate System

Generic template functions for adjusting the coordinate system of any module that has a `Converter` member. Supported modules: `movement`, `compass`, `compoundEye`.

### Functions of Coordinate System

#### coordinateReset

```cpp
template<typename T>
void coordinateReset(T& module);
```

**Description**: Resets the coordinate system of the given module to its default state.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `T&` | `module` | Reference to a module with a `converter` member (e.g. `robot.movement`, `robot.compass`, `robot.compoundEye`) |

#### coordinateRotate

```cpp
template<typename T>
void coordinateRotate(T& module, uint16_t angle, RotationDir dir = CW);
```

**Description**: Rotates the coordinate system of the given module by a specified non-negative angle. The rotation direction is `CW` (clockwise) by default; pass `dir = CCW` for counter-clockwise rotation.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `T&` | `module` | Reference to a module with a `converter` member |
| `uint16_t` | `angle` | The non-negative rotation angle in degrees. |
| `RotationDir` | `dir` | Rotation direction: `CW` (default) or `CCW` |

> **Example**: After `coordinateRotate(robot.movement, 90, CCW)`, the angles will be rotated as follows:
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

#### coordinateFlip

```cpp
template<typename T>
void coordinateFlip(T& module);
```

**Description**: Flips the direction of the coordinate system of the given module (`clockwise -> counter-clockwise or vice versa`).

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `T&` | `module` | Reference to a module with a `converter` member |

### Example Usage of Coordinate System

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();

  // Movement coordinate system
  robot.coordinateReset(robot.movement);
  robot.coordinateRotate(robot.movement, 90, CW);
  robot.coordinateFlip(robot.movement);

  // Compass coordinate system
  robot.coordinateReset(robot.compass);
  robot.coordinateRotate(robot.compass, 90, CCW);

  // Compound eye coordinate system
  robot.coordinateReset(robot.compoundEye);
  robot.coordinateFlip(robot.compoundEye);
}
```

## Motor - Single motor control

**Description**: All methods operate motors based on **physical positions**, internally mapped to physical ports.

### Available Ports of Motor

| Type | Port | Default position |
| ---- | ---- | ----------- |
| `MotorId` | `M1` | Left front wheel |
| `MotorId` | `M2` | Right front wheel |
| `MotorId` | `M3` | Right back wheel |
| `MotorId` | `M4` | Left back wheel |

### Physical Positions of Motor

| Type | Position | Description | Default port |
| ---- | -------- | ----------- | ------------ |
| `MotorPos` | `LeftFront` | Left front wheel | `M1` |
| `MotorPos` | `RightFront` | Right front wheel | `M2` |
| `MotorPos` | `RightBack` | Right back wheel | `M3` |
| `MotorPos` | `LeftBack` | Left back wheel | `M4` |

```
LeftFront (M1)  RightFront (M2)
        \           /
       center of robot
        /           \
LeftBack (M4)   RightBack (M3)
```

> **Note**: The motors are not using the `SensorPos` type (see [Common Positions Type](#common-positions-type-sensorpos)), because the motors are not sensors. The `MotorPos` type is used instead to represent the physical positions of the motors.
>
> **Important**: You should run the Motor example (see [Motor Related Example ino](#motor-related-example-ino)) to check the motor configuration and flip direction before using the movement functions. If the motors are not configured correctly, the robot may not move as expected.

### Functions of Motor control

#### motorConfiguration

```cpp
void motorConfiguration(MotorId LeftFront, MotorId RightFront, MotorId RightBack, MotorId LeftBack);
```

**Description**: Assigns which motor port (`M1`–`M4`) controls which wheel position.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `MotorId` | `LeftFront` | Port for left front wheel |
| `MotorId` | `RightFront` | Port for right front wheel |
| `MotorId` | `RightBack` | Port for right back wheel |
| `MotorId` | `LeftBack` | Port for left back wheel |

#### motorFlipDirection

```cpp
void motorFlipDirection(MotorPos pos, bool flip = true);
```

**Description**: Flips the rotational direction of a specific motor based on its physical position.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `MotorPos` | `pos` | The physical position of the motor |
| `bool` | `flip` | `true` to flip the rotational direction, `false` for normal direction (default: `true` if not specified) |

#### motorSetSpeed

```cpp
void motorSetSpeed(MotorPos pos, int16_t speed);
```

**Description**: Sets the speed of a specific motor based on its physical position.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `MotorPos` | `pos` | The physical position of the motor |
| `int16_t` | `speed` | The speed to set the motor to (range: -255 to 255) |

> **Default direction rules:**
>
> - Positive motor speed (`+`) → Counter-clockwise rotation (when watching the wheel facing us, not the motor facing us)
>   - us -> wheel -> motor
> - All motors positive (`+`) → Robot rotates clockwise (CW)

#### motorStop

```cpp
void motorStop(MotorPos pos);
```

**Description**: Stops a specific motor based on its physical position (brake mode).

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `MotorPos` | `pos` | The physical position of the motor |

#### motorStopAll

```cpp
void motorStopAll(void);
```

**Description**: Stops all motors (brake mode).

### Example Usage of Motor Control

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  // Initialize all modules
  robot.init();
  // In this example, we configure the motors to their default positions
  //  - left front wheel is connnected to M1
  //  - right front wheel is connnected to M2
  //  - right back wheel is connnected to M3
  //  - left back wheel is connnected to M4
  robot.motorConfiguration(M1, M2, M3, M4);
  // Optionally, flip the rotational direction of the right front motor
  robot.motorFlipDirection(RightFront, true);
}

void loop() {
  // Set motor speed (positive = CCW, negative = CW)
  robot.motorSetSpeed(LeftFront, 150);   // Left Front CCW
  robot.motorSetSpeed(RightFront, 150);  // Right Front CW — direction flipped
  robot.motorSetSpeed(RightBack, -100);  // Right Back CW
  robot.motorSetSpeed(LeftBack, -100);   // Left Back CW
  delay(2000);

  // Stop Left Front motor (brake mode)
  robot.motorStop(LeftFront); 
  delay(1000);
  
  // Stop all motors (brake mode)
  robot.motorStopAll(); 
  delay(1000);
}
```

### Motor Related Example ino

[examples/Version4/Motor/Motor.ino](examples/Version4/Motor/Motor.ino)

[examples/Version4/CompassCar/CompassCar.ino (required Compass)](examples/Version4/CompassCar/CompassCar.ino)

## Movement - Robot advanced movement control

**Description**: The movement methods provide advanced movement control for the robot, allowing for auto correction of movement based on the robot's orientation and position. It utilizes the motor control functions to achieve precise movements.

### Movement Default Coordinate System

**Description**: When all the motors port **(check the M1~M4 and see [Physical Positions of Motor](#physical-positions-of-motor))** is connected to the default positions, the robot's movement coordinate system is defined as follows:

```
  robot's front
(M1)   0°   (M2)
  315° ↑  45°
     \ | /
270°←-    -→90°
     / | \
  215° ↓  135°
(M4)  180°  (M3)
```

> **Note**: The movement coordinate system is based on the robot's front direction, which is defined as 0 degrees. The angles increase clockwise, with 90 degrees to the right, 180 degrees backward, and 270 degrees to the left.
>
> **Note**: If you find any of the motors is not working as expected, please check the motor configuration, flip direction setting in motor part (see function described in [motorConfiguration](#motorconfiguration) and [motorFlipDirection](#motorflipdirection)), or adjust the movement coordinates via the generic coordinate system wrappers (see [Coordinate System](#coordinate-system)).

### Parameters of movement mode switch

| Type | Parameter | Description | Default |
| ---- | --------- | ----------- | ------- |
| `bool` | `compassCorrectEnabled` | Enables or disables **compass correction for movement**. When enabled, the robot will always try to facing the default direction (0 degrees) | `true` |
| `bool` | `outBoundPreventEnabled` | Enables or disables **out-of-bounds prevention**. When enabled, the robot will move to opposite side if it detects that it is about to go out of bounds | `false` |

> **Note**: `outBoundPreventEnabled` is not implemented yet. It is planned for future releases.

### Functions of Movement control

#### move

```cpp
void move(float mAngle, float mSpeed, float rotate = 0);
```

**Description**: Moves the robot in a specified direction with a specified speed and optional rotation.

There are different movement behaviors based on the parameters:

1. **Both `compassCorrectEnabled` and `outBoundPreventEnabled` are enabled**:
   - The robot will move in the specified direction while correcting its orientation to face the default direction (0 degrees) and preventing out-of-bounds movement.
2. **Only `compassCorrectEnabled` is enabled**:
   - The robot will move in the specified direction while correcting its orientation to face the default direction (0 degrees).
3. **Only `outBoundPreventEnabled` is enabled**:
   - The robot will move in the specified direction while preventing out-of-bounds movement.
4. **Both `compassCorrectEnabled` and `outBoundPreventEnabled` are disabled**:
   - The robot will move in the specified direction without any correction or prevention.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `float` | `mAngle` | The angle of movement in degrees (0-360). 0 degrees is forward, 90 degrees is right, 180 degrees is backward, and 270 degrees is left. |
| `float` | `mSpeed` | The speed of movement (0-255). |
| `float` | `rotate` | The rotation speed (-255 to +255). Positive values rotate clockwise, negative values rotate counter-clockwise. Default is 0 (no rotation). `rotate` is applied only when both correction features are disabled. When compass correction is enabled, rotation is automatically controlled by the compass-correction algorithm and the supplied `rotate` value is ignored. |

> **Coordinate system wrappers** — see [Coordinate System](#coordinate-system) below.

### Example Usage of Movement Control

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
  robot.outBoundPreventEnabled = false; // Disable out-of-bounds prevention

  // Adjust coordinate system (generic template wrappers)
  robot.coordinateReset(robot.movement);              // Reset to default coordinate system
  robot.coordinateRotate(robot.movement, 90);         // Rotate coordinate system 90° clockwise (default CW)
  robot.coordinateRotate(robot.movement, 90, CCW);    // Rotate coordinate system 90° counter-clockwise
  robot.coordinateFlip(robot.movement);               // Flip 180 degrees (CW<->CCW)
}

void loop() {
  // Enable compass correction to maintain orientation
  robot.compassCorrectEnabled = true;
  // move forward with compass correction
  robot.move(0, 100);  
  delay(2000);
  // move rightward with compass correction
  robot.move(90, 100); 
  delay(2000);

  // Disable compass correction to allow free rotation
  robot.compassCorrectEnabled = false;
  // Rotate clockwise
  robot.move(0, 0, 100);  
  delay(2000);

  // Stop all motors
  robot.motorStopAll();   
  delay(1000);
}
```

### Movement Related Example ino

[examples/Version4/Movement/Movement.ino](examples/Version4/Movement/Movement.ino)

[examples/Version4/MoveSquare/MoveSquare.ino (required Compass)](examples/Version4/MoveSquare/MoveSquare.ino)

[examples/Version4/OutOfBound/OutOfBound.ino (required Color Sensor)](examples/Version4/OutOfBound/OutOfBound.ino)

## Color Sensor - Color sensor data reading

**Description**: There are functions to read color data from the color sensor module. It allows you to detect whiteline and obtain RGB, HSL and raw RGBC values.

### Available Ports of Color Sensor

**Description**: The color sensor module can be connected to the following ports. The default configuration is as follows:

You can change the port configuration using the `colorSensorConfiguration` function when you connected the sensor to a different port.

| Type | Port | Default position |
| ---- | ---- | ----------- |
| `ColorSensorId` | `CL1` | Front color sensor |
| `ColorSensorId` | `CL2` | Right color sensor |
| `ColorSensorId` | `CL3` | Back color sensor |
| `ColorSensorId` | `CL4` | Left color sensor |
| `ColorSensorId` | `CL5-CL8` | Extra sensors (disabled by default) |

### Physical Positions of Color Sensor

| Type | Position | Description | Default port |
| ---- | -------- | ----------- | ------------ |
| `SensorPos` | `Front` | Front color sensor | `CL1` |
| `SensorPos` | `Right` | Right color sensor | `CL2` |
| `SensorPos` | `Back` | Back color sensor | `CL3` |
| `SensorPos` | `Left` | Left color sensor | `CL4` |

```
     (CL1) Front
             ↑
             |
(CL4) Left ←   → Right (CL2)
             |
             ↓
           Back (CL3)
```

> **Note**: You can check more details about `SensorPos` type in [Common Positions Type (SensorPos)](#common-positions-type-sensorpos).

### Data Structures of Color Sensor

#### RGBC

| Name | Description |
|------|-------------|
| `r`  | Red raw value (0~65535) |
| `g`  | Green raw value (0~65535) |
| `b`  | Blue raw value (0~65535) |
| `c`  | Clear raw value (0~65535) |

#### RGB

| Name | Description |
|------|-------------|
| `r`  | Red value (0~255) |
| `g`  | Green value (0~255) |
| `b`  | Blue value (0~255) |

#### HSL

| Name | Description |
|------|-------------|
| `h`  | Hue value (0~360) |
| `s`  | Saturation value (0~100) |
| `l`  | Lightness value (0~100) |

### Functions of Color Sensor

#### colorSensorConfiguration

```cpp
void colorSensorConfiguration(ColorSensorId Front, ColorSensorId Right, ColorSensorId Back, ColorSensorId Left);
```

**Description**: Assigns which color sensor port (`CL1`–`CL8`) corresponds to which physical position on the robot.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `ColorSensorId` | `Front` | Port for front color sensor |
| `ColorSensorId` | `Right` | Port for right color sensor |
| `ColorSensorId` | `Back` | Port for back color sensor |
| `ColorSensorId` | `Left` | Port for left color sensor |

#### colorSensorReadRGBC

```cpp
RGBC colorSensorReadRGBC(SensorPos pos); 
```

**Description**: Reads the raw RGBC values from the color sensor at the specified position.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the color sensor to read from (Front, Right, Back, Left) |

**Usage**: Access RGBC returned values by the following code:

```cpp
RGBC rgbc = robot.colorSensorReadRGBC(Front);
uint32_t r = rgbc.r; // Red raw value (0~65535)
uint32_t g = rgbc.g; // Green raw value (0~65535)
uint32_t b = rgbc.b; // Blue raw value (0~65535)
uint32_t c = rgbc.c; // Clear raw value (0~65535)
```

#### colorSensorReadRGB

```cpp
RGB colorSensorReadRGB(SensorPos pos);
```

**Description**: Reads the RGB values from the color sensor at the specified position.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the color sensor to read from (Front, Right, Back, Left) |

**Usage**: Access RGB returned values by the following code:

```cpp
RGB rgb = robot.colorSensorReadRGB(Front);
int red = rgb.r; // Red value (0~255)
int green = rgb.g; // Green value (0~255)
int blue = rgb.b; // Blue value (0~255)
```

#### colorSensorReadHSL

```cpp
HSL colorSensorReadHSL(SensorPos pos);
```

**Description**: Reads the HSL values from the color sensor at the specified position.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the color sensor to read from (Front, Right, Back, Left) |

**Usage**: Access HSL returned values by the following code:

```cpp
HSL hsl = robot.colorSensorReadHSL(Front);
int hue = hsl.h; // Hue value (0~360)
int saturation = hsl.s; // Saturation value (0~100)
int lightness = hsl.l; // Lightness value (0~100)
```

#### isWhiteLine

```cpp
bool isWhiteLine(SensorPos pos);
```

**Description**: Determines if the color sensor at the specified position detects a white line.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the color sensor to read from (Front, Right, Back, Left) |

| Return | Description |
| ------ | ----------- |
| `true` | The color sensor detects a white line. |
| `false` | The color sensor does not detect a white line. |

The implementation uses `greenHue / 8` as the hue margin and checks hue only. During `robot.init()`, only currently enabled color sensors are calibrated (by default CL1–CL4).

#### colorSensorCalBaseline

```cpp
void colorSensorCalBaseline(SensorPos pos, uint8_t samples = 10);
```

**Description**: Calibrates the white line detection baseline by sampling the sensor values while the sensor is over a green surface. This should be called with the robot placed on the green field before using `isWhiteLine()`. By default, it is already called during initialization (`robot.init()`).

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the color sensor to calibrate (Front, Right, Back, Left) |
| `uint8_t` | `samples` | Number of samples to average for calibration (default: 10) |

#### colorSensorGetBaseline

```cpp
GreenBaseline colorSensorGetBaseline(SensorPos pos);
```

**Description**: Returns the calibrated baseline data for the specified sensor position. Check `baseline.calibrated` before using the values.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the color sensor (Front, Right, Back, Left) |

**`GreenBaseline` struct**:

| Type | Field | Description |
| ---- | ----- | ----------- |
| `uint16_t` | `greenHue` | Average green field hue |
| `uint8_t` | `greenLight` | Average green field lightness |
| `uint8_t` | `greenSat` | Average green field saturation |
| `bool` | `calibrated` | `true` if calibration has been completed |

**Usage**:

```cpp
// Calibrate all sensors on the green field
robot.colorSensorCalBaseline(Front);
robot.colorSensorCalBaseline(Right);
robot.colorSensorCalBaseline(Back);
robot.colorSensorCalBaseline(Left);

// Read baseline data
GreenBaseline baseline = robot.colorSensorGetBaseline(Front);
if (baseline.calibrated) {
  Serial.print("Green hue: ");
  Serial.println(baseline.greenHue);
}
```

### Example Usage of Color Sensor

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
  robot.colorSensorConfiguration(CL1, CL2, CL3, CL4);
}

void loop() {
  // Read RGB values from the front color sensor
  RGB frontRGB = robot.colorSensorReadRGB(Front);
  Serial.print("Front RGB: R=");
  Serial.print(frontRGB.r);
  Serial.print(", G=");
  Serial.print(frontRGB.g);
  Serial.print(", B=");
  Serial.println(frontRGB.b);

  // Read HSL values from the right color sensor
  HSL rightHSL = robot.colorSensorReadHSL(Right);
  Serial.print("Right HSL: H=");
  Serial.print(rightHSL.h);
  Serial.print(", S=");
  Serial.print(rightHSL.s);
  Serial.print(", L=");
  Serial.println(rightHSL.l);

  // Check if the back color sensor detects a white line
  bool isBackWhiteLine = robot.isWhiteLine(Back);
  if (isBackWhiteLine) {
    Serial.println("Back color sensor detects a white line.");
  } else {
    Serial.println("Back color sensor does not detect a white line.");
  }
}
```

### Color Sensor Related Examples ino

[examples/Version4/ColorSensor/ColorSensor.ino](examples/Version4/ColorSensor/ColorSensor.ino)

[examples/Version4/ScreenColorSensor/ScreenColorSensor.ino](examples/Version4/ScreenColorSensor/ScreenColorSensor.ino)

[examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino (required Color Sensor)](examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino)

[examples/Version4/OutOfBound/OutOfBound.ino](examples/Version4/OutOfBound/OutOfBound.ino)

## CompoundEye — Infrared Ray Compound Eye

**Description**: The CompoundEye module provides an interface to read data from the infrared compound eye sensor (see [CompoundEye Layout](#compoundeye-layout) below). It allows you to detect the distance and the angle of infrared light sources, which can be used for ir ball detection

### Port Connection of CompoundEye Module

**Description**: You should connect the CompoundEye module to any one i2c port in the pila shield.

### CompoundEye layout

```
               (front, 0°)
                    3
                 4     2
               5          1
(left, 270°) 6              0 (right, 90°)
               7         11
                 8    10
                    9
               (back, 180°)
```

| Type | Name | Description |
| ---- | ---- | ----------- |
| `EyeId`  | `Eye0~Eye11` | The 12 eyes of the compound eye sensor, numbered from 0 to 11. Each eye corresponds to a specific direction around the sensor, as shown in the layout above. |

### Functions of CompoundEye

#### compoundEyeReadAll

```cpp
uint8_t* compoundEyeReadAll(void);
```

**Description**: Reads the values from all 12 eyes of the compound eye sensor.

| Return | Description |
| ------ | ----------- |
| `uint8_t*` | A pointer to an array of 12 values, each representing the strength of infrared light detected by each eye (0-255). The values are ordered from Eye0 to Eye11. |

#### compoundEyeValueRead

```cpp
uint8_t compoundEyeValueRead(EyeId eyeIndex);
```

**Description**: Reads the value from a specific eye of the compound eye sensor.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `EyeId` | `eyeIndex` | The index of the eye to read from (0-11). |

| Return | Description |
| ------ | ----------- |
| `uint8_t` | The strength of infrared light (0-255) detected by the specified eye. |

#### compoundMaxEyeRead

```cpp
EyeId compoundMaxEyeRead(void);
```

**Description**: Returns the ID of the eye that detects the maximum strength of infrared light.

| Return | Description |
| ------ | ----------- |
| `EyeId` | The ID of the eye (0-11) that detects the maximum strength of infrared light |

#### compoundMaxEyeValueRead

```cpp
uint8_t compoundMaxEyeValueRead(void);
```

**Description**: Returns the maximum strength of infrared light detected by any eye.

| Return | Description |
| ------ | ----------- |
| `uint8_t` | The maximum strength of infrared light (0-255) detected by any eye. |

#### compoundEyeAngleRead

```cpp
uint16_t compoundEyeAngleRead(void);
```

**Description**: Reads the angle of the infrared light source detected by the compound eye sensor and returns the angle in degrees (0-360). The angle is calculated based on the position of the eyes and the strength of the infrared light detected by each eye. The raw sensor angle is automatically converted via the coordinate system (see [Coordinate System](#coordinate-system) for adjustment).

| Return | Description |
| ------ | ----------- |
| `uint16_t` | The angle of the infrared light source in degrees (0-360). |

#### compoundEyeModeRead

```cpp
uint8_t compoundEyeModeRead(void);
```

**Description**: Returns the detection mode of the compound eye sensor.

| Return | Description |
| ------ | ----------- |
| `uint8_t` | `0` = single IR source detected, `1` = dual IR sources detected |

> **Coordinate system wrappers** — see [Coordinate System](#coordinate-system) below.

### Example Usage of CompoundEye

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Read values from all eyes
  uint8_t* eyeValues = robot.compoundEyeReadAll();
  Serial.print("Compound Eye Values: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(eyeValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Read value from a specific eye (e.g., Eye0)
  uint8_t eye0Value = robot.compoundEyeValueRead(Eye0);
  Serial.print("Eye0 Value: ");
  Serial.println(eye0Value);

  // Get the eye with the maximum value
  EyeId maxEye = robot.compoundMaxEyeRead();
  uint8_t maxValue = robot.compoundMaxEyeValueRead();
  Serial.print("Max Eye ID: ");
  Serial.print(maxEye);
  Serial.print(", Max Value: ");
  Serial.println(maxValue);

  // Get the angle of the infrared light source
  uint16_t angle = robot.compoundEyeAngleRead();
  Serial.print("Infrared Light Source Angle: ");
  Serial.println(angle);
}
```

### CompoundEye Related Example ino

[examples/Version4/CompoundEye/CompoundEye.ino](examples/Version4/CompoundEye/CompoundEye.ino)

[examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino (required LCD Screen - TFT module)](examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino)

## Button — Button Control

**Description**: The Button methods provide an interface to read the state of buttons on the robot. It allows you to detect button presses, released, holding, etc., which can be used for user input or control.

### Available Buttons

| Type | Name | Description |
| ---- | ---- | ----------- |
| `ButtonId` | `Button1` | Button 1 (most left top) |
| `ButtonId` | `Button2` | Button 2 (below Button1) |
| `ButtonId` | `Button3` | Button 3 (below Button2) |
| `ButtonId` | `Button4` | Button 4 (below Button3) |

```
┌────────────────────────┐
|        Led
| Button1 <--
| Button2 <--
| Button3 <--
| Button4 <--
|
```

### Button State

| Type | Name | Description |
| ---- | ---- | ----------- |
| `ButtonState` | `ButtonIdle` | The button is idle |
| `ButtonState` | `ButtonPressed` | The button is pressed |
| `ButtonState` | `ButtonHolding` | The button is being held down |
| `ButtonState` | `ButtonReleased` | The button is released |

### Functions of Button control

#### buttonUpdate

```cpp
void buttonUpdate(void);
```

**Description**: Updates the button state machine. Call this **once per loop** before reading button states.

#### buttonStateRead

```cpp
ButtonState buttonStateRead(ButtonId btn);
```

**Description**: Reads the current state of a specific button, returning one of the `ButtonState` values. Must be called **after** `buttonUpdate()`.

### Example Usage of Button Control

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Update button state machine first
  robot.buttonUpdate();

  // Then read the state of Button2
  ButtonState button2State = robot.buttonStateRead(Button2);
  switch (button2State) {
  case ButtonIdle:
    break;
  case ButtonPressed:
    Serial.println("Button2 is pressed.");
    break;
  case ButtonHolding:
    Serial.println("Button2 is being held down.");
    break;
  case ButtonReleased:
    Serial.println("Button2 is released.");
    break;
  }
}
```

### Button Related Example ino

[examples/Version4/Button/Button.ino](examples/Version4/Button/Button.ino)

## Led — On board RGB LED Control

**Description**: The Led methods provide an interface to control the on-board RGB LED on the robot. It allows you to set the color (combination of Red, Green, Blue) of the LED, which can be used for visual feedback or status indication.

### Available LED

```
┌────────────────────────┐
|        Led <-
| Button1
| Button2
| Button3
| Button4
|
```

### LED Color Type

| Type | Name | Description |
| ---- | ---- | ----------- |
| `LEDColor` | `LEDOff` | turn off all channels |
| `LEDColor` | `LEDWhite` | turn on all channels |
| `LEDColor` | `LEDRed` | turn on red channel |
| `LEDColor` | `LEDGreen` | turn on green channel |
| `LEDColor` | `LEDBlue` | turn on blue channel |
| `LEDColor` | `LEDYellow` | turn on red and green channels |
| `LEDColor` | `LEDCyan` | turn on green and blue channels |
| `LEDColor` | `LEDPurple` | turn on red and blue channels |

### Functions of LED control

#### onBoardLedSet

```cpp
void onBoardLedSet(LEDColor color);
```

**Description**: Sets the color of the on-board RGB LED to the specified `LEDColor`.

### Example Usage of LED Control

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Set the on-board LED to red
  robot.onBoardLedSet(LEDRed);
  delay(1000);

  // Set the on-board LED to green
  robot.onBoardLedSet(LEDGreen);
  delay(1000);

  // Set the on-board LED to blue
  robot.onBoardLedSet(LEDBlue);
  delay(1000);

  // Turn off the on-board LED
  robot.onBoardLedSet(LEDOff);
  delay(1000);
}
```

### LED Related Example ino

[examples/Version4/LED/LED.ino](examples/Version4/LED/LED.ino)

## Ultrasound — Ultrasonic Sensor

**Description**: The Ultrasound methods provide an interface to read distance measurements from the ultrasonic sensor on the robot. It allows you to detect obstacles and measure distances, which can be used for navigation and collision avoidance.

### Available Ports of Ultrasound Sensor

| Type | Port | Default position |
| ---- | ---- | ---------------- |
| `UltrasoundId` | `U1` | Front |
| `UltrasoundId` | `U2` | Right |
| `UltrasoundId` | `U3` | Back |
| `UltrasoundId` | `U4` | Left |

### Physical Positions of Ultrasound Sensor

| Type | Position | Description | Default port |
| ---- | -------- | ----------- | ------------ |
| `SensorPos` | `Front` | Front ultrasonic sensor | `U1` |
| `SensorPos` | `Right` | Right ultrasonic sensor | `U2` |
| `SensorPos` | `Back` | Back ultrasonic sensor | `U3` |
| `SensorPos` | `Left` | Left ultrasonic sensor | `U4` |

```
     (U1) Front
             ↑
             |
(U4) Left ←     → Right (U2)
             |
             ↓
           Back (U3)
```

> **Note**: You can check more details about `SensorPos` type in [Common Positions Type (SensorPos)](#common-positions-type-sensorpos).
>
> **Note**: You are able to change the port configuration using the `ultrasoundConfiguration` function when you connected the sensor to a different port.

### Functions of Ultrasound Sensor

#### ultrasoundConfiguration

```cpp
void ultrasoundConfiguration(UltrasoundId Front, UltrasoundId Right, UltrasoundId Back, UltrasoundId Left);
```

**Description**: Assigns which ultrasonic sensor port (`U1`–`U4`) corresponds to which physical position on the robot.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `UltrasoundId` | `Front` | Port for front ultrasonic sensor |
| `UltrasoundId` | `Right` | Port for right ultrasonic sensor |
| `UltrasoundId` | `Back` | Port for back ultrasonic sensor |
| `UltrasoundId` | `Left` | Port for left ultrasonic sensor |

#### ultrasoundGetDist

```cpp
uint16_t ultrasoundGetDist(SensorPos pos);
```

`ultrasoundGetDist()` returns a measured distance in millimetres when available. Invalid or disabled positions return `65535`; rate-limited reads may return the cached distance.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `SensorPos` | `pos` | The physical position of the ultrasonic sensor to read from (Front, Left, Right, Back) |

#### Example Usage of Ultrasound Sensor

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Read distance from the front ultrasonic sensor
  uint16_t frontDistance = robot.ultrasoundGetDist(Front);
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.println(" mm");

  delay(1000);
}
```

## Compass — Compass & IMU

**Description**: The Compass methods provide an interface to read orientation and heading information (0-360 degrees) from the compass and the 9-axis IMU sensor (accelerometer, gyroscope, magnetometer) on the robot. It allows you to determine the robot's heading, orientation, and movement direction, which can be used for navigation and control.

### Compass Module Connection

**Description**: You should connect the Compass module to any one i2c port in the pila shield.

And you should place the compass module on the robot correctly, as shown in the following diagram:

```
(view from top)
                    Robot's front
                  ┌──────────────┐
                  │ O        ┌─>y│
                  │      /\  x   │
                  │     /  \     │
                  │ O          O │
                  └──────────────┘ (compass module)
```

### Coordinate System of Compass

**Description**: If you placed the compass module on the robot correctly (see [Compass Module Connection](#compass-module-connection)), the default coordinate system of the compass is defined as follows:

```
  robot's front
       0°
  315° ↑  45°
     \ | /
270°←-    -→90°
     / | \
  215° ↓  135°
      180°
```

### Functions of Compass reading

#### compassRead

```cpp
uint16_t compassRead(void);
```

**Description**: Reads the current heading of the robot from the compass sensor and returns the heading in degrees (0-360). The heading is measured clockwise from the robot's front direction (0 degrees) in default.

#### compassReadRawAccel

```cpp
int16_t* compassReadRawAccel(void);
```

**Description**: Reads the raw accelerometer data from the IMU sensor and returns a pointer to an array of three `int16_t` values representing the acceleration in the X, Y, and Z axes.

**Usage**: Access the accelerometer data by the following code:

```cpp
int16_t* accelData = robot.compassReadRawAccel();
int16_t x = accelData[0]; // X-axis acceleration
int16_t y = accelData[1]; // Y-axis acceleration
int16_t z = accelData[2]; // Z-axis acceleration
```

#### compassReadRawGyro

```cpp
int16_t* compassReadRawGyro(void);
```

**Usage**: Access the gyroscope data by the following code:

```cpp
int16_t* gyroData = robot.compassReadRawGyro();
int16_t x = gyroData[0]; // X-axis angular velocity
int16_t y = gyroData[1]; // Y-axis angular velocity
int16_t z = gyroData[2]; // Z-axis angular velocity
```

**Description**: Reads the raw gyroscope data from the IMU sensor and returns a pointer to an array of three `int16_t` values representing the angular velocity in the X, Y, and Z axes.

#### compassReadRawMag

```cpp
int16_t* compassReadRawMag(void);
```

**Usage**: Access the magnetometer data by the following code:

```cpp
int16_t* magData = robot.compassReadRawMag();
int16_t x = magData[0]; // X-axis magnetic field
int16_t y = magData[1]; // Y-axis magnetic field
int16_t z = magData[2]; // Z-axis magnetic field
```

**Description**: Reads the raw magnetometer data from the IMU sensor and returns a pointer to an array of three `int16_t` values representing the magnetic field in the X, Y, and Z axes.

> **Coordinate system wrappers** — see [Coordinate System](#coordinate-system) below.

### Example Usage of Compass

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
}

void loop() {
  // Read the compass heading
  uint16_t heading = robot.compassRead();
  Serial.print("Compass Heading: ");
  Serial.println(heading);

  // Read raw accelerometer data
  int16_t* accelData = robot.compassReadRawAccel();
  Serial.print("Accelerometer: X=");
  Serial.print(accelData[0]);
  Serial.print(", Y=");
  Serial.print(accelData[1]);
  Serial.print(", Z=");
  Serial.println(accelData[2]);

  // Read raw gyroscope data
  int16_t* gyroData = robot.compassReadRawGyro();
  Serial.print("Gyroscope: X=");
  Serial.print(gyroData[0]);
  Serial.print(", Y=");
  Serial.print(gyroData[1]);
  Serial.print(", Z=");
  Serial.println(gyroData[2]);

  // Read raw magnetometer data
  int16_t* magData = robot.compassReadRawMag();
  Serial.print("Magnetometer: X=");
  Serial.print(magData[0]);
  Serial.print(", Y=");
  Serial.print(magData[1]);
  Serial.print(", Z=");
  Serial.println(magData[2]);

  delay(1000);
}
```

### Compass Related Example ino

[examples/Version4/Compass/Compass.ino](examples/Version4/Compass/Compass.ino)

[examples/Version4/CompassCar/CompassCar.ino](examples/Version4/CompassCar/CompassCar.ino)

[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)

## LCD Screen - TFT module

**Description**: The TFT module provides an interface to control the `128*160` pixels TFT LCD screen. It allows you to display text, numbers, and simple graphics, which can be used for debugging and user feedback.

### TFT Display Color Types

**Important**: You must use the following color types when setting text color or drawing on the TFT display.

| Name | Description |
| ---- | ----------- |
| `ST7735_BLACK` | Black color |
| `ST7735_RED` | Red color |
| `ST7735_BLUE` | Blue color |
| `ST7735_GREEN` | Green color |
| `ST7735_YELLOW` | Yellow color |
| `ST7735_MAGENTA` | Magenta color |
| `ST7735_CYAN` | Cyan color |
| `ST7735_WHITE` | White color |

### Basic Functions of TFT Display

#### screenSetTextColor

```cpp
void screenSetTextColor(uint16_t color);
void screenSetTextColor(uint16_t fg, uint16_t bg);
```

**Description**: Sets the text color and optionally the background color for subsequent text output.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `uint16_t` | `color` | Text color (e.g., `ST7735_WHITE`, `ST7735_RED`) |
| `uint16_t` | `fg` | Foreground (text) color |
| `uint16_t` | `bg` | Background color |

#### screenSetTextSize

```cpp
void screenSetTextSize(uint8_t size);
```

**Description**: Sets the text size for subsequent text output.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `uint8_t` | `size` | Text size (1 = smallest, larger values = bigger text) |

#### screenPrintText / screenPrintNumber

```cpp
void screenPrintText(uint8_t col, uint8_t row, const char* string);
void screenPrintNumber(uint8_t col, uint8_t row, int16_t number);
```

**Description**: Prints a string or number at the specified position on the screen.

The 128*160 pixel TFT screen is divided into about `21 columns (0-20)` and `16 rows (0-15)` of character positions, with each character being approximately `6 pixels wide` and `10 pixels tall`.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `uint8_t` | `col` | Column position (character-based, 0 = leftmost) |
| `uint8_t` | `row` | Row position (character-based, 0 = topmost) |
| `const char*` | `string` | String to display |
| `int16_t` | `number` | Number to display |

#### screenClear

```cpp
void screenClear(void);
```

**Description**: Clears the entire screen (fills with black).

#### screenDrawAnglePointer

```cpp
void screenDrawAnglePointer(int x, int y, int radius, uint16_t angle, uint16_t arrowColor = ST7735_MAGENTA);
```

**Description**: Draws an angle pointer (arrow) at the specified position, pointing in the given direction.

> **Note**: This method changes the text size and color settings of the TFT display, so you may need to reset them before subsequent text output.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `int` | `x` | X coordinate of the center (0-127) |
| `int` | `y` | Y coordinate of the center (0-159) |
| `int` | `radius` | Radius of the pointer circle |
| `uint16_t` | `angle` | Angle in degrees (0-360, 0 = up) |
| `uint16_t` | `color` | Color of the pointer (default: `ST7735_MAGENTA`) |

### Example Usage of TFT Display

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
  robot.screenSetTextColor(ST7735_WHITE);
  robot.screenSetTextSize(2);
  robot.screenClear();
}

int angle = 0;

void loop() {
  // Display a string
  robot.screenPrintText(0, 0, "Hello!");

  // Display a number
  robot.screenPrintNumber(0, 2, 12345);

  // update the angle for the pointer
  angle += 1;
  if (angle >= 360) {
    angle = 0;
  }
  // Draw an angle pointer
  int x = 64; // center x
  int y = 80; // center y
  int radius = 30; // pointer radius
  robot.screenDrawAnglePointer(x, y, radius, angle, ST7735_RED);

  delay(500);
}
```

### TFT Related Example ino

[examples/Version4/LCDScreen/LCDScreen.ino](examples/Version4/LCDScreen/LCDScreen.ino)

[examples/Version4/ScreenColorSensor/ScreenColorSensor.ino](examples/Version4/ScreenColorSensor/ScreenColorSensor.ino)

[examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino](examples/Version4/ScreenWhiteLine/ScreenWhiteLine.ino)

[examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino](examples/Version4/ScreenCompoundEye/ScreenCompoundEye.ino)

[examples/Version4/ScreenUltrasound/ScreenUltrasound.ino](examples/Version4/ScreenUltrasound/ScreenUltrasound.ino)

[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)

## PS2 Controller

**Description**: The PS2 Controller methods provide an interface to read input from a PS2 controller connected to the robot. It allows you to detect button presses, joystick movements, and get vibration feedback, which can be used for remote control of the robot.

### PS2 Controller Connection

The PS2 receiver module has the following pinout:

```
┌────────────────────────────┐
│ CLK  CS  CMD DAT  VCC  GND |
│  +    +   +   +    +    +  |
└────────────────────────────┘

```

Below is the digital port on the  right side of the pila shield:

```
                    |
        D  V  G     |
        +  +  +  D1 |
        +  +  +  D2 |
        +  +  +  D3 |
        +  +  +  D4 |
        +  +  +  D5 |
        +  +  +  D6 |
                    |
```

You shd connect the pins of CLK  CS  CMD DAT to the digital port (D), and connect VCC to (V) and GND to (G).
For example, you can connect the PS2 controller to the pila shield as follows:

```
                    |
        D  V  G     |
        +  V--G  D1 |
        +  +  +  D2 |
 CLK -> |  +  +  D3 |
  CS -> |  +  +  D4 |
 CMD -> |  +  +  D5 |
 DAT -> |  +  +  D6 |
                    |
```

And remember which digital port (D1~D6) you connected the `CLK` and `DAT` pins to, because you will need to specify them when initializing the PS2 controller in your code (see the function od [ps2Init](#ps2init) and the [Example Usage of PS2 Controller](#example-usage-of-ps2-controller)).

### PS2 Button Types

| Type | Name | Description |
| ---- | ---- | ----------- |
| `PS2Button` | `PS2Select` | SELECT button |
| `PS2Button` | `PS2L3` | L3 button (left joystick press) |
| `PS2Button` | `PS2R3` | R3 button (right joystick press) |
| `PS2Button` | `PS2Start` | START button |
| `PS2Button` | `PS2Up` | D-pad UP |
| `PS2Button` | `PS2Right` | D-pad RIGHT |
| `PS2Button` | `PS2Down` | D-pad DOWN |
| `PS2Button` | `PS2Left` | D-pad LEFT |
| `PS2Button` | `PS2L2` | L2 button (below L1) |
| `PS2Button` | `PS2R2` | R2 button (below R1) |
| `PS2Button` | `PS2L1` | L1 button in controller's left shoulder |
| `PS2Button` | `PS2R1` | R1 button in controller's right shoulder |
| `PS2Button` | `PS2Triangle` | TRIANGLE button |
| `PS2Button` | `PS2Circle` | CIRCLE button |
| `PS2Button` | `PS2Cross` | CROSS button |
| `PS2Button` | `PS2Square` | SQUARE button |

### PS2 Joystick Types

| Type | Name | Description |
| ---- | ---- | ----------- |
| `PS2Joystick` | `PS2LeftJoystick` | Left joystick |
| `PS2Joystick` | `PS2RightJoystick` | Right joystick |

### PS2 Button State

| Type | Name | Description |
| ---- | ---- | ----------- |
| `PS2ButtonState` | `PS2Idle` | The button is idle |
| `PS2ButtonState` | `PS2Pressed` | The button is pressed |
| `PS2ButtonState` | `PS2Holding` | The button is being held down |
| `PS2ButtonState` | `PS2Released` | The button is released |

### PS2 Data Structures

#### PS2JoystickData

| Field | Type | Description |
| ----- | ---- | ----------- |
| `angle` | `float` | Joystick angle in degrees (0-360) |
| `strength` | `float` | Joystick strength (0-255) |

### Functions of PS2 Controller

#### ps2Init

```cpp
byte ps2Init(DigitalPinId CLK = D1_P, DigitalPinId DAT = D4_P, bool pressure = false, bool vibration = false);
```

**Description**: Initializes the PS2 controller with the specified pins and options.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `DigitalPinId` | `CLK` | The digital pin connected to the CLK pin (default: `D1_P`) |
| `DigitalPinId` | `DAT` | The digital pin connected to the DAT pin (default: `D4_P`) |
| `bool` | `pressure` | Enable pressure sensitivity (default: `false`) |
| `bool` | `vibration` | Enable vibration feedback (default: `false`) |

#### ps2Update

```cpp
void ps2Update(void);
```

**Description**: Updates the PS2 controller state. Call this in `loop()` to read the latest input.

#### ps2ButtonStateRead

```cpp
PS2ButtonState ps2ButtonStateRead(PS2Button button);
```

**Description**: Reads the current state of a PS2 button, returning one of the `PS2ButtonState` values.

#### ps2JoystickRead

```cpp
PS2JoystickData ps2JoystickRead(PS2Joystick joystick);
```

**Description**: Reads the joystick data for the specified joystick (LEFT or RIGHT).

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `PS2Joystick` | `joystick` | Which joystick to read (`PS2LeftJoystick` or `PS2RightJoystick`) |

#### ps2SetVibration

```cpp
void ps2SetVibration(byte strength);
```

**Description**: Sets the vibration motor strength.

| Type | Parameter | Description |
| ---- | --------- | ----------- |
| `byte` | `strength` | Vibration strength (0 = off, 255 = max) |

### Example Usage of PS2 Controller

```cpp
#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot;

void setup() {
  robot.init();
  // Initialize PS2 controller with default pins
  robot.ps2Init(D1_P, D4_P, false, false);
}

void loop() {
  // Update PS2 state
  robot.ps2Update();

  // Check button state
  PS2ButtonState state = robot.ps2ButtonStateRead(PS2Cross);
  switch (state) {
  case PS2Idle:
    break;
  case PS2Pressed:
    Serial.println("CROSS pressed!");
    break;
  case PS2Holding:
    Serial.println("CROSS holding!");
    break;
  case PS2Released:
    Serial.println("CROSS released!");
    break;
  }

  // Read joystick
  PS2JoystickData leftStick = robot.ps2JoystickRead(PS2LeftJoystick);
  Serial.print("Left Stick Angle: ");
  Serial.print(leftStick.angle);
  Serial.print(", Strength: ");
  Serial.println(leftStick.strength);

  delay(50);
}
```

### PS2 Related Example ino

[examples/Version4/PS2/PS2.ino](examples/Version4/PS2/PS2.ino)

[examples/Version4/PS2Remote/PS2Remote.ino](examples/Version4/PS2Remote/PS2Remote.ino)

## Bluetooth Module

**Description**: The Bluetooth module provides an interface to communicate with a Bluetooth Low Energy (BLE) device. It allows you to send and receive data wirelessly, which can be used for remote control and data logging.
