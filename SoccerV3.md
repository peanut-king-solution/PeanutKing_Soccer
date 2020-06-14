# Soccer Robot v3.0 scope

## Repository Contents
Upgrade version Soccer Robot 2020
New features:
- Dependent SensorBoard handling IR sensor and color sensor
- Dependent TopControlBoard for ultrasonic sensor and LED
- Built-in de-noise algorithm for all sensor
- up to 100hz update frequency

### Input sensors
- Compass 
- Ultrasonic
- IR sensor
- Color sensor

### Output acuators
- Motor
- LED
- LCD Screen

---
### 1. Sensorboard
- STM32 MCU 
- IR eye ring
- 4 color sensors
- communicates to arduino
- Data

|Sensor| no. | bit | byte length | IOs |
| --- | --- | --- | --- | --- |
| IR | 12 | 2 | 24 | I |
|Color| 4 | 3 | 12 | I |

max input length: 32
---
### 2. Top brd controller
Rgb led on mainboard moved to top brd 
plus controlled by xsonic stm32
- STM32 MCU 
- xsonic
- LEDs

|Sensor| no. |  | byte length | IOs |
| --- | --- | --- | --- | --- |
|xsonic| 4 | 2 | 8  | I |
| LEDs | 8 | 4 | 32 | O |

input length : 8
output length: 32

---

### 3. Compass BMX055
length: 2 byte
Commands: set zero, calibrate, 6/9 dof switching

---
### 4. Arduino V3 Library
Advanced control library p5

---
## protocol Draft (v3.0)
format: (sensor)(cmd)(- optional)

### Slave
1. receive one byte (sensor type)
2. receive one byte (cmd)
3. respond
- send data: (sensor)(cmdSendData)(data)
- receive byte length depends on sensor

### Master
request data: (sensor)(cmdRequstData) - 2 byte
send data: (sensor)(cmdSendData)(data)
set parameter: (sensor)(cmdSetParameter)(parameter name)(value)

### Reserved word list:
Commands
'T' - cmdTransmit
'R' - cmdReceive
'P' - cmdParameter

Inputs
'C' - Compass
'U' - Ultrasonic
'I' - IR sensor
'K' - Color sensor

Outputs
'M' - Motor
'L' - LED
'S' - Screen

---
## APP
Featueres:
- Power move
- Joystick control
- Data reading
- Attributes stats
- Upcoming programmable control

---
## code

### protocol prototype

#### Slave
```
```

### Basic functions
```
  uint16_t
    compassRead(),
    ultrasonicRead(idx),
    compoundEyeRead(idx),
    colorRead(idx);

  void
    motorSet(idx, speed),
    ledSet(idx, r, g, b, w),
    screenSet(x, y, text);
```
