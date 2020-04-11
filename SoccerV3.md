# Soccer Robot v3.0 scope

## Repository Contents

### Inputs
- Compass 
- Ultrasonic
- IR sensor
- Color sensor

### Outputs
- Motor
- LED
- LCD Screen

---

So for , up to now we will have:
Sensor board stm32 firmware p1
Xsonic stm32 firmware p2
Arduino <-> stm32s comm p3
Bnx055 stm32 firmware p4
Advanced control library p5
(Potential motor control stm32 firmware p6)

---
### Sensorboard:
- STM32 MCU 
- IR eye ring
- 4 color sensors
- communicates to arduino
- Data

|Sensor| no. | bit | byte length |
| --- | --- | --- | --- |
| IR | 12 | 12bit | 24 |
|Color| 4 | 10bit | 8 |

Total length: 32

### Top brd controller
- STM32 MCU 
- xsonic
- LEDs

|Sensor| no. | bit | byte length |
| --- | --- | --- | --- |
|xsonic| 4 | 10bit | 8  |
| LEDs | 8 | 32bit | 32 |


Rgb led on mainboard moved to top brd plus controlled by xsonic stm32

---
## protocol Draft


---

APP
---
code
```
  uint16_t
    compoundEyeRead(uint8_t),
    ultrasonicRead(uint8_t),
    compassRead(void);
```
