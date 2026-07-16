# PeanutKing Soccer

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-00979D)](https://www.arduino.cc/reference/en/libraries/)
[![Version](https://img.shields.io/badge/version-4.2.0-blue)](https://github.com/peanut-king-solution/PeanutKing_Soccer)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

Arduino 函式庫，用於控制 **PeanutKing 足球機器人**（V2 / V3 / V4 相容）。

---

## 目錄

- [PeanutKing Soccer](#peanutking-soccer)
  - [目錄](#目錄)
  - [簡介](#簡介)
    - [支援版本](#支援版本)
  - [安裝](#安裝)
    - [透過 Arduino IDE 安裝](#透過-arduino-ide-安裝)
    - [手動安裝](#手動安裝)
    - [相依函式庫](#相依函式庫)
  - [快速開始](#快速開始)
  - [模組總覽](#模組總覽)
  - [模組文件](#模組文件)
    - [Motor — 馬達控制](#motor--馬達控制)
      - [主要方法](#主要方法)
      - [範例](#範例)
      - [相容性包裝函式](#相容性包裝函式)
      - [相關範例](#相關範例)
    - [Movement — 全向移動](#movement--全向移動)
      - [主要方法](#主要方法-1)
      - [子類別](#子類別)
      - [範例](#範例-1)
      - [相關範例](#相關範例-1)
    - [ColorSensor — 色彩感測器](#colorsensor--色彩感測器)
      - [資料結構](#資料結構)
      - [主要方法](#主要方法-2)
      - [範例](#範例-2)
      - [相容性包裝函式](#相容性包裝函式-1)
      - [相關範例](#相關範例-2)
    - [CompoundEye — 紅外線複眼](#compoundeye--紅外線複眼)
      - [暫存器對應](#暫存器對應)
      - [主要方法](#主要方法-3)
      - [範例](#範例-3)
      - [相容性包裝函式](#相容性包裝函式-2)
      - [相關範例](#相關範例-3)
    - [ButtonManager — 按鈕管理](#buttonmanager--按鈕管理)
      - [主要方法](#主要方法-4)
      - [範例](#範例-4)
      - [相容性包裝函式](#相容性包裝函式-3)
      - [相關範例](#相關範例-4)
    - [LedController — LED 控制](#ledcontroller--led-控制)
      - [主要方法](#主要方法-5)
      - [範例](#範例-5)
      - [相容性包裝函式](#相容性包裝函式-4)
      - [相關範例](#相關範例-5)
    - [Ultrasonic — 超音波感測器](#ultrasonic--超音波感測器)
      - [主要方法](#主要方法-6)
      - [範例](#範例-6)
      - [相容性包裝函式](#相容性包裝函式-5)
      - [相關範例](#相關範例-6)
    - [Compass — 羅盤與 IMU](#compass--羅盤與-imu)
      - [主要方法](#主要方法-7)
      - [子類別](#子類別-1)
      - [範例](#範例-7)
      - [相容性包裝函式](#相容性包裝函式-6)
      - [相關範例](#相關範例-7)
    - [I2C — I2C 匯流排管理](#i2c--i2c-匯流排管理)
      - [主要方法](#主要方法-8)
      - [範例](#範例-8)
      - [相關測試](#相關測試)
    - [TFT 顯示器](#tft-顯示器)
      - [主要方法](#主要方法-9)
      - [顏色常數](#顏色常數)
      - [直接繪圖](#直接繪圖)
      - [範例](#範例-9)
      - [相關範例](#相關範例-8)
    - [工具類別](#工具類別)
      - [Converter](#converter)
      - [PIDController](#pidcontroller)
  - [範例程式](#範例程式)
  - [硬體設定](#硬體設定)
    - [預設腳位配置](#預設腳位配置)
    - [I2C 裝置地址](#i2c-裝置地址)
  - [授權條款](#授權條款)

---

## 簡介

**PeanutKing Soccer** 是一套完整的 Arduino 函式庫，支援 PeanutKing 系列足球機器人的感測器讀取、馬達控制、全向移動、無線遙控等功能。

### 支援版本

- V2（舊版相容）
- V3（舊版相容）
- **V4**（主要開發版本）

---

## 安裝

### 透過 Arduino IDE 安裝

1. 下載此函式庫的 ZIP 壓縮檔
2. 在 Arduino IDE 中選擇 **Sketch → Include Library → Add .ZIP Library...**
3. 選擇下載的 ZIP 檔案即可完成安裝

### 手動安裝

將整個 `PeanutKing_Soccer` 資料夾複製到 Arduino 的 `libraries` 目錄中：

| 平台 | 路徑 |
|------|------|
| Windows | `Documents\Arduino\libraries\` |
| macOS | `~/Documents/Arduino/libraries/` |
| Linux | `~/Arduino/libraries/` |

### 相依函式庫

此函式庫包含以下相依程式碼（已內建，無需另外安裝）：

- `PDQ_GFX` / `PDQ_ST7735` — TFT 顯示器驅動
- `SlowSoftI2CMaster` — 軟體 I2C 實作
- `pcint` — Pin Change Interrupt 中斷處理

---

## 快速開始

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();          // 初始化所有模組
  Serial.begin(9600);    // 開啟序列埠
}

void loop() {
  // 讀取羅盤方向
  uint16_t heading = robot.compass.read();
  Serial.print("Heading: ");
  Serial.println(heading);

  // 讀取超音波距離
  uint16_t dist = robot.ultrasonicRead(U1);
  Serial.print("Front distance: ");
  Serial.println(dist);

  // 設定 LED 顏色
  robot.setOnBrdLED(LED_CYAN);

  delay(100);
}
```

---

## 模組總覽

| 模組 | 類別 | 功能 |
|------|------|------|
| Motor | `robot.motor` | 4 個 DC 馬達控制 |
| Movement | `robot.move` | 麥克納姆輪全向移動 |
| ColorSensor | `robot.colorSensor` | 最多 8 個色彩感測器（I2C） |
| CompoundEye | `robot.compoundEye` | 12 頻道紅外線感測器 |
| ButtonManager | `robot.buttonMgr` | 4 個按鈕狀態管理 |
| LedController | `robot.ledCtrl` | 板載 RGB LED 控制 |
| Ultrasonic | `robot.xsound` | 4 個超音波距離感測器 |
| Compass | `robot.compass` | 羅盤方位與 IMU 資料 |
| TFT Display | `robot.tft` | ST7735 TFT 顯示器 |
| I2C | `I2CManager::getInstance()` | I2C 匯流排管理（單例） |

---

## 模組文件

### Motor — 馬達控制

控制 4 個 DC 馬達，支援速度設定、方向反轉與位置映射。

**預設馬達編號對照：**

| 名稱 | 實際 ID | 位置 |
|------|---------|------|
| `M1` | `0` | 左前輪（Left Front） |
| `M2` | `1` | 右前輪（Right Front） |
| `M3` | `2` | 右後輪（Right Back） |
| `M4` | `3` | 左後輪（Left Back） |

**方向規則：**
- 馬達正速度（`+`）→ 逆時針旋轉（CCW）
- 全部馬達正速度（`+`）→ 機器人順時針旋轉（CW）

#### 主要方法

| 方法 | 說明 |
|------|------|
| `setSpeed(MOTOR_ID mi, int16_t speed)` | 設定馬達速度，範圍 `-255`（逆時針）~ `+255`（順時針），`0` 為煞車 |
| `stopAll()` | 停止所有馬達（煞車模式） |
| `flipMotor(MOTOR_ID mi, bool flip = true)` | 反轉單一馬達旋轉方向 |
| `flipMotors(bool m1, bool m2, bool m3, bool m4)` | 分別設定四個馬達的旋轉方向 |
| `configuration(MOTOR_ID LF, MOTOR_ID RF, MOTOR_ID RB, MOTOR_ID LB)` | 重新映射馬達埠到實際安裝位置 |
| `testAll(int16_t speed)` | 依序測試所有馬達（M1→M2→M3→M4） |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // 重新映射馬達位置（如果接線錯誤，互換左前和右前馬達）
  robot.motor.configuration(M2, M1, M3, M4);

  // 反轉馬達旋轉方向
  robot.motor.flipMotor(M2, true);
}

void loop() {
  // 設定馬達速度（正速度 = 逆時針，負速度 = 順時針）
  robot.motor.setSpeed(M1, 150);   // 左前馬達逆時針 (CCW)
  robot.motor.setSpeed(M2, 150);   // 右前馬達順時針 (CW) — 方向已反轉
  robot.motor.setSpeed(M3, -100);  // 右後馬達順時針 (CW)
  robot.motor.setSpeed(M4, -100);  // 左後馬達順時針 (CW)
  delay(2000);
  robot.motor.stopAll();           // 停止所有馬達（煞車模式）
  delay(1000);
}
```

#### 相容性包裝函式

```cpp
robot.setMotorSpeed(M1, 150);   // 同 motor.setSpeed()
robot.stopAllMotors();          // 同 motor.stopAll()
```

#### 相關範例

[examples/Version4/Motor/Motor.ino](examples/Version4/Motor/Motor.ino)

---

### Movement — 全向移動

使用麥克納姆輪（Mecanum wheel）實現全向移動，支援角度控制與羅盤 PID 修正。

#### 主要方法

| 方法 | 說明 |
|------|------|
| `byAngle(float mAngle, float mSpeed, float rotate)` | 以指定角度移動，`mAngle` = `0-360°`，`mSpeed` = `0-255`，`rotate` = `-255~+255` |
| `byAnglePID(float mAngle, float mSpeed, float compassReading)` | 帶羅盤 PID 修正的移動，自動維持方向 |
| `test(float speed)` | 測試移動模式（前進 → 右前 → 右移） |

#### 子類別

| 成員 | 類別 | 用途 |
|------|------|------|
| `robot.move.converter` | `Converter` | 角度轉換工具 |
| `robot.move.motorPID` | `PIDController` | PID 控制器（預設 `Kp=300.0`, `Ki=1.0`, `Kd=2.0`） |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // 調整座標系統
  robot.move.converter.config().shift(0);  // 偏移 0 度
  robot.move.converter.config().flip();    // 翻轉 180 度 (順時針<->逆時針)
}

void loop() {
  // 基本移動
  robot.move.byAngle(0, 100, 0);     // 前進
  robot.move.byAngle(90, 100, 0);    // 左移 (已反轉)
  robot.move.byAngle(0, 0, 100);     // 順時針旋轉

  // 帶羅盤修正的移動
  robot.move.byAnglePID(0, 100, robot.compass.read());
}
```

#### 相關範例

[examples/Version4/Movement/Movement.ino](examples/Version4/Movement/Movement.ino)

---

### ColorSensor — 色彩感測器

支援最多 8 個色彩感測器（CL1–CL8），透過軟體 I2C 通訊，可讀取顏色索引、RGB、HSL、RGBC 原始值。

**預設感測器編號對照：**

| 名稱 | 實際 ID | 位置 |
|------|---------|------|
| `CL1` | `0` | Sensor 1 |
| `CL2` | `1` | Sensor 2 |
| `CL3` | `2` | Sensor 3 |
| `CL4` | `3` | Sensor 4 |
| `CL5` | `4` | Sensor 5 |
| `CL6` | `5` | Sensor 6 |
| `CL7` | `6` | Sensor 7 |
| `CL8` | `7` | Sensor 8 |

**顏色索引對照：**

| 名稱 | 實際 ID | 顏色 |
|------|---------|------|
| `CLR_BLACK` | `0` | 黑色 |
| `CLR_WHITE` | `1` | 白色 |
| `CLR_GREY` | `2` | 灰色 |
| `CLR_RED` | `3` | 紅色 |
| `CLR_GREEN` | `4` | 綠色 |
| `CLR_BLUE` | `5` | 藍色 |
| `CLR_YELLOW` | `6` | 黃色 |
| `CLR_CYAN` | `7` | 青色 |

#### 資料結構

| 結構 | 資料 | 說明 |
|------|------|------|
| `rgbc_t` | `r, g, b, c` | RGBC 原始值（各 `0-65535`, `uint16_t`） |
| `rgb_t` | `r, g, b` | RGB 值（各 `0-255`, `uint16_t`） |
| `hsl_t` | `h, s, l` | HSL 值, `h -> uint16_t`, `s, l -> uint8_t` |

#### 主要方法

| 方法 | 說明 |
|------|------|
| `readColor(CLR_SENSOR_ID)` | 讀取顏色索引（0-7） |
| `readRGB(CLR_SENSOR_ID)` | 讀取 RGB 值（各 0-255） |
| `readHSL(CLR_SENSOR_ID)` | 讀取 HSL 值 |
| `readRGBRaw(CLR_SENSOR_ID)` | 讀取原始 RGBC 值（各 0-65535） |
| `setEnabled(uint8_t mask)` | 設定啟用遮罩（預設 `0x0F` = CL1-CL4） |
| `enableSensor(CLR_SENSOR_ID, bool)` | 啟用/停用單一感測器 |
| `isEnabled(CLR_SENSOR_ID)` | 檢查感測器是否啟用 |
| `whiteLedOn(CLR_SENSOR_ID)` | 開啟底部白色 LED |
| `whiteLedOff(CLR_SENSOR_ID)` | 關閉底部白色 LED |
| `rgbwLedOn(CLR_SENSOR_ID)` | 開啟頂部 RGBW LED（顯示偵測顏色） |
| `rgbwLedOff(CLR_SENSOR_ID)` | 關閉頂部 RGBW LED |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // 啟用/停用感測器
  robot.colorSensor.setEnabled(0b00001111);  // 啟用 CL1-CL4
  robot.colorSensor.enableSensor(CL5, false); // 停用 CL5
}

void loop() {
  // 讀取顏色
  uint8_t colorIdx = robot.colorSensor.readColor(CL1);
  rgb_t rgb = robot.colorSensor.readRGB(CL1);
  hsl_t hsl = robot.colorSensor.readHSL(CL1);
  rgbc_t raw = robot.colorSensor.readRGBRaw(CL1);

  // 控制 LED
  robot.colorSensor.whiteLedOn(CL1);
  robot.colorSensor.rgbwLedOff(CL1);
}
```

#### 相容性包裝函式

```cpp
uint8_t colorIdx = robot.getColorSensor(CL1);    // 同 readColor()
rgb_t rgb = robot.getColorSensorRGB(CL1);        // 同 readRGB()
hsl_t hsl = robot.getColorSensorHSL(CL1);        // 同 readHSL()
```

#### 相關範例

[examples/Version4/Colour_Sensor/Colour_Sensor.ino](examples/Version4/Colour_Sensor/Colour_Sensor.ino)
[examples/Version4/ScreenColor/ScreenColor.ino](examples/Version4/ScreenColor/ScreenColor.ino)

---

### CompoundEye — 紅外線複眼

12 頻道紅外線感測器陣列，用於偵測足球位置。透過硬體 I2C 通訊（地址 `0x13`）。

#### 暫存器對應

| 暫存器 | 偏移 | 說明 |
|--------|------|------|
| `0x00` | 12 bytes | IR1–IR12 原始讀值（0-255） |
| `0x0C` | 1 byte | 最大 IR 值 |
| `0x0D` | 1 byte | 最大 IR 索引（1-12） |
| `0x0E` | 1 byte | 角度（需 ×2 得 0-360°） |
| `0x0F` | 1 byte | 模式（0=單球, 1=雙球） |

#### 主要方法

| 方法 | 說明 |
|------|------|
| `readAll()` | 讀取全部 12 個 IR 感測器值，回傳 `uint8_t*` 陣列 |
| `getMaxEye()` | 取得最大值感測器索引（0-11） |
| `getMaxEyeVal()` | 取得最大值感測器讀值 |
| `getEyeVal(uint8_t n)` | 取得指定索引（0-11）的感測器值 |
| `getAngle()` | 計算球的方向角度（0-360°） |
| `getMode()` | 取得偵測模式（0 = 單球, 1 = 雙球） |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // 讀取所有 IR 值
  uint8_t* ir = robot.compoundEye.readAll();
  for (int i = 0; i < 12; i++) {
    Serial.print(ir[i]);
    Serial.print(" ");
  }

  // 取得球的位置
  uint8_t maxEye = robot.compoundEye.getMaxEye();
  uint8_t maxVal = robot.compoundEye.getMaxEyeVal();
  uint16_t angle = robot.compoundEye.getAngle();
}
```

#### 相容性包裝函式

```cpp
uint8_t* ir = robot.compoundEyeRead();     // 同 readAll()
uint8_t maxEye = robot.compoundMaxEye();   // 同 getMaxEye()
uint8_t maxVal = robot.compoundMaxEyeVal();// 同 getMaxEyeVal()
uint16_t angle = robot.compoundEyeAngle(); // 同 getAngle()
```

#### 相關範例

[examples/Version4/CompoundEye/CompoundEye.ino](examples/Version4/CompoundEye/CompoundEye.ino)
[examples/Version4/ScreenIR/ScreenIR.ino](examples/Version4/ScreenIR/ScreenIR.ino)

---

### ButtonManager — 按鈕管理

管理 4 個按鈕（BTN_1–BTN_4），支援按下、放開、長按、雙擊等手勢偵測。

**按鈕編號對照：**

| 名稱 | 實際 ID | 位置 |
|------|---------|------|
| `BTN_1` | `1` | Button 1 |
| `BTN_2` | `2` | Button 2 |
| `BTN_3` | `3` | Button 3 |
| `BTN_4` | `4` | Button 4 |

**手勢狀態對照：**

| 名稱 | 實際 ID | 說明 |
|------|---------|------|
| `NONE` | `0` | 無事件 |
| `TAP` | `1` | 單擊 |
| `PRESS` | `2` | 按下 |
| `HOLD` | `3` | 長按（1000ms） |
| `TAP2` | `4` | 雙擊 |
| `TAP3` | `5` | 三擊 |
| `RELEASE` | `6` | 放開 |
| `HOLD2` | `13` | 超長按 |

#### 主要方法

| 方法 | 說明 |
|------|------|
| `read(BUTTON_ID btn)` | 讀取按鈕是否按下（`true` / `false`） |
| `update()` | 更新按鈕狀態機（需在 `loop()` 中定期呼叫） |
| `getStatus(BUTTON_ID btn)` | 取得按鈕手勢狀態 |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // 基本讀取
  if (robot.buttonMgr.read(BTN_1)) {
    Serial.println("Button 1 pressed");
  }

  // 手勢偵測（需定期呼叫 update()）
  robot.buttonMgr.update();
  buttonStatus_t status = robot.buttonMgr.getStatus(BTN_1);
  switch (status) {
    case TAP:  Serial.println("TAP");  break;
    case TAP2: Serial.println("DOUBLE TAP"); break;
    case HOLD: Serial.println("HOLD"); break;
  }
}
```

#### 相容性包裝函式

```cpp
bool pressed = robot.buttonRead(BTN_1);  // 同 read()
robot.buttonUpdate();                    // 同 update()
buttonStatus_t s = robot.buttonGetStatus(BTN_1); // 同 getStatus()
```

#### 相關範例

[examples/Version4/Button/Button.ino](examples/Version4/Button/Button.ino)
[examples/Version4/Button_StateMachine/Button_StateMachine.ino](examples/Version4/Button_StateMachine/Button_StateMachine.ino)

---

### LedController — LED 控制

控制板載 RGB LED，支援 8 種顏色模式。

**LED 顏色對照：**

| 名稱 | 實際 ID | 顏色 |
|------|---------|------|
| `LED_OFF` | `0` | 關閉 |
| `LED_BLUE` | `1` | 藍色 |
| `LED_GREEN` | `2` | 綠色 |
| `LED_CYAN` | `3` | 青色 |
| `LED_RED` | `4` | 紅色 |
| `LED_PURPLE` | `5` | 紫色 |
| `LED_YELLOW` | `6` | 黃色 |
| `LED_WHITE` | `7` | 白色 |

#### 主要方法

| 方法 | 說明 |
|------|------|
| `setOnBrdLED(obBrdLEDCL color)` | 設定全部 LED 顏色 |
| `setOnBrdLED(uint8_t LED, uint8_t status)` | 設定單一色 LED（0=紅, 1=綠, 2=藍） |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();
}

void loop() {
  // 設定顏色
  robot.ledCtrl.setOnBrdLED(LED_RED);
  robot.ledCtrl.setOnBrdLED(LED_CYAN);

  // 個別控制
  robot.ledCtrl.setOnBrdLED(0, HIGH);  // 紅 LED 亮
  robot.ledCtrl.setOnBrdLED(1, LOW);   // 綠 LED 滅
}
```

#### 相容性包裝函式

```cpp
robot.setOnBrdLED(LED_CYAN);
robot.setOnBrdLED(0, HIGH);
```

#### 相關範例

[examples/Version4/LED/LED.ino](examples/Version4/LED/LED.ino)
[examples/Version4/Digital_Analog/Digital_Analog.ino](examples/Version4/Digital_Analog/Digital_Analog.ino)

---

### Ultrasonic — 超音波感測器

管理 4 個超音波距離感測器（U1–U4），使用 Pin Change Interrupt 實現非阻塞式測距，採用輪詢觸發避免干擾。

**預設感測器編號對照：**

| 名稱 | 實際 ID | 位置 |
|------|---------|------|
| `U1` | `0` | 前方（Front） |
| `U2` | `1` | 右方（Right） |
| `U3` | `2` | 後方（Back） |
| `U4` | `3` | 左方（Left） |

#### 主要方法

| 方法 | 說明 |
|------|------|
| `read(ULTR_SENSOR sensor)` | 讀取距離（mm，範圍 0–4500mm） |
| `configuration(ULTR_SENSOR Front, Right, Back, Left)` | 重新映射感測器實體位置 |
| `setEnabled(bool u1, bool u2, bool u3, bool u4)` | 設定啟用狀態 |
| `enableSensor(ULTR_SENSOR sensor, bool enabled)` | 啟用/停用單一感測器 |
| `enableAll(bool enabled)` | 啟用/停用全部感測器 |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // 重新映射感測器位置
  robot.xsound.configuration(U2, U1, U4, U3);

  // 啟用/停用
  robot.xsound.enableSensor(U1, true);
  robot.xsound.enableAll(false);
}

void loop() {
  // 讀取各方向距離
  uint16_t front = robot.xsound.read(U1);
  uint16_t right = robot.xsound.read(U2);
}
```

#### 相容性包裝函式

```cpp
uint16_t dist = robot.ultrasonicRead(U1);
```

#### 相關範例

[examples/Version4/Ultrasonic/Ultrasonic.ino](examples/Version4/Ultrasonic/Ultrasonic.ino)
[examples/Version4/ScreenXsound/ScreenXsound.ino](examples/Version4/ScreenXsound/ScreenXsound.ino)

---

### Compass — 羅盤與 IMU

透過硬體 I2C 讀取羅盤方位（0–360°）與 9 軸 IMU 原始資料（加速度計、陀螺儀、磁力計）。

#### 主要方法

| 方法 | 說明 |
|------|------|
| `read()` | 讀取羅盤方位（0–360°），順時針 |
| `getAccelerometerRaw()` | 取得原始加速度計資料 `int16_t[3]`（X, Y, Z） |
| `getGyroscopeRaw()` | 取得原始陀螺儀資料 `int16_t[3]`（X, Y, Z） |
| `getMagnetometerRaw()` | 取得原始磁力計資料 `int16_t[3]`（X, Y, Z） |
| `clearBuffer()` | 清除接收緩衝區 |

#### 子類別

| 成員 | 用途 |
|------|------|
| `robot.compass.converter` | 方向角度轉換工具 |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init();

  // 校正零點
  robot.compass.converter.config().shift(robot.compass.read());
}

void loop() {
  // 讀取方位
  uint16_t heading = robot.compass.read();
  Serial.print("Heading: ");
  Serial.println(heading);

  // 讀取 IMU 原始資料
  int16_t* accel = robot.compass.getAccelerometerRaw();
  int16_t* gyro = robot.compass.getGyroscopeRaw();
  int16_t* mag = robot.compass.getMagnetometerRaw();
}
```

#### 相容性包裝函式

```cpp
uint16_t heading = robot.compassRead();
int16_t* accel = robot.getAccelerometerRaw();
```

#### 相關範例

[examples/Version4/Compass/Compass.ino](examples/Version4/Compass/Compass.ino)
[examples/Version4/CompassCar/CompassCar.ino](examples/Version4/CompassCar/CompassCar.ino)
[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)

---

### I2C — I2C 匯流排管理

管理 8 條軟體 I2C 匯流排（SW0–SW7）和 1 條硬體 I2C 匯流排（HW），採用單例模式。

**匯流排索引對照：**

| 名稱 | 實際 ID | 類型 |
|------|---------|------|
| `SW0`–`SW7` | `0`–`7` | 軟體 I2C |
| `HW` | `8` | 硬體 I2C |

**I2C_Handle 結構：**

| 欄位 | 說明 |
|------|------|
| `busIndex` | 匯流排索引（`BusIndex`） |
| `deviceAddress` | I2C 裝置地址（`0x00`–`0x7F`） |
| `speed` | I2C 速度（Hz） |
| `isValid()` | 檢查 Handle 是否有效 |

#### 主要方法

| 方法 | 說明 |
|------|------|
| `I2CManager::getInstance()` | 取得單例實例 |
| `init()` | 初始化所有 I2C 匯流排 |
| `RegisterDevice(BusIndex, address, speed)` | 註冊 I2C 裝置，回傳 `I2C_Handle` |
| `SensorRead(handle, reg, buffer, length)` | 讀取 I2C 暫存器資料 |
| `SensorSend(handle, buffer, length)` | 發送資料到 I2C 裝置 |

#### 範例

```cpp
#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();
I2CManager& i2c = I2CManager::getInstance();

void setup() {
  robot.init();
  i2c.init();

  // 註冊裝置
  I2C_Handle device = i2c.RegisterDevice(BusIndex::HW, 0x08, 400000);
}

void loop() {
  // 讀取資料
  uint8_t buffer[8];
  i2c.SensorRead(device, 0x55, buffer, 3);
}
```

#### 相關測試

[test/test_i2c_manager/test_i2c_manager.ino](test/test_i2c_manager/test_i2c_manager.ino)

---

### TFT 顯示器

ST7735 TFT 顯示器（128×160 像素），透過 SPI 通訊，繼承自 PDQ_GFX 繪圖庫。

#### 主要方法

| 方法 | 說明 |
|------|------|
| `clearScreen()` | 清除螢幕（填黑） |
| `setTextColor(uint16_t color)` | 設定文字顏色 |
| `setTextColor(uint16_t fg, uint16_t bg)` | 設定前景與背景色 |
| `setTextSize(uint8_t size)` | 設定文字大小（1-3） |
| `setScreen(uint8_t col, uint8_t row, char string[])` | 在指定位置顯示文字 |
| `setScreen(uint8_t col, uint8_t row, int16_t number)` | 在指定位置顯示數字 |
| `drawAnglePointer(x, y, radius, angle, color)` | 繪製方向指標（附 N/S/E/W 標記） |

#### 顏色常數

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

#### 直接繪圖

透過 `robot.tft` 可直接使用 PDQ_ST7735 的繪圖方法：

```cpp
robot.tft.fillCircle(x, y, r, color);
robot.tft.drawRect(x, y, w, h, color);
robot.tft.fillTriangle(x1, y1, x2, y2, x3, y3, color);
robot.tft.drawLine(x0, y0, x1, y1, color);
// 更多方法請參考 PDQ_GFX 文件
```

#### 範例

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
  // 清除舊值
  robot.setTextColor(ST7735_BLACK);
  robot.setScreen(0, 1, (int16_t)oldValue);
  robot.setTextColor(ST7735_WHITE);
  robot.setScreen(0, 1, (int16_t)newValue);

  // 繪製方向指標
  robot.drawAnglePointer(64, 120, 25, heading);
}
```

#### 相關範例

[examples/Version4/LCDScreen/LCDScreen.ino](examples/Version4/LCDScreen/LCDScreen.ino)
[examples/Version4/ScreenCompass/ScreenCompass.ino](examples/Version4/ScreenCompass/ScreenCompass.ino)
[examples/Version4/ScreenColor/ScreenColor.ino](examples/Version4/ScreenColor/ScreenColor.ino)
[examples/Version4/ScreenIR/ScreenIR.ino](examples/Version4/ScreenIR/ScreenIR.ino)
[examples/Version4/ScreenXsound/ScreenXsound.ino](examples/Version4/ScreenXsound/ScreenXsound.ino)

---

### 工具類別

#### Converter

角度轉換工具，支援翻轉、偏移與正規化。可用於羅盤校正或移動座標調整。

```cpp
class Converter {
public:
  Converter& config();          // 開始方法鏈
  Converter& flip();            // 翻轉方向（+/- 反轉）
  Converter& shift(float deg);  // 偏移角度
  float normalize(float angle); // 正規化到 [0, 360)
  float convert(float angle);   // 套用轉換
  void reset();                 // 重置為預設
};
```

使用範例：

```cpp
// 方法鏈
robot.compass.converter.config().shift(90).flip();

// 或分別設定
robot.move.converter.config().shift(180);
```

#### PIDController

PID 控制演算法，用於 Movement 模組的羅盤方向修正。

```cpp
class PIDController {
public:
  PIDController(double kp, double ki, double kd);
  double update(double currentValue);  // 計算 PID 輸出

  double kp, ki, kd;        // PID 係數
  double setPoint;          // 目標值
  double integral;          // 積分項
  double previousError;     // 前次誤差
};
```

使用範例：

```cpp
// 自訂 PID 參數（預設 Kp=300.0, Ki=1.0, Kd=2.0）
robot.move.motorPID.kp = 200.0;
robot.move.motorPID.ki = 0.5;
robot.move.motorPID.kd = 1.0;
```

---

## 範例程式

| 範例 | 說明 |
|------|------|
| [Bluetooth_Remote](examples/Version4/Bluetooth_Remote/Bluetooth_Remote.ino) | 藍牙遙控 |
| [Button](examples/Version4/Button/Button.ino) | 基本按鈕讀取 |
| [Button_StateMachine](examples/Version4/Button_StateMachine/Button_StateMachine.ino) | 按鈕狀態機（TAP/HOLD） |
| [Colour_Sensor](examples/Version4/Colour_Sensor/Colour_Sensor.ino) | 色彩感測器讀取 |
| [Compass](examples/Version4/Compass/Compass.ino) | 羅盤與 IMU 資料 |
| [CompassCar](examples/Version4/CompassCar/CompassCar.ino) | 羅盤導航 |
| [CompoundEye](examples/Version4/CompoundEye/CompoundEye.ino) | 紅外線複眼 |
| [Digital_Analog](examples/Version4/Digital_Analog/Digital_Analog.ino) | GPIO 數位/類比 I/O |
| [Goalkeeper](examples/Version4/Goalkeeper/Goalkeeper.ino) | 守門員行為策略 |
| [LCDScreen](examples/Version4/LCDScreen/LCDScreen.ino) | TFT 顯示器 |
| [LED](examples/Version4/LED/LED.ino) | RGB LED 控制 |
| [Motor](examples/Version4/Motor/Motor.ino) | 馬達測試與設定 |
| [Movement](examples/Version4/Movement/Movement.ino) | 全向移動 |
| [ScreenColor](examples/Version4/ScreenColor/ScreenColor.ino) | 螢幕 + 色彩感測器整合 |
| [ScreenCompass](examples/Version4/ScreenCompass/ScreenCompass.ino) | 螢幕 + 羅盤整合 |
| [ScreenIR](examples/Version4/ScreenIR/ScreenIR.ino) | 螢幕 + 複眼整合 |
| [ScreenXsound](examples/Version4/ScreenXsound/ScreenXsound.ino) | 螢幕 + 超音波整合 |
| [Striker](examples/Version4/Striker/Striker.ino) | 前鋒行為策略 |
| [Ultrasonic](examples/Version4/Ultrasonic/Ultrasonic.ino) | 超音波感測器 |

---

## 硬體設定

### 預設腳位配置

| 功能 | 腳位 | 說明 |
|------|------|------|
| TFT CS | 0 | TFT 晶片選擇 |
| TFT DC | 53 | TFT 資料/指令 |
| TFT RST | 50 | TFT 重置 |
| TFT SCL | 52 | TFT SPI 時脈 |
| TFT SDA | 51 | TFT SPI 資料 |
| S1–S4 | 10–13 | 伺服/PWM 輸出 |
| D1–D3 | 53–55 | 數位輸入 |
| D4–D6 | 56–58 | 數位輸出 |
| A1–A4 | 59–62 | 類比輸入 |

### I2C 裝置地址

| 裝置 | 地址 | 匯流排 |
|------|------|--------|
| 羅盤模組 | `0x08` | 硬體 I2C (HW) |
| 複眼模組 | `0x13` | 硬體 I2C (HW) |
| 色彩感測器 CL1–CL8 | 自訂 | 軟體 I2C (SW0–SW7) |

---

## 授權條款

本專案採用 MIT 授權條款。詳細內容請參閱 LICENSE 檔案。

Copyright © 2024 PeanutKing Solution