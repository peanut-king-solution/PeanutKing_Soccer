# Peanut Queen Controller — BLE Protocol Specification

> Generated from codebase analysis on 2026-09-16
> Branch: `android_ios_ble_v1`

---

## 1. BLE Transport Layer (Shared)

### Service & Characteristic UUIDs

| Role | BBC micro:bit NUS | BT04-A (HM-10) |
|------|-------------------|----------------|
| **Service UUID** | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | `0000ffe0-0000-1000-8000-00805f9b34fb` |
| **RX** (App → Device) | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Single characteristic (write) |
| **TX** (Device → App) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | Single characteristic (notify) |

### Transmission Rules

| Parameter | Value |
|-----------|-------|
| Max chunk size | **20 bytes** (BLE 4.0 limit) |
| Write mode | `writeWithoutResponse` preferred |
| Inter-chunk delay | 20 ms |
| MTU request | 512 |
| Encoding | UTF-8 |
| Message delimiter | newline character `0x0A` |

### Newline Behavior

Every message sent by the app **must end with newline** (`0x0A`). The device uses newline as the message boundary to know where one command ends and the next begins.

- `sendMicrobitMessage` — always appends newline before sending
- `sendArduinoMessage` — always appends newline before sending
- Device → App parsing splits on newline to assemble complete messages from chunked BLE packets

---

## 2. Three Operating Modes — Overview

The app connects to a BLE device, then shows a **device type selection page**. The user picks one of three modes:

```
┌─────────────────────────────────────┐
│         Select Device Type          │
│                                     │
│  🤖 Robot PILA              (Legacy)│
│  🤖 Robot PILA (Config)     (Config)│
│  📊 Data Management Dashboard (Dash)│
└─────────────────────────────────────┘
```

Each mode uses **different command sets** and **different data flows**.

| Mode | UI | Commands Available | Telemetry Format |
|------|----|--------------------|-----------------|
| **Soccer Legacy** | Hardcoded | `J` `P` `C` `Z` `H` `B` `D` | Fixed 7-field |
| **Soccer Config** | Config-driven | All above + `T` (toggle/slider/text) | Dynamic `R,...` |
| **Dashboard** | Config-driven | `T` only (toggle/slider/text) | Dynamic `R,...` |

---

## 3. Soccer Legacy Mode (Robot PILA)

### When to Use

Robot soccer car with **fixed hardware layout** — compass, 4 ultrasonic sensors, eye sensor. No config string needed.

### Complete Data Flow

```
Device                                    App
  │                                        │
  │──── BLE Connect ──────────────────────>│
  │                                        │
  │<─── ACK: "Correct config received\n" ──│
  │                                        │
  │──── Telemetry (periodic) ─────────────>│
  │     soccer,90,25,30,15,20,3,100        │
  │                                        │
  │<─── Commands (on user input) ──────────│
  │     J90,153 / P0 / C060 / etc.         │
  │                                        │
  │<─── Disconnect ───────────────────────>│
```

### Step-by-Step

1. **Connect** — App connects via BLE, requests MTU 512
2. **Discover** — App discovers services, locks onto NUS or HM-10 service
3. **ACK handshake** — App sends `"Correct config received\n"` to notify device that app is ready
4. **Device sends telemetry** — Device periodically sends fixed-format sensor data
5. **User controls robot** — App sends movement/color/PID commands
6. **Disconnect** — User leaves page, app disconnects

### 3.1 Device → App: Telemetry

```
soccer,<compass>,<ultrasound_front>,<ultrasound_back>,<ultrasound_left>,<ultrasound_right>,<max_eye>,<max_eye_value>
```

| Index | Field | Type | Example |
|-------|-------|------|---------|
| 0 | compass | int | `90` |
| 1 | ultrasound_front | int | `25` |
| 2 | ultrasound_back | int | `30` |
| 3 | ultrasound_left | int | `15` |
| 4 | ultrasound_right | int | `20` |
| 5 | max_eye | int | `3` |
| 6 | max_eye_value | int | `100` |

**Parsing logic:**
1. Split by `,`
2. Verify `split[0] == "soccer"` (device identifier match)
3. Remove the identifier
4. Remaining 7 values stored as `infoFromDevice`

### 3.2 App → Device: Commands

#### Joystick Movement

```
J<angle>,<speed>
```

| Field | Range | Description |
|-------|-------|-------------|
| angle | 0–360 | Direction in degrees (0=forward, 90=right, etc.) |
| speed | 0–255 | Motor power |

**Examples:**
- `J90,153` → Move right at speed 153
- `J0,200` → Move forward at speed 200
- `J315,128` → Move left-front at speed 128

**Joystick → Motor mapping (Pad mode):**

| Direction | Angle | Speed Index |
|-----------|-------|------------|
| Forward | 0° | `speeds[0]` |
| Right | 90° | `speeds[3]` |
| Backward | 180° | `speeds[1]` |
| Left | 270° | `speeds[2]` |
| Right-Front | 45° | `speeds[5]` |
| Right-Back | 135° | `speeds[6]` |
| Left-Back | 225° | `speeds[7]` |
| Left-Front | 315° | `speeds[4]` |

Joystick polling interval: **100 ms** (throttled from continuous input)

#### Stop All Motors

```
P0
```

Sent when user releases joystick or lifts finger from pad.

#### Turn (Spin in Place)

```
C<speed>
```

Speed is 3-digit zero-padded. Negative speeds use offset encoding to avoid sending a minus sign:

```
transmitted = speed >= 0 ? speed : speed + 900
```

| Actual Speed | Transmitted | Meaning |
|-------------|-------------|---------|
| +60 | `C060` | Spin right at 60 |
| -60 | `C840` | Spin left at 60 |
| +255 | `C255` | Spin right at max |
| -255 | `C645` | Spin left at max |

#### LED RGB Color

```
Z<RRR><GGG><BBB>
```

Each channel is 3-digit zero-padded (000–255). Sent continuously as user drags sliders.

| Example | Meaning |
|---------|---------|
| `Z255000000` | Full red |
| `Z000255000` | Full green |
| `Z000000255` | Full blue |
| `Z255128000` | Orange |
| `Z000000000` | LED off |

#### Heading Reset

```
H0
```

Resets the compass heading reference point.

#### Button Press / Release

```
B1    ← press down
B0    ← release
```

Used for kick / action button. Press → hold → release pattern.

#### PID Tuning

```
D<kp>,<ki>,<kd>
```

| Field | Default | Range |
|-------|---------|-------|
| kp | 130 | 0–255 |
| ki | 1 | 0–100 |
| kd | 40 | 0–100 |

Sent once via "Send to Arduino" button in Settings dialog.

**Example:** `D130,1,40`

---

## 4. Soccer Config Mode (Robot PILA Config)

### When to Use

Robot soccer car with **configurable UI** — device tells the app what controls to show. Supports same motor commands as Legacy, plus dynamic toggles/sliders/text fields.

### Complete Data Flow

```
Device                                    App
  │                                        │
  │──── BLE Connect ──────────────────────>│
  │                                        │
  │<─── ACK: "Correct config received\n" ──│
  │                                        │
  │──── Config String ────────────────────>│
  │     C,I,1,J,angle,100,power,drive,     │
  │     S,0,255,brightness,B,kick,TB,led   │
  │                                        │
  │     ← App builds UI from config →      │
  │                                        │
  │<─── Legacy Commands (motor, LED, etc) ─│
  │     J90,153 / P0 / Z255000000 / ...    │
  │                                        │
  │<─── Config Commands (T protocol) ──────│
  │     T,brightness,128, / T,led,1,       │
  │                                        │
  │──── Telemetry (if O in config) ───────>│
  │     R,compass,90,ultrasound_front,25   │
  │                                        │
  │<─── Disconnect ───────────────────────>│
```

### Step-by-Step

1. **Connect** — Same as Legacy
2. **ACK handshake** — Same as Legacy
3. **Device sends config string** — Device sends `C,...` describing what UI elements to show
4. **App builds UI** — Dynamically creates joystick, sliders, buttons, toggles, text fields based on config
5. **User controls robot** — Legacy commands (J/P/C/Z/H/B/D) still work for motor/LED/PID
6. **User uses config controls** — Toggle/slider/text field send `T` commands
7. **Device sends telemetry** — If config includes `O` (plot), device sends `R,...` data
8. **Disconnect**

### 4.1 Device → App: Config String

Sent once after ACK handshake. Defines the UI layout.

```
C,<type><params>,<type><params>,...
```

#### Config Element Types

| Code | Type | Parameters | Example |
|------|------|-----------|---------|
| `I` | Input | `<numOfControllers>` | `I,2` |
| `J` | Joystick | `<angleName>,<maxStrength>,<strengthName>,<joystickName>` | `J,angle,100,power,drive` |
| `S` | Slider | `<min>,<max>,<name>` | `S,0,255,brightness` |
| `B` | Button | `<name>` | `B,kick` |
| `TB` | Toggle Button | `<name>` | `TB,led` |
| `TF` | Text Field | `<name>` | `TF,message` |
| `O` | Plot | `<numOfPlot>,<name1>,<plotable1>,<name2>,<plotable2>,...` | `O,2,temp,true,humidity,false` |

#### Full Config String Example

```
C,I,1,J,angle,100,power,drive,S,0,255,brightness,B,kick,TB,led,O,2,compass,true,ultrasound_front,true
```

This configures:
- `I,1` → 1 controller input
- `J,angle,100,power,drive` → 1 joystick (max strength 100)
- `S,0,255,brightness` → 1 slider (0–255, named "brightness")
- `B,kick` → 1 momentary button (named "kick")
- `TB,led` → 1 toggle button (named "led")
- `O,2,compass,true,ultrasound_front,true` → 2 plot channels (both graphable)

### 4.2 App → Device: Config Commands (`T` protocol)

Sent from dynamically-generated UI elements. Unified format:

```
T,<param_name>,<value>,
```

| UI Element | `<value>` | Example | When Sent |
|------------|----------|---------|-----------|
| Toggle Button | `0` or `1` | `T,led,1,` | On tap (toggle on/off) |
| Slider | numeric (double) | `T,brightness,128.0,` | On every drag change |
| Text Field | free text | `T,message,hello,` | On submit |

> **Important:** The `T` command ends with a trailing comma `,`. This is the protocol convention.

### 4.3 App → Device: Legacy Commands (Still Work)

All Soccer Legacy commands remain functional in Config mode:

| Command | Format | Purpose |
|---------|--------|---------|
| `J` | `J<angle>,<speed>` | Joystick motor control |
| `P` | `P0` | Stop all motors |
| `C` | `C<speed>` | Spin in place |
| `Z` | `Z<RRR><GGG><BBB>` | LED RGB color |
| `H` | `H0` | Reset heading |
| `B` | `B1`/`B0` | Button press/release |
| `D` | `D<kp>,<ki>,<kd>` | PID tuning |

### 4.4 Device → App: Telemetry (If Config Has `O`)

If the config string includes plot elements (`O`), the device sends telemetry data:

```
R,<name1>,<value1>,<name2>,<value2>,...
```

Same format as Dashboard mode (see Section 5.2).

---

## 5. Dashboard Mode (micro:bit Data Management)

### When to Use

**micro:bit** or **Arduino + HM-10** for **data monitoring** — no motor control, just sensor readings, graphs, and config-driven controls (sliders, toggles, text fields).

### Complete Data Flow

```
Device (micro:bit / Arduino)         App
  │                                        │
  │──── BLE Connect ──────────────────────>│
  │                                        │
  │<─── ACK: "Correct config received\n" ──│
  │                                        │
  │──── Config String ────────────────────>│
  │     C,O,3,temp,true,humidity,true,     │
  │     light,false,S,0,100,brightness,    │
  │     TB,led,TF,name                     │
  │                                        │
  │     ← App builds dashboard UI →        │
  │                                        │
  │──── Telemetry (periodic) ─────────────>│
  │     R,temp,24.5,humidity,60.0,light,350│
  │                                        │
  │<─── Config Commands (T protocol) ──────│
  │     T,brightness,64, / T,led,1,        │
  │                                        │
  │<─── Disconnect ───────────────────────>│
```

### Step-by-Step

1. **Connect** — Same as other modes
2. **ACK handshake** — App sends `"Correct config received\n"`
3. **Device sends config string** — Device sends `C,...` describing dashboard layout
4. **App builds dashboard** — Creates sensor cards, graphs, sliders, toggles, text fields
5. **Device sends telemetry** — Device periodically sends `R,...` sensor readings
6. **App displays data** — Values update in real-time, numeric values graph over time
7. **User sends commands** — Sliders/toggles/text fields send `T` commands to device
8. **Disconnect**

### 5.1 Device → App: Config String

Same format as Soccer Config (Section 4.1). Typically focuses on:

- `O` — Plot/sensor channels (which sensors to display and graph)
- `S` — Sliders (adjustable parameters)
- `TB` — Toggle buttons (on/off switches)
- `TF` — Text fields (free-text input)

**Example:**

```
C,O,3,temp,true,humidity,true,light,false,S,0,100,brightness,TB,led,TF,name
```

This configures:
- `O,3,temp,true,humidity,true,light,false` → 3 sensor channels (temp & humidity graphable, light is display-only)
- `S,0,100,brightness` → 1 slider (0–100)
- `TB,led` → 1 toggle button
- `TF,name` → 1 text field

### 5.2 Device → App: Telemetry

Sent periodically by the device. Variable-length key-value pairs:

```
R,<name1>,<value1>,<name2>,<value2>,...
```

| Part | Description |
|------|-------------|
| `R` | Telemetry marker (always first) |
| `<name>` | Sensor name (auto-normalized to lowercase) |
| `<value>` | Reading value (string; parsed to double for graphing if numeric) |

**Examples:**

```
R,temp,24.5,humidity,60.0
R,compass,90,ultrasound_front,25,light,350
R,soil_moisture,45,ph,6.8,temperature,22.3
```

**Parsing logic:**
1. Verify string starts with `R`
2. Iterate through comma-separated parts in pairs: index 1 = name, index 2 = value
3. Name is normalized to lowercase for consistent matching
4. If value is a valid number → add to time-series history buffer
5. History auto-slides: max 30 visible points per sensor
6. `onDataReceived` callback notifies all listeners

### 5.3 App → Device: Commands (`T` protocol only)

Dashboard mode uses **only** the `T` protocol for outgoing commands:

| Command | Format | Example | When Sent |
|---------|--------|---------|-----------|
| Toggle | `T,<name>,<0 or 1>,` | `T,led,1,` | On tap |
| Slider | `T,<name>,<value>,` | `T,brightness,64,` | On drag |
| Text Field | `T,<name>,<text>,` | `T,message,hello,` | On submit |

> **Difference from Soccer Config:** In Dashboard mode, messages are sent via `sendMicrobitMessage` which always appends newline. In Soccer Config, messages go through `sendArduinoMessage` which also appends newline. The wire format is identical.

---

## 6. Command Quick Reference

### All Outgoing Commands (App → Device)

| Cmd | Format | Modes | Purpose |
|-----|--------|-------|---------|
| `J` | `J<angle>,<speed>` | Soccer (all) | Joystick motor control |
| `P` | `P0` | Soccer (all) | Stop all motors |
| `C` | `C<speed>` | Soccer (all) | Spin in place (3-digit, negative = +900) |
| `Z` | `Z<RRR><GGG><BBB>` | Soccer (all) | LED RGB color (3×3-digit) |
| `H` | `H0` | Soccer (all) | Reset compass heading |
| `B` | `B1` / `B0` | Soccer (all) | Button press / release |
| `D` | `D<kp>,<ki>,<kd>` | Soccer (all) | PID tuning |
| `T` | `T,<name>,<value>,` | Config / Dashboard | Toggle / Slider / Text Field |

### All Incoming Data (Device → App)

| Prefix | Format | Modes | Purpose |
|--------|--------|-------|---------|
| `soccer` | `soccer,<7 ints>` | Soccer Legacy | Fixed telemetry |
| `R` | `R,<n>,<v>,<n>,<v>,...` | Config / Dashboard | Dynamic telemetry |
| `C` | `C,<elements>` | Config / Dashboard | UI config string |

### Message Delimiter

**All messages end with newline (0x0A).**

The app's BLE parser accumulates incoming bytes and splits on `\n` to extract complete messages. This handles BLE's 20-byte chunking — a long message may arrive as multiple BLE packets, reassembled by the newline delimiter.

---

## 7. Message Reassembly

The device parser (`_onDataReceived` in communication.dart) handles:

1. **Backspace characters** (byte `8` or `127`) — treated as delete previous character
2. **Buffer accumulation** — partial messages stored until `\n` arrives
3. **Message extraction** — everything before `\n` is the complete message
4. **Regex cleanup** — removes non-alphanumeric characters except `().,;:-?`
5. **Route by prefix:**
   - Starts with `R` → parse as telemetry (`ReceiveData.fromProtocolString`)
   - Starts with `C` → parse as config (`MqttConfig.fromProtocolString`)
   - Otherwise → check if matches device type identifier (e.g., `soccer`)

---

## 8. Summary Table

| Feature | Soccer Legacy | Soccer Config | Dashboard |
|---------|:---:|:---:|:---:|
| Device | Robot PILA | Robot PILA | micro:bit / Arduino |
| BLE Service | NUS or HM-10 | NUS or HM-10 | NUS or HM-10 |
| Config string | ❌ | `C,...` | `C,...` |
| Telemetry | `soccer,...` | `R,...` (if `O` in config) | `R,...` |
| Joystick | `J<angle>,<speed>` | `J<angle>,<speed>` | ❌ |
| Stop | `P0` | `P0` | ❌ |
| Spin | `C<speed>` | `C<speed>` | ❌ |
| LED | `Z<RRR><GGG><BBB>` | `Z<RRR><GGG><BBB>` | ❌ |
| Heading | `H0` | `H0` | ❌ |
| PID | `D<kp>,<ki>,<kd>` | `D<kp>,<ki>,<kd>` | ❌ |
| Button | `B1`/`B0` | `B1`/`B0` | ❌ |
| Toggle | ❌ | `T,<name>,<0\|1>,` | `T,<name>,<0\|1>,` |
| Slider | ❌ | `T,<name>,<value>,` | `T,<name>,<value>,` |
| Text Field | ❌ | `T,<name>,<text>,` | `T,<name>,<text>,` |
| UI | Hardcoded | Config-driven | Config-driven |
