// Dashboard Mode Example
// Uses dynamic app-defined dashboard with widgets, live telemetry and T commands.
// Dashboard has no built-in joystick or legacy commands.
// Config is built with txDataPacker for full widget control.

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;
static txDataPacker txPacker;

// ── Button callbacks ─────────────────────────────────────────
void onClick(bool pressed) {
  if (pressed) robot.onBoardLedSet(LEDCyan);
  else robot.onBoardLedSet(LEDOff);
}

void onToggle(bool pressed) {
  if (pressed) robot.onBoardLedSet(LEDRed);
  else robot.onBoardLedSet(LEDOff);
}

void setup() {
  robot.init();
  robot.bluetoothInit(&Serial1, DASHBOARD);

  robot.compassCorrectEnabled = true;

  // Build dashboard config (Dashboard needs txDataPacker for slider/textfield)
  static const int INPUT_COUNT = 5;
  static const int OUTPUT_COUNT = 3;
  InputComponent inputs[INPUT_COUNT] = {
    txPacker.makeSlider("Speed", 0, 255),
    txPacker.makeButton("Click"),
    txPacker.makeToggleButton("Toggle"),
    txPacker.makeJoystick("Move", 255),
    txPacker.makeTextField("Message"),
  };
  OutputComponent outputs[OUTPUT_COUNT] = {
    txPacker.makeGraph("temp", true),
    txPacker.makeGraph("compass", true),
    txPacker.makeGraph("ultrasoundFront", false),
  };

  String inputConfig = txPacker.buildInputConfigMessage(inputs, INPUT_COUNT);
  String outputConfig = txPacker.buildOutputConfigMessage(outputs, OUTPUT_COUNT);
  robot.bluetoothSetConfig(txPacker.buildConfigMessage(inputConfig, outputConfig));

  robot.bluetoothOnButton("Click", onClick);
  // Toggle button should not use callbacks, but rather poll the state
}

uint32_t lastSensorSendTime = 0;

void loop() {
  robot.bluetoothRemote();

  // Joystick
  JoystickState joy = robot.bluetoothGetJoystick("Move");
  robot.move(joy.angle, joy.strength * 255/100);  // Scale strength to 0-255

  // LED toggle
  robot.onBoardLedSet(robot.bluetoothGetToggle("Toggle") ? LEDWhite : LEDOff);

  // TextField
  String msg = robot.bluetoothGetTextField("Message");
  if (msg.length() > 0) {
    Serial.print(F("[BLE][MSG] "));
    Serial.println(msg);
  }
  // Slider
  int speed = robot.bluetoothGetSlider("Speed");
  if (speed != 0) {
    Serial.print(F("[BLE][SLIDER] Speed = "));
    Serial.println(speed);
  }

  // Send sensor data every 100ms only if the config is sent
  if (robot.bluetoothIsConfig() && millis() - lastSensorSendTime >= 100) {
    robot.dataFetch();
    robot.bluetoothSendOutput("temp", (float)24.5);
    robot.bluetoothSendOutput("compass", (int)robot.heading);
    robot.bluetoothSendOutput("ultrasoundFront", (int)robot.distances[0]);
    lastSensorSendTime = millis();
  }
}
