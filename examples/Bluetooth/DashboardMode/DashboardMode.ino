// Dashboard Mode Example
// Uses dynamic app-defined dashboard with widgets, live telemetry and T commands.

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;
static txDataPacker txPacker;

// ── Button callbacks (no name param needed) ─────────────────
void onKick(bool pressed) {
  if (pressed) robot.onBoardLedSet(LEDCyan);
  else robot.onBoardLedSet(LEDOff);
}

void onShoot(bool pressed) {
  if (pressed) robot.onBoardLedSet(LEDRed);
  else robot.onBoardLedSet(LEDOff);
}

void setup() {
  robot.init();
  robot.motorConfiguration(M4, M3, M1, M2);
  robot.compassCorrectEnabled = true;

  robot.bluetoothInit(&Serial1, DASHBOARD);

  // Build dashboard config
  InputComponent inputs[] = {
    txPacker.makeSlider("Speed", 0, 255),
    txPacker.makeButton("Kick"),
    txPacker.makeButton("Shoot"),
    txPacker.makeToggleButton("LED"),
    txPacker.makeJoystick("Move", "angle", "strength", 255),
    txPacker.makeTextField("Message"),
  };
  OutputComponent outputs[] = {
    txPacker.makeGraph("temp", true),
    txPacker.makeGraph("compass", true),
    txPacker.makeGraph("ultrasound_front", false),
  };

  String inputConfig = txPacker.buildInputConfigMessage(inputs, 6);
  String outputConfig = txPacker.buildOutputConfigMessage(outputs, 3);
  robot.bluetoothSetConfig(txPacker.buildConfigMessage(inputConfig, outputConfig));

  // Route commands by name - pass function directly
  robot.bluetoothOnButton("Kick", onKick);
  robot.bluetoothOnButton("Shoot", onShoot);
}

void loop() {
  robot.bluetoothRemote();

  // Joystick
  JoystickState joy = robot.bluetoothGetJoystick("Move");
  robot.move(joy.angle, joy.strength, 0);

  // LED toggle
  robot.onBoardLedSet(robot.bluetoothGetToggle("LED") ? LEDWhite : LEDOff);

  // Live telemetry
  robot.dataFetch();
  robot.bluetoothSetOutput("temp", (float)24.5);
  robot.bluetoothSetOutput("compass", (int)robot.heading);
  robot.bluetoothSetOutput("ultrasound_front", (int)robot.distances[0]);
}
