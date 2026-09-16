// Soccer Config Mode Example
// Uses user-defined config with R telemetry and T widget commands.
// Sends config repeatedly until the app answers "Correct config received".

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;
static txDataPacker txPacker;

// ── Button callbacks (no name param needed) ─────────────────
void onClick1(bool pressed) {
  if (pressed) {
    robot.motorSetSpeed(RightFront, 200);
    robot.motorSetSpeed(LeftFront, 200);
  } else {
    robot.motorStopAll();
  }
}

void onClick2(bool pressed) {
  if (pressed) robot.onBoardLedSet(LEDRed);
  else robot.onBoardLedSet(LEDOff);
}

void setup() {
  robot.init();

  robot.bluetoothInit(&Serial1, PILA_CONFIG);

  // Build config using txDataPacker
  InputComponent inputs[] = {
    txPacker.makeSlider("Speed", 0, 255),
    txPacker.makeButton("Kick"),
    txPacker.makeButton("Shoot"),
    txPacker.makeToggleButton("Auto"),
    txPacker.makeJoystick("Move", "angle", "strength", 255),
  };
  OutputComponent outputs[] = {
    txPacker.makeGraph("compass", false),
    txPacker.makeGraph("ultrasound_front", false),
    txPacker.makeGraph("ball_angle", false),
  };

  String inputConfig = txPacker.buildInputConfigMessage(inputs, 5);
  String outputConfig = txPacker.buildOutputConfigMessage(outputs, 3);
  robot.bluetoothSetConfig(txPacker.buildConfigMessage(inputConfig, outputConfig));

  // Route commands by name - pass function directly
  robot.bluetoothOnButton("Motor", onClick1);
  robot.bluetoothOnButton("LED", onClick2);
}

void loop() {
  robot.bluetoothRemote();
  if (!robot.bluetoothGetToggle("Auto")) return;

  robot.dataFetch();
  robot.bluetoothSetOutput("compass", (int)robot.heading);
  robot.bluetoothSetOutput("ultrasound_front", (int)robot.distances[0]);
  robot.bluetoothSetOutput("ball_angle", (int)robot.irAngle);
}
