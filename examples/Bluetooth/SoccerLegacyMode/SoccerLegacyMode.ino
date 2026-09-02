// Soccer Legacy Mode Example
// Uses fixed PILA commands (J, P, C, Z, H, B, T) without config handshake.
// Receives sensor commands and sends legacy telemetry directly after BLE connection.

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

// ── Button callback ───
void onKick(bool pressed) {
  if (pressed) {
    robot.onBoardLedSet(LEDWhite);
  } else {
    robot.onBoardLedSet(LEDOff);
  }
}

void setup() {
  robot.init();
  robot.motorConfiguration(M4, M3, M1, M2);
  robot.compassCorrectEnabled = true;

  robot.bluetoothInit(&Serial1, PILA_LEGACY);
  robot.bluetoothOnButton("LED", onKick);  // directly pass the function
}

void loop() {
  robot.bluetoothRemote();
}
