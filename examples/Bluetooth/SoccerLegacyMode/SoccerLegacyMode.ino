// Soccer Legacy Mode Example
// Uses fixed PILA commands (J, P, C, Z, H, B, T) without config handshake.
// Receives sensor commands and sends legacy telemetry directly after BLE connection.

#include <PeanutKingSoccerV4.h>

static PeanutKingSoccerV4 robot;

// ── Button callback ───
void onClick(bool pressed) {
  if (pressed) {
    robot.onBoardLedSet(LEDWhite);
  } else {
    robot.onBoardLedSet(LEDOff);
  }
}

void setup() {
  robot.init();

  robot.bluetoothInit(&Serial1, PILA_LEGACY);
  robot.bluetoothOnButton("LED", onClick);  // directly pass the function

  robot.compassCorrectEnabled = true;  // enable compass correction for movement
}

void loop() {
  robot.bluetoothRemote();

  // the data is sent automatically every 1 second in legacy mode
}
