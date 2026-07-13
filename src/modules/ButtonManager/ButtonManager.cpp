#include "ButtonManager.h"

ButtonManager::ButtonManager() :
  buttonPin{22, 23, 24, 25},
  btnStatus{NONE, NONE, NONE, NONE}
{
}

void ButtonManager::init(void)
{
  // Initialize button pins as INPUT_PULLUP
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }
}

bool ButtonManager::read(BUTTON_ID btn)
{
  uint8_t index = static_cast<uint8_t>(btn) - 1;
  if (index < 4) {
    return !digitalRead(buttonPin[index]);
  }
  return false;
}

void ButtonManager::update(void)
{
  // TODO: Implement the button state machine logic here to detect TAP, PRESS, HOLD, etc.

  // static uint32_t holdTimer[4] = {0};
  // uint32_t currentTime = millis();

  // for (uint8_t i = 0; i < 4; i++) {
  //   bool b = !digitalRead(buttonPin[i]);

  //   // Update button state machine based on the current button state and the raw button reading
  //   if (b) {
  //     switch (btnStatus[i])
  //     {
  //       case NONE:  btnStatus[i] = TAP; holdTimer[i] = currentTime; break;
  //       case TAP:   btnStatus[i] = PRESS; break;
  //       case TAP2:  if (currentTime - holdTimer[i] > HOLD_DURATION) btnStatus[i] = HOLD2; break;
  //       case TAP3:  if (currentTime - holdTimer[i] > HOLD_DURATION) btnStatus[i] = RELEASE; break;
  //       case PRESS: if (currentTime - holdTimer[i] > HOLD_DURATION) btnStatus[i] = HOLD; break;
  //       case TAP1_W:  holdTimer[i] = currentTime; btnStatus[i] = TAP2; break;
  //       case TAP2_W:  holdTimer[i] = currentTime; btnStatus[i] = TAP3; break;
  //       case TAP3_W:  if (currentTime - holdTimer[i] > HOLD_DURATION) btnStatus[i] = RELEASE; break;
  //       case RELEASE:
  //       case RELEASE_S:
  //       case RELEASE_L: btnStatus[i] = TAP; break;
  //       case HOLD: break;
  //       default: break;
  //     }
  //   }
  //   else {
  //     switch (btnStatus[i])
  //     {
  //       case TAP:   btnStatus[i] = TAP1_W; holdTimer[i] = currentTime; break;
  //       case TAP2:  btnStatus[i] = TAP2_W; holdTimer[i] = currentTime; break;
  //       case TAP3:  btnStatus[i] = RELEASE; holdTimer[i] = currentTime; break;
  //       case PRESS: btnStatus[i] = RELEASE_S; break;
  //       case TAP1_W: if (currentTime - holdTimer[i] > WAIT_DURATION) btnStatus[i] = RELEASE_S; break;
  //       case HOLD:  btnStatus[i] = RELEASE_L; break;
  //       case TAP2_W: if (currentTime - holdTimer[i] > WAIT_DURATION) btnStatus[i] = TAP2_R; break;
  //       case TAP3_W: if (currentTime - holdTimer[i] > WAIT_DURATION) btnStatus[i] = TAP3_R; break;
  //       case RELEASE:
  //       case RELEASE_S:
  //       case RELEASE_L:
  //       case TAP2_R:
  //       case TAP3_R: btnStatus[i] = NONE; break;
  //       default: btnStatus[i] = NONE; break;
  //     }
  //   }
  // }
}

buttonStatus_t ButtonManager::getStatus(BUTTON_ID btn)
{
  uint8_t index = static_cast<uint8_t>(btn) - 1;
  if (index < 4) {
    return btnStatus[index];
  }
  return NONE;
}