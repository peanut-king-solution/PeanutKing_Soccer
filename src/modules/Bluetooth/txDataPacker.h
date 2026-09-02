#ifndef TX_DATA_PACKER_H
#define TX_DATA_PACKER_H

#include <Arduino.h>
#include "dataStructs.h"

class txDataPacker
{
private:
  /* data */
public:
  // Constructor
  txDataPacker(/* args */);
  /**/
  InputComponent makeSlider(const char* name, uint16_t minValue, uint16_t maxValue);
  InputComponent makeJoystick(
    const char* joystickName, const char* angleName, const char* strengthName, uint16_t maxStrength
  );
  InputComponent makeButton(const char* name);
  InputComponent makeToggleButton(const char* name);
  InputComponent makeTextField(const char* name);
  /**/
  OutputComponent makeGraph(const char* name, bool plotableFlag = false);
  /**/
  String buildInputConfigMessage(const InputComponent* inputComponents = nullptr, size_t inputCount = 0);
  String buildOutputConfigMessage(const OutputComponent* outputComponents = nullptr, size_t outputCount = 0);
  String buildConfigMessage(const String& inputConfig, const String& outputConfig);
  /**/
  String buildSendMessage(const char* outputLabel, bool value);
  String buildSendMessage(const char* outputLabel, uint8_t value);
  String buildSendMessage(const char* outputLabel, uint16_t value);
  String buildSendMessage(const char* outputLabel, float value);
  String buildSendMessage(const char* outputLabel, const String& value);
};

#endif // TX_DATA_PACKER_H