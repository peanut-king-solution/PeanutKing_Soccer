#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H
#include <Arduino.h>

enum InputComponentType : uint8_t {
  SliderType       = 0,
  JoystickType     = 1,
  ButtonType       = 2,
  ToggleButtonType = 3,
  TextFieldType    = 4
};
struct SliderInfo {
  const char* name;
  uint16_t minValue;
  uint16_t maxValue;
  uint16_t currentValue;
};
struct JoystickInfo {
  const char* joystickName;
  const char* angleName;
  const char* strengthName;
  uint16_t maxStrength;
  uint16_t currentStrength;
  uint16_t currentAngle;
};
struct ButtonInfo { 
  const char* name;
  bool currentStatus;
};
struct ToggleButtonInfo {
  const char* name;
  bool currentStatus;
};
struct TextFieldInfo { 
  const char* name;
  // String currentText;
};

struct InputComponent {
  InputComponentType type;
  union {
    SliderInfo slider;
    JoystickInfo joystick;
    ButtonInfo button;
    ToggleButtonInfo toggleButton;
    TextFieldInfo textField;
  } info;
};
struct OutputComponent {
  const char* name;
  bool plotableFlag;  // true = enable time graphing, false = display as value only
};

#endif // DATA_STRUCTS_H