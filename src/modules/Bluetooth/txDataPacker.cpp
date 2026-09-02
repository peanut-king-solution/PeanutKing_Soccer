#include "txDataPacker.h"

static const char* inputComponentLabels[] = {
  "S", // Slider
  "J", // Joystick
  "B", // Button
  "TB", // Toggle Button
  "TF"  // TextField
};

txDataPacker::txDataPacker(/* args */)
{
}

InputComponent txDataPacker::makeSlider(
  const char* name, uint16_t minValue, uint16_t maxValue
) {
  InputComponent component;
  component.type = SliderType;
  component.info.slider.name = name;
  component.info.slider.minValue = minValue;
  component.info.slider.maxValue = maxValue;
  component.info.slider.currentValue = minValue; // Initialize current value to minValue
  return component;
}
InputComponent txDataPacker::makeJoystick(
  const char* joystickName, const char* angleName, const char* strengthName, uint16_t maxStrength
) {
  InputComponent component;
  component.type = JoystickType;
  component.info.joystick.joystickName = joystickName;
  component.info.joystick.angleName = angleName;
  component.info.joystick.strengthName = strengthName;
  component.info.joystick.maxStrength = maxStrength;
  component.info.joystick.currentStrength = 0; // Initialize current strength to 0
  component.info.joystick.currentAngle = 0;    // Initialize current angle to 0
  return component;
}
InputComponent txDataPacker::makeButton(const char* name) {
  InputComponent component;
  component.type = ButtonType;
  component.info.button.name = name;
  component.info.button.currentStatus = false; // Initialize current status to false
  return component;
}
InputComponent txDataPacker::makeToggleButton(const char* name) {
  InputComponent component;
  component.type = ToggleButtonType;
  component.info.toggleButton.name = name;
  component.info.toggleButton.currentStatus = false; // Initialize current status to false
  return component;
}
InputComponent txDataPacker::makeTextField(const char* name) {
  InputComponent component;
  component.type = TextFieldType;
  component.info.textField.name = name;
  // component.info.textField.currentText = ""; // Initialize current text to empty string
  return component;
}

OutputComponent txDataPacker::makeGraph(const char* name, bool plotableFlag) {
  OutputComponent component;
  component.name = name;
  component.plotableFlag = plotableFlag;
  return component;
}

String txDataPacker::buildInputConfigMessage(
  const InputComponent* inputComponents, size_t inputCount
) {
  // Start building the input configuration message with the prefix "I" and the input count
  String message = "I," + String(inputCount);
  // Loop through each input component and append its details to the message
  for (size_t i = 0; i < inputCount; ++i) {
    // Check if inputComponents is nullptr to avoid dereferencing a null pointer
    if (inputComponents == nullptr) { break; }
    // Get the current input component and its type index
    const InputComponent& comp = inputComponents[i];
    const uint8_t typeIndex = static_cast<uint8_t>(comp.type);
    // Append the input component type label and its details to the message
    message += ","; message += inputComponentLabels[typeIndex];
    // Append component-specific details based on its type
    switch (comp.type) {
      case SliderType:
        message += ","; message += String(comp.info.slider.minValue);
        message += ","; message += String(comp.info.slider.maxValue);
         message += ","; message += comp.info.slider.name;
        break;
      case JoystickType:
        message += ","; message += comp.info.joystick.angleName;
        message += ","; message += String(comp.info.joystick.maxStrength);
        message += ","; message += comp.info.joystick.strengthName;
        message += ","; message += comp.info.joystick.joystickName;
        break;
      case ButtonType:
        message += ","; message += comp.info.button.name;
        break;
      case ToggleButtonType:
        message += ","; message += comp.info.toggleButton.name;
        break;
      case TextFieldType:
        message += ","; message += comp.info.textField.name;
        break;
    }
  }
  // Return the complete input configuration message
  return message;
}
String txDataPacker::buildOutputConfigMessage(
  const OutputComponent* outputComponents, size_t outputCount
) {
  // Start building the output configuration message with the prefix "O" and the output count
  String message = "O," + String(outputCount);
  // Loop through each output component and append its details to the message
  for (size_t i = 0; i < outputCount; ++i) {
    // Check if outputComponents is nullptr to avoid dereferencing a null pointer
    if (outputComponents == nullptr) { break; }
    // Append the output component name and plotable flag to the message
    message += ","; message += outputComponents[i].name;
    message += ","; message += outputComponents[i].plotableFlag ? "true" : "false";
  }
  // Return the complete output configuration message
  return message;
}
String txDataPacker::buildConfigMessage(
  const String& inputConfig, const String& outputConfig
) {
  // Start building the configuration message with the prefix "C"
  String message = "C,";
  // Append input component details
  message += inputConfig;
  // Append a comma to separate input and output components
  message += ",";
  // Append output component details
  message += outputConfig;
  // Append a newline character at the end of the message
  message += ",\n"; 
  // Return the complete configuration message
  return message;
}


String txDataPacker::buildSendMessage(const char* outputLabel, uint8_t value) {
  // Start building the data send message with the prefix "R"
  String message = "R";
  // Append the output label and value to the message
  message += ","; message += outputLabel;
  message += ","; message += String(value);
  // Append a newline character at the end of the message
  message += ",\n"; return message;
}
String txDataPacker::buildSendMessage(const char* outputLabel, uint16_t value) {
  // Start building the data send message with the prefix "R"
  String message = "R";
  // Append the output label and value to the message
  message += ","; message += outputLabel;
  message += ","; message += String(value);
  // Append a newline character at the end of the message
  message += ",\n"; return message;
}
String txDataPacker::buildSendMessage(const char* outputLabel, float value) {
  // Start building the data send message with the prefix "R"
  String message = "R";
  // Append the output label and value to the message
  message += ","; message += outputLabel;
  message += ","; message += String(value);
  // Append a newline character at the end of the message
  message += ",\n"; return message;
}
String txDataPacker::buildSendMessage(const char* outputLabel, bool value) {
  // Start building the data send message with the prefix "R"
  String message = "R";
  // Append the output label and value to the message
  message += ","; message += outputLabel;
  message += ","; message += value ? "true" : "false";
  // Append a newline character at the end of the message
  message += ",\n"; return message;
}
String txDataPacker::buildSendMessage(const char* outputLabel, const String& value) {
  String message = "R";
  message += ","; message += outputLabel;
  message += ","; message += value;
  message += ",\n"; return message;
}
 