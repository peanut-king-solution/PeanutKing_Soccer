#ifndef RX_DATA_PARSER_H
#define RX_DATA_PARSER_H

#include <Arduino.h>

// PILA remote control command codes
// T is shared by buttons, toggles, sliders, joysticks and text fields.
enum PILA_RX_Cmd
{
  CMD_CIRCLE   = 'C',  // C<speed>
  CMD_LIGHT    = 'Z',  // Z<R><G><B> (0~255)
  CMD_PAUSE    = 'P',  // P0 = pause
  CMD_JOYSTICK = 'J',  // J<angle>,<speed>
  CMD_SET_PID  = 'D',  // D<kp>,<ki>,<kd>
  CMD_COMPASS  = 'H',  // H0 = reset compass heading
  CMD_BUTTON   = 'B',  // Legacy B1/B0 compatibility
  CMD_TOGGLE   = 'T',  // T,<name>,<value>,
};

struct RxCommand
{
  PILA_RX_Cmd type;
  char name[16];
  String value;
  int16_t first;
  int16_t second;
};

class rxDataParser
{
public:
  rxDataParser();
  PILA_RX_Cmd parseCommandType(const String& frame);
  bool parseCommand(const String& frame, RxCommand& command);
  bool isConnectionNotification(const String& frame);
  bool isDisconnectNotification(const String& frame);
  bool isConfigAck(const String& frame);

  int16_t decodeMotor(const String& value) const;
  void copyName(char* destination, const String& source);
};

#endif // RX_DATA_PARSER_H
