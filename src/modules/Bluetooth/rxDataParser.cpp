#include "rxDataParser.h"

rxDataParser::rxDataParser() {}

namespace {
// Strict decimal token parse: whole token must be a number within range.
bool parseDecimal(const String& token, long min, long max, int16_t& out)
{
  if (token.length() == 0) return false;
  const char* s = token.c_str();
  char* end = nullptr;
  long value = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false; // junk after digits
  if (value < min || value > max) return false;
  out = static_cast<int16_t>(value);
  return true;
}
} // namespace

PILA_RX_Cmd rxDataParser::parseCommandType(const String& frame)
{
  String command = frame;
  command.trim();
  if (command.length() == 0) return CMD_PAUSE;

  switch (command.charAt(0)) {
    case 'C': return CMD_CIRCLE;
    case 'Z': return CMD_LIGHT;
    case 'P': return CMD_PAUSE;
    case 'J': return CMD_JOYSTICK;
    case 'D': return CMD_SET_PID;
    case 'H': return CMD_COMPASS;
    case 'B': return CMD_BUTTON;
    case 'T': return CMD_TOGGLE;
    default: return CMD_PAUSE;
  }
}

void rxDataParser::copyName(char* destination, const String& source)
{
  source.substring(0, 15).toCharArray(destination, 16);
}

bool rxDataParser::parseCommand(const String& frame, RxCommand& command)
{
  String input = frame;
  input.trim();
  command.type = parseCommandType(input);
  command.name[0] = '\0';
  command.value = "";
  command.first = 0;
  command.second = 0;

  switch (command.type) {
    case CMD_JOYSTICK: {
      int comma = input.indexOf(',');
      if (comma <= 1) return false;
      int16_t angle, speed;
      if (!parseDecimal(input.substring(1, comma), 0, 360, angle)) return false;
      if (!parseDecimal(input.substring(comma + 1), 0, 255, speed)) return false;
      command.first = angle;
      command.second = speed;
      return true;
    }
    case CMD_CIRCLE: {
      if (input.length() < 2) return false;
      int16_t speed;
      // Encoded motor range: 0-255 forward, 645-1155 maps to -255..-1.
      if (!parseDecimal(input.substring(1), 0, 1155, speed)) return false;
      speed = decodeMotor(String(speed));
      command.first = speed;
      return true;
    }
    case CMD_LIGHT: {
      if (input.length() != 10) return false;
      int16_t r, g, b;
      if (!parseDecimal(input.substring(1, 4), 0, 255, r)) return false;
      if (!parseDecimal(input.substring(4, 7), 0, 255, g)) return false;
      if (!parseDecimal(input.substring(7, 10), 0, 255, b)) return false;
      command.first = r;
      command.second = g;
      command.value = input.substring(7, 10);
      return true;
    }
    case CMD_PAUSE:
      return input == "P0";
    case CMD_COMPASS:
      return input == "H0";
    case CMD_BUTTON:
      return input == "B1" || input == "B0";
    case CMD_SET_PID: {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      if (firstComma <= 1 || secondComma <= firstComma + 1) return false;
      int16_t kp, ki;
      if (!parseDecimal(input.substring(1, firstComma), -32768, 32767, kp)) return false;
      if (!parseDecimal(input.substring(firstComma + 1, secondComma), -32768, 32767, ki)) return false;
      command.first = kp;
      command.second = ki;
      command.value = input.substring(secondComma + 1);
      return command.value.length() > 0;
    }
    case CMD_TOGGLE: {
      // Multi-pair T frames are handled by Bluetooth::processFrame() before
      // reaching here; this validates the single-pair form for direct use.
      if (!input.startsWith("T,")) return false;
      int firstComma = input.indexOf(',', 2);
      if (firstComma <= 2) return false;
      int secondComma = input.indexOf(',', firstComma + 1);
      if (secondComma < 0) secondComma = input.length();
      if (secondComma <= firstComma + 1) return false;
      copyName(command.name, input.substring(2, firstComma));
      command.value = input.substring(firstComma + 1, secondComma);
      return command.value.length() > 0;
    }
    default:
      return false;
  }
}

bool rxDataParser::isConnectionNotification(const String& frame)
{
  // Trim whitespace
  String value = frame; value.trim();
  // Return true if the frame is a connection notification
  return value == "OK+CONN";
}
bool rxDataParser::isDisconnectNotification(const String& frame)
{
  // Trim whitespace
  String value = frame; value.trim();
  // Return true if the frame is a disconnection notification
  return value == "OK+LOST";
}

bool rxDataParser::isConfigAck(const String& frame)
{
  // Trim whitespace
  String value = frame; 
  value.trim();
  // Return true if the frame is a configuration acknowledgment
  return value == "Correct config received";
}

int16_t rxDataParser::decodeMotor(const String& value) const
{
  int16_t decoded = static_cast<int16_t>(value.toInt());
  // Encoded reverse speeds arrive as 645..1155 and map to -255..-1;
  // anything else in 0..255 is already a plain forward speed.
  if (decoded >= 645 && decoded <= 1155) return decoded - 900;
  return decoded;
}
