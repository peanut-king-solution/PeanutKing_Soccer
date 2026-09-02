#include "Bluetooth.h"

Bluetooth::Bluetooth() :
  _mode(PILA_LEGACY),
  _status(BLE_DISCONNECTED),
  _serial(&Serial1),
  _isConfigured(false)
{
  _rxBuffer.reserve(128); // Reserve space for the incoming data parser to avoid dynamic allocations
}
bool Bluetooth::init(HardwareSerial* port, RemoteMode mode)
{
  // Check if the provided serial port is valid
  if (port == nullptr) return false;

  _mode = mode; // Set the current mode of operation
  _status = BLE_DISCONNECTED;  // Set the current connection status
  _isConfigured = false;  // Set the configuration status to false

  // Initialize the serial port for communication with the Bluetooth module
  _serial = port; _serial->begin(BAUD_RATE); delay(100);

  // Flush any buffered data
  while (_serial->available()) _serial->read();
  // Test communication with the module
  if (!ping()) return false;
  // Enable notifications for connection events
  if (!setNotifications(true)) return false;

  // Keep any buffered OK+CONN for the first poll() instead of flushing it.
  return true;
}

String Bluetooth::sendATCommand(const char* cmd, uint32_t timeout)
{
  // Check if the serial port is initialized
  if (_serial == nullptr) return String();

  // Send the AT command to the Bluetooth module
  _serial->println(cmd);

  // Wait for a response from the module within the specified timeout
  String response;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (_serial->available()) {
      char c = _serial->read();
      if (c != '\r') response += c;
    }
  }
  // Trim any leading or trailing whitespace from the response
  response.trim();
  // Return the response received from the Bluetooth module
  return response;
}
bool Bluetooth::setNotifications(bool enable)
{
  String resp = sendATCommand(enable ? "AT+NOTI1" : "AT+NOTI0", 500);
  return resp.indexOf(enable ? "OK+Set:1" : "OK+Set:0") >= 0;
}
bool Bluetooth::rename(const char* name)
{
  // Check if the provided name is valid
  if (name == nullptr || strlen(name) == 0) return false;
  
  // Construct the AT command to rename the Bluetooth module
  char cmd[32]; snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);

  // Send the rename command and check for a successful response
  String resp = sendATCommand(cmd);
  if (resp.indexOf("OK+Set:") < 0) return false;

  // Verify the new name by querying the module
  resp = sendATCommand("AT+NAME?");
  return resp.indexOf("OK+Get:") >= 0;
}
bool Bluetooth::reset(void)
{
  String resp = sendATCommand("AT+RESET", 2000);
  return resp.indexOf("OK+RESET") >= 0;
}
bool Bluetooth::ping(void)
{
  String resp = sendATCommand("AT", 500);
  return resp.indexOf("OK") >= 0;
}

void Bluetooth::setMode(RemoteMode mode) { _mode = mode; }
RemoteMode Bluetooth::getMode(void) const { return _mode; }

bool Bluetooth::isConnected(void) const { return _status == BLE_CONNECTED; }
void Bluetooth::reconnect(void) { /* Soft reconnect - let the next OK+CONN elevate status. */ }


bool Bluetooth::isConfigured(void) {
  if (_mode == PILA_LEGACY) { 
    _isConfigured = true;
    return true;
  }
  return _isConfigured;
}
void Bluetooth::setConfig(const String& config) { 
  // _config = config; 
  _isConfigured = false; 
}

void Bluetooth::processFrame(const String& frame)
{
  // Check for configuration acknowledgment frame
  if (_rxParser.isConfigAck(frame)) {
    _isConfigured = true;
    return;
  }
  
  // Check for telemetry frame (T,<name>,<value>,<name>,<value>,...) to update input states
  if (frame.startsWith("T,")) {
    String input = frame.substring(2); // remove the "T," prefix
    int pairCount = 0, pos = 0;        // Count the number of <name>,<value> pairs in the input
    // Count the number of commas to determine the number of pairs
    while (pos < input.length()) {
      int commaPos = input.indexOf(',', pos); // Find the next comma in the input
      if (commaPos < 0) break;  // No more commas, exit loop
      pairCount++;  // Increment the pair count for each comma found
      pos = commaPos + 1; // Move past the comma for the next iteration
    }
    // Valid message: must have an even number of tokens (name,value pairs), "T," is not counted in pairCount.
    // Check if the number of pairs is valid (even number of tokens)
    if (pairCount > 0 && pairCount % 2 == 0) {
      pos = 0;
      // Process each <name>,<value> pair
      while (pos < input.length()) {
        // Find the next comma to extract the name
        int nameEnd = input.indexOf(',', pos);
        if (nameEnd < 0) break; // No more names, exit loop
        String name = input.substring(pos, nameEnd);
        pos = nameEnd + 1; // Move past the comma

        // Find the next comma to extract the value
        int valueEnd = input.indexOf(',', pos);
        String value;
        if (valueEnd < 0) {
          value = input.substring(pos); // Last value
          pos = input.length(); // Move to end
        } else {
          value = input.substring(pos, valueEnd);
          value.trim();
          pos = valueEnd + 1; // Move past the comma
        }

        // Update the state with the parsed name and value
        // setState(name, value); // not implemented, need to define how to handle the parsed name and value pairs
      }
    } else {
      // Invalid message format: log or handle the error as needed
      // Serial.println("Invalid T frame format: " + frame);
    }
  }
}
void Bluetooth::processSerial(void)
{
  // check if the serial port is valid
  if (_serial == nullptr) return;

  // Read all available characters from the serial port
  while (_serial->available()) {
    char c = _serial->read(); // Read a character from the serial 
    if (c != '\r') continue;  // Ignore carriage return characters
    _rxBuffer += c;           // Append character to buffer
  
    // Check for connection notifications (OK+CONN or OK+LOST)
    bool isNotification = false;
    if (_rxParser.isConnectionNotification(_rxBuffer)) {
      _status = BLE_CONNECTED;
      isNotification = true;
    } 
    else if (_rxParser.isDisconnectNotification(_rxBuffer)) {
      _status = BLE_DISCONNECTED;
      isNotification = true;
    }
    // handle connection notifications by resetting configuration status and clearing the buffer
    if (isNotification) {
      _isConfigured = false; // Reset configuration status on connection change
      _rxBuffer = ""; // Clear the buffer after processing the notification
      continue;       // Skip further processing for this frame
    }

    // Frame delimiter: process the frame if it's valid, then reset the buffer.
    if (c == '\n') {
      processFrame(_rxBuffer);  // Process the received frame
      _rxBuffer = ""; // Clear the buffer after processing the frame
      continue;       // Skip further processing for this frame
    }
  }
}

int Bluetooth::getSliderValue(const String& name) const
{
  return 0;
}
bool Bluetooth::getToggleState(const String& name) const
{
  return false;
}
void Bluetooth::onButton(const String& name, ButtonCallback callback)
{
  // Register a callback for button events (not implemented)
}
String Bluetooth::getTextFieldValue(const String& name) const
{
  return String();
}
JoystickState Bluetooth::getJoystick(const String& name) const
{
  return JoystickState();
}

void Bluetooth::setOutput(const String& name, int value) {}
void Bluetooth::setOutput(const String& name, float value) {}
void Bluetooth::setOutput(const String& name, bool value) {}