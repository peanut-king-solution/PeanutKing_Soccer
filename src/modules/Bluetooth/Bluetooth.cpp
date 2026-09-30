#include "Bluetooth.h"

Bluetooth::Bluetooth() :
  _mode(PILA_LEGACY),
  _status(BLE_DISCONNECTED),
  _serial(&Serial1),
  _isConfigured(false),
  _stateCount(0),
  _cmdQueue(CMD_QUEUE_MAX),
  _buttonHandlerCount(0)
{
  _rxBuffer.reserve(128); // Reserve space for the incoming data parser to avoid dynamic allocations
  _cmdQueue.Clear();
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
  if (config.length() == 0 || _mode != DASHBOARD) return;
  _config = config;
  _isConfigured = false;
  _lastConfigSendMs = 0;
}
void Bluetooth::sendConfig(const uint32_t intervalMs) {
  // Auto-build config from registered widgets (Soccer Config mode only)
  if (_config.length() == 0 && !_configBuilt && _mode == PILA_CONFIG
      && (_hasSoccerButton || _hasSoccerToggle || _configOutputCount > 0)) {
    _config = _buildAutoConfig();
    _configBuilt = true;
  }
  if (_config.length() == 0) return; // No config to send
  uint32_t now = millis();
  if (now - _lastConfigSendMs >= intervalMs) {
    _serial->print(_config);
    _lastConfigSendMs = now;
  }
}

// ============================================================================
//                           Config auto-builder
// ============================================================================

void Bluetooth::_addButton(const char* name) {
  if (strcmp(name, "SoccerBtn") == 0) {
    _hasSoccerButton = true;
    _hasSoccerToggle = false;  // Only one action widget at a time (normal button wins)
  }
}

void Bluetooth::_addToggle(const char* name) {
  if (strcmp(name, "SoccerTog") == 0) {
    _hasSoccerToggle = true;
    _hasSoccerButton = false;  // Only one action widget at a time
  }
}

void Bluetooth::_addOutput(const char* name) {
  if (_configOutputCount >= CONFIG_OUTPUT_MAX) return;
  _configOutputs[_configOutputCount] = name;
  _configOutputCount++;
}

String Bluetooth::_buildAutoConfig(void) {
  // Soccer Config auto-build: C,O,<count>,<outputs...>,<B|TB,name>,\n
  // Outputs are built via txDataPacker; action widget depends on user registration.
  String cfg = "C,O," + String(_configOutputCount);
  for (uint8_t i = 0; i < _configOutputCount; i++) {
    cfg += "," + _configOutputs[i] + ",false";
  }
  // Action widget: only one of B (normal) or TB (toggle) at the end
  if (_hasSoccerButton)       cfg += ",B,SoccerBtn";
  else if (_hasSoccerToggle)  cfg += ",TB,SoccerTog";
  cfg += "\n";
  return cfg;
}

// ============================================================================
//                           State helpers
// ============================================================================

void Bluetooth::_setState(const String& name, const String& value) {
  // Update existing
  for (uint8_t i = 0; i < _stateCount; i++) {
    if (_states[i].name == name) { _states[i].value = value; return; }
  }
  // Add new
  if (_stateCount < STATE_MAX) {
    _states[_stateCount].name = name;
    _states[_stateCount].value = value;
    _stateCount++;
  }
}

String Bluetooth::_getState(const String& name) const {
  for (uint8_t i = 0; i < _stateCount; i++) {
    if (_states[i].name == name) return _states[i].value;
  }
  return String();
}

void Bluetooth::_processTelemetry(const String& frame) {
  if (!frame.startsWith("T,")) return;

  String parts[32];
  int n = 0, pos = 0;
  // Split the frame into parts using commas as delimiters, up to a maximum of 32 parts
  while (pos <= (int)frame.length() && n < 32) {
    int comma = frame.indexOf(',', pos);
    String tok = (comma < 0) ? frame.substring(pos) : frame.substring(pos, comma);
    tok.trim();
    if (tok.length() > 0) parts[n++] = tok;
    if (comma < 0) break;
    pos = comma + 1;
  }

  // Ensure there are at least 3 parts (T,<name>,<value>) and that the number of parts is odd (pairs of name/value)
  if (n < 3 || (n % 2) == 0) return;

  for (int i = 1; i + 1 < n; i += 2) {
    // Update state with the new value
    _setState(parts[i], parts[i + 1]);
    // Fire button callbacks if registered (Dashboard mode only)
    for (uint8_t h = 0; h < _buttonHandlerCount; h++) {
      if (_buttonHandlers[h].name == parts[i]) {  // Match button name
        _buttonHandlers[h].callback(parts[i + 1] == "1");
      }
    }
  }
}
void Bluetooth::_processLegacyCmd(const String& frame) {
  RxCommand cmd;
  if (_rxParser.parseCommand(frame, cmd)) {
    // Fire button callbacks immediately (before queue)
    if (cmd.type == CMD_BUTTON) {
      bool pressed = (cmd.first == 1);
      for (uint8_t i = 0; i < _buttonHandlerCount; i++) {
        _buttonHandlers[i].callback(pressed);
      }
    }
    // Enqueue the command for later processing
    _cmdQueue.Push(cmd);
  }
}
void Bluetooth::_processFrame(const String& frame) {
  if (frame.startsWith("T,")) {
    _processTelemetry(frame);
  } else {
    _processLegacyCmd(frame);
  }
}
void Bluetooth::processSerial(void)
{
  // check if the serial port is valid
  if (_serial == nullptr) return;

  // Read all available characters from the serial port
  while (_serial->available()) {
    char c = _serial->read(); // Read a character from the serial
    if (c == '\r') continue;  // Ignore carriage return characters
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
      // Config acknowledgment from the app: mark module as configured
      if (_rxParser.isConfigAck(_rxBuffer)) {
        _isConfigured = true;
        Serial.println(F("[BLE][ACK] Correct config received"));
        _rxBuffer = ""; // Clear the buffer after processing the acknowledgment
        continue;       // Skip further processing for this frame
      }
      // For debugging: print the received frame
      // Serial.print(F("[BLE][RX] "));
      // Serial.println(_rxBuffer);
      
      _processFrame(_rxBuffer);  // Process the received frame
      _rxBuffer = ""; // Clear the buffer after processing the frame
      continue;       // Skip further processing for this frame
    }
  }
}

int Bluetooth::getSliderValue(const String& name) const
{
  String v = _getState(name);
  return v.length() ? v.toInt() : 0;
}
bool Bluetooth::getToggleState(const String& name) const
{
  return _getState(name) == "1";
}
void Bluetooth::onButton(const String& name, ButtonCallback callback)
{
  // Legacy / Config mode: only store the first callback (fixed single-button UI)
  if (_mode != DASHBOARD) {
    if (_buttonHandlerCount == 0) {
      _buttonHandlers[0].name = name;
      _buttonHandlers[0].callback = callback;
      _buttonHandlerCount = 1;
    }
    return;
  }
  // Dashboard mode: multiple named callbacks
  if (_buttonHandlerCount < BTN_MAX) {
    _buttonHandlers[_buttonHandlerCount].name = name;
    _buttonHandlers[_buttonHandlerCount].callback = callback;
    _buttonHandlerCount++;
  }
}
String Bluetooth::getTextFieldValue(const String& name) const
{
  return _getState(name);
}
JoystickState Bluetooth::getJoystick(const String& name) const
{
  JoystickState js;
  String angleKey = name + "Ang";
  String strengthKey = name + "Str";
  js.angle = _getState(angleKey).toInt();
  js.strength = _getState(strengthKey).toInt();
  return js;
}

// ============================================================================
//                            Command queue
// ============================================================================

bool Bluetooth::hasCommand(void) const { return !_cmdQueue.IsEmpty(); }

RxCommand Bluetooth::getCommand(void)
{
  RxCommand cmd = *_cmdQueue.Front();
  _cmdQueue.Pop();
  return cmd;
}

// ============================================================================
//                            Output senders
// ============================================================================

void Bluetooth::sendOutput(const String& name, int value)
{
  _serial->print(_txPacker.buildSendMessage(name.c_str(), (float)value));
}
void Bluetooth::sendOutput(const String& name, float value)
{
  _serial->print(_txPacker.buildSendMessage(name.c_str(), value));
}
void Bluetooth::sendOutput(const String& name, bool value)
{
  _serial->print(_txPacker.buildSendMessage(name.c_str(), value));
}

// ============================================================================
//                            Raw send
// ============================================================================

void Bluetooth::sendRaw(const String& data) {
  _serial->print(data);
}