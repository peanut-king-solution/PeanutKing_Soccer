#include "BLEManager.h"

BLEManager::BLEManager() :
  _mode(RemoteMode::PILA),        // Default mode: PILA
  _status(BLEStatus::DISCONNECTED), // Default status: DISCONNECTED
  _serial(&Serial1)              // Default: use Serial1
{
}

// ============================================================================
//                              Configuration
// ============================================================================

bool BLEManager::setSerial(HardwareSerial* port)
{
  if (_serial == port) { return true; }      // No change
  else if (port == nullptr) { return false; } // Invalid port
  else {  // If a different port is set, end the previous one
    _serial->end();
  }
  _serial = port; // Set the new serial port
  return true;
}

bool BLEManager::init(uint32_t baudRate, RemoteMode mode)
{
  // Set the remote mode (PILA or DASHBOARD)
  _mode = mode;
  // Begin serial communication with the BLE module
  _serial->begin(baudRate);
  delay(100);

  // Ping the BLE module to check if it's responsive
  if (!ping()) {
    Serial.println("[BLE] Ping failed");
    return false;
  }

  // Enable notifications for connection status changes
  if (!setNotifications(true)) {
    Serial.println("[BLE] Failed to enable notifications");
    return false;
  }

  // Flush any leftover data
  while (_serial->available()) { _serial->read(); }
  return true;
}

// =============================================================================
//                              AT Commands
// =============================================================================

String BLEManager::sendATCommand(const char* cmd, uint32_t timeout)
{
  // Send command to BLE module
  _serial->println(cmd);
  Serial.print("[TX] ");
  Serial.println(cmd);

  // Wait for response with timeout
  String response;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    // Check if data is available to read
    if (_serial->available()) {
      // Read a single character and append to response
      char c = _serial->read();
      response += c;
    }
    // Small delay to let more data arrive
    delay(1);
  }

  // Trim whitespace and return the response
  response.trim();
  if (response.length() > 0) {
    Serial.print("[RX] ");
    Serial.println(response);
  }
  return response;
}

bool BLEManager::getBasicInfo(void)
{
  Serial.println("--- Module Info ---");

  // Query module information: NAME, UUID, CHAR, BAUD, ROLE, VERSION, ADDR
  String nameResp    = sendATCommand("AT+NAME?");
  String uuidResp    = sendATCommand("AT+UUID?");
  String charResp    = sendATCommand("AT+CHAR?");
  String baudResp    = sendATCommand("AT+BAUD?");
  String roleResp    = sendATCommand("AT+ROLE?");
  String versionResp = sendATCommand("AT+VERS?");
  String addrResp    = sendATCommand("AT+ADDR?");

  Serial.println("-------------------");

  // Return true if at least one query got a response
  return (nameResp.length() > 0);
}

bool BLEManager::setNotifications(bool enable)
{
  // Enable or disable notifications for connection status changes
  String cmd = enable ? "AT+NOTI1" : "AT+NOTI0";
  String resp = sendATCommand(cmd.c_str(), 500);
  return (resp.indexOf(enable ? "OK+Set:1" : "OK+Set:0") >= 0);
}

bool BLEManager::rename(const char* name)
{
  // Validate input name
  if (name == nullptr || strlen(name) == 0) return false;

  Serial.println("-------------------");
  Serial.print("Renaming BLE module to: ");
  Serial.println(name);

  // Build rename command: AT+NAME{name}
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);

  // Send the rename command and check for "OK+Set:" response
  String resp = sendATCommand(cmd);
  if (resp.indexOf("OK+Set:") < 0) {
    return false;
  }

  // Verify the name was set
  resp = sendATCommand("AT+NAME?");
  Serial.print("Current BLE name: ");
  Serial.println(resp);
  Serial.println("-------------------");
  return (resp.indexOf("OK+Get:") >= 0);
}

bool BLEManager::reset(void)
{
  Serial.println("-------------------");
  Serial.println("Resetting BLE module...");

  // Send AT+RESET command and wait for OK+RESET response
  String resp = sendATCommand("AT+RESET", 2000);

  Serial.println("-------------------");
  return (resp.indexOf("OK+RESET") >= 0);
}

bool BLEManager::ping(void)
{
  Serial.println("-------------------");
  Serial.println("Pinging BLE module...");

  // Send AT command and wait for OK response
  String resp = sendATCommand("AT", 500);

  Serial.println("-------------------");
  return (resp.indexOf("OK") >= 0);
}

// ============================================================================
//                             Connection
// ============================================================================

void BLEManager::setMode(RemoteMode mode) { _mode = mode; }

RemoteMode BLEManager::getMode(void) { return _mode; }

bool BLEManager::isConnected(void) { return (_status == BLEStatus::CONNECTED); }

bool BLEManager::checkConnection(void)
{
  // Check for connection status notifications from the BLE module
  if (_serial->available()) {
    // Peek at the first character to see if it's 'O' (start of "OK+...")
    if (_serial->peek() == 'O') {
      // Read the line from the serial buffer
      String line = _serial->readStringUntil('\n');
      line.trim();  // Remove whitespace and newline characters
      
      // Check for connection status notifications
      if (line.indexOf("OK+CONN") >= 0) {
        _status = BLEStatus::CONNECTED;
      } else if (line.indexOf("OK+LOST") >= 0) {
        _status = BLEStatus::DISCONNECTED;
      }
      return true; // Notification handled, done for this loop
    }
  }
  return false; // No notification received
}

void BLEManager::handleConnection(void)
{
  // no action needed; just log the connection
  if (_status == BLEStatus::CONNECTED) {
    Serial.println("[BLE] Connected");
  }
  // If disconnected, try to reconnect
  else if (_status == BLEStatus::DISCONNECTED) {
    Serial.println("[BLE] Disconnected");
    reconnect();
  }
}

void BLEManager::reconnect(void)
{
  // TODO: Implement reconnection logic
}

bool BLEManager::sendData(const String& data)
{
  if (!isConnected()) {
    Serial.println("[BLE] Not connected, cannot send data");
    return false;
  }
  _serial->println(data);

  // Log the sent data for debugging
  // Serial.print("[BLE TX] ");
  // Serial.println(data);
  return true;
}