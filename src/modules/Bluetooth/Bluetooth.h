#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>

#include "txDataPacker.h"
#include "rxDataParser.h"

enum class BLEStatus : uint8_t
{
  DISCONNECTED = 0,
  CONNECTED = 1,
};

enum class RemoteMode : uint8_t
{
  PILA = 0,
  DASHBOARD = 1,
};

class Bluetooth
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  // Remote mode (PILA or DASHBOARD)
  RemoteMode _mode;
  // BLE status (CONNECTED or DISCONNECTED)
  BLEStatus _status;
  // Serial port for BLE module (default: Serial1)
  HardwareSerial *_serial;

  // Instances of data packer and parser for BLE communication
  txDataPacker _txPacker; // Instance of txDataPacker for packing data to send
  rxDataParser _rxParser; // Instance of rxDataParser for parsing received data

  // Constructor (accessible only to the friend PeanutKingSoccerV4)
  Bluetooth();

public:

// ============================================================================
//                              Configuration 
// ============================================================================

  /**
   * Set the hardware serial port for the BLE module
   * `port` - Pointer to the serial port (e.g., &Serial1, &Serial2, &Serial3)
   *          Default: &Serial1
   * Must call init() after setting the serial port.
   * 
   * `Returns` - true if the serial port is set successfully, false otherwise
   */
  bool setSerial(HardwareSerial *port);

  /**
   * Initialize the BLE module (begin serial at 9600 baud)
   * Call this once in setup() after Serial.begin()
   * `baudRate` - Serial baud rate for BLE module (default: 9600)
   * `mode` - Remote mode (PILA or DASHBOARD, default: PILA)
   * 
   * `Returns` - true if initialization is successful, false otherwise
   */
  bool init(uint32_t baudRate = 9600, RemoteMode mode = RemoteMode::PILA);

// =============================================================================
//                              BLE Operations
// =============================================================================

  /**
   * Send a raw AT command and get the response
   * `cmd`    - AT command string (e.g., "AT", "AT+NAME?")
   * `timeout`- Wait time in ms (default: 1500)
   *
   * `Returns` - Response string (empty if timeout)
   */
  String sendATCommand(const char *cmd, uint32_t timeout = 1500);

  /**
   * Get basic information from the BLE module
   * info: NAME, UUID, CHAR, BAUD, ROLE, VERSION, ADDR
   *
   * `Returns` - true if successful, false otherwise
   */
  bool getBasicInfo(void);

  /**
   * Enable or disable BLE notifications for connection status changes
   * `enable` - true to enable, false to disable
   *
   * `Returns` - true if successful, false otherwise
   */
  bool setNotifications(bool enable);

  /**
   * Rename the BLE module
   * `name` - New name (max 20 characters)
   *
   * `Returns` - true if successful, false otherwise
   */
  bool rename(const char *name);

  /**
   * Reset the BLE module
   *
   * `Returns` - true if successful, false otherwise
   */
  bool reset(void);

  /**
   * Ping the BLE module to check if it's responsive
   *
   * `Returns` - true if successful, false otherwise
   */
  bool ping(void);

// ============================================================================
//                              Mode
// ============================================================================

  /**
   * Set the remote mode (PILA or DASHBOARD)
   * `mode` - The desired remote mode
   */
  void setMode(RemoteMode mode);
  /**
   * Get the current remote mode
   *
   * `Returns` - Current remote mode (PILA or DASHBOARD)
   */
  RemoteMode getMode(void);

// ============================================================================
//                              Connection
// ============================================================================

  /**
   * Check if the BLE module is currently connected
   *
   * `Returns` - true if connected, false otherwise
   */
  bool isConnected(void);
  /**
   * Check for async connection status notifications from the BLE module
   * When notifications are enabled, HM-10 sends:
   *   "OK+CONN" on connection established
   *   "OK+LOST" on connection lost
   *
   * `Returns` - true if a notification was received and handled, false otherwise
   */
  bool checkConnection(void);
  /**
   * Handle connection status changes (connected/disconnected)
   * This function is called when a connection status notification is received
   */
  void handleConnection(void);
  /**
   * Attempt to reconnect to the BLE module if disconnected
   * This function is called when a disconnection is detected
   */
  void reconnect(void);

  /**
   * Send data to the connected BLE device
   * `data` - The data to send
   *
   * `Returns` - true if successful, false otherwise
   */
  bool sendData(const String& data);
};

#endif // BLUETOOTH_H