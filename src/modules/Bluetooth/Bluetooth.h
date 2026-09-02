#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#define BAUD_RATE 9600

#include <Arduino.h>

#include "txDataPacker.h"
#include "rxDataParser.h"

enum BLEStatus : uint8_t
{
  BLE_DISCONNECTED = 0,
  BLE_CONNECTED = 1,
};

enum RemoteMode : uint8_t
{
  PILA_LEGACY = 0,
  PILA_CONFIG = 1,
  DASHBOARD = 2,
};

typedef void (*ButtonCallback)(bool pressed);

struct JoystickState { int angle; int strength; };
class Bluetooth
{
private:
  friend class PeanutKingSoccerV4;
  Bluetooth();

// Basic info
  RemoteMode _mode;   // Current mode of operation
  BLEStatus _status;  // Current connection status
  HardwareSerial *_serial;  // Serial port for communication with the Bluetooth module

// Serial handling
  txDataPacker _txPacker;  // Packer for outgoing data
  rxDataParser _rxParser;  // Parser for incoming data
  String _rxBuffer;  // Buffer for incoming data


// Configuration
  bool _isConfigured;  // Whether the module has been configured

// Processing
  void processFrame(const String& frame);

public:
  bool init(
    HardwareSerial *port = &Serial1,
    RemoteMode mode = PILA_LEGACY
  );

// ============================================================================
//                          AT Commands
// ============================================================================

  String sendATCommand(const char *cmd, uint32_t timeout = 1500);
  bool setNotifications(bool enable);
  bool rename(const char *name);
  bool reset(void);
  bool ping(void);

// ============================================================================
//                              Mode
// ============================================================================

  void setMode(RemoteMode mode);
  RemoteMode getMode(void) const;

// ============================================================================
//                            Connection
// ============================================================================

  bool isConnected(void) const;
  void reconnect(void);

// ============================================================================
//                            Configuration
// ============================================================================

  bool isConfigured(void);
  void setConfig(const String& config);

// ============================================================================
//                               Process
// ============================================================================

  void processSerial(void);
  bool hasCommand(void) const;
  RxCommand getCommand(void);

// ============================================================================
//                            Components getters
// ============================================================================

  int getSliderValue(const String& name) const;
  bool getToggleState(const String& name) const;
  void onButton(const String& name, ButtonCallback callback);
  String getTextFieldValue(const String& name) const;
  JoystickState getJoystick(const String& name) const;

// ============================================================================
//                            Output setters
// ============================================================================

  void setOutput(const String& name, int value);
  void setOutput(const String& name, float value);
  void setOutput(const String& name, bool value);
};

#endif // BLUETOOTH_H