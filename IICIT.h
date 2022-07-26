/*
 * Copyright (c) 2018 nitacku
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * @file        IICIT.h
 * @summary     Generic I2C Master/Slave Interface
 * @version     2.1
 * @author      nitacku
 * @data        15 July 2018
 */

#ifndef _nI2C_H_
#define _nI2C_H_

#include <Arduino.h>
#include <inttypes.h>
#include "queue.h"

// Using singleton design pattern
class IICIT {
public:
  enum size_t : uint8_t {
    SIZE_BUFFER = 36,
    SIZE_QUEUE = 4,
  };

  enum class Speed : uint8_t {
    SLOW,
    FAST,
    INIT = SLOW,
  };

  enum status_t : uint8_t {
    STATUS_OK = 0,
    STATUS_BUSY,
    STATUS_TIMEOUT,
    STATUS_ERROR_BUFFER_OVERFLOW,
    STATUS_ERROR_MEMORY_ALLOCATION,
    STATUS_ERROR_STATE_MISMATCH,
    STATUS_ERROR_NO_DEVICE_RESPONSE,
    STATUS_ERROR_DEVICE_REJECTED_DATA,
    STATUS_ERROR_ILLEGAL_START_STOP,
    STATUS_ERROR_LOST_ARBITRATION,
  };

  enum class Mode : uint8_t {
    WRITE,
    READ,
  };

  union Address {
    Address(const uint32_t _value)
        : value{_value} {
      // empty
    }
    uint32_t value;
    uint8_t byte[sizeof(uint32_t)];
  };

  struct Register {
    Register(const Address _address,
             const uint8_t _address_size)
        : address{_address}, address_size{_address_size}  {
      // empty
    }
    Address address;
    uint8_t address_size;
  };

  struct Packet {
    Packet() : length{0}, delay{0}, callback{nullptr} {
      // empty
    }
    Packet(const Mode _mode,
           const uint8_t _device_address,
           uint8_t *_data,
           const uint32_t _length,
           const uint8_t _delay,
           void (*_callback)(const status_t status))
        : mode{_mode}, device_address{_device_address}, data{_data}, length{_length}, delay{_delay}, callback{_callback}
    {
      // empty
    }
    Mode mode;                               // Read or Write
    uint8_t device_address;                  // Device I2C address
    uint8_t *data;                           // Pointer to read/write buffer
    uint32_t length;                         // Length of data to read/write
    uint8_t delay;                           // Optional delay on success
    void (*callback)(const status_t status); // Optional callback on success
  };

private:
  enum mask_t : uint8_t {
    MASK_TWCR_CMD = 0x0F,
  };

  enum class State : uint8_t {  // types
    IDLE,
    MASTER_TX,
    MASTER_RX,
    SLAVE_TX,
    SLAVE_RX,
  };

public:
  struct Handle  {
    Handle(void) : device_address{0}, address_size{0}, speed{Speed::INIT} {
      // empty
    }
    Handle(const uint8_t _device_address,
           const uint8_t _address_size,
           const Speed _speed)
        : device_address(_device_address), address_size(_address_size), speed(_speed)
    {
      // empty
    }
    uint8_t device_address;
    uint8_t address_size;
    Speed speed;
  };

private:
  enum limit_t : uint8_t {
    MAX_ADDRESS = 0x80,
  };

  const uint32_t DISCARD_ADDRESS = -1;

  // Reduce clutter and allow type-casting
  typedef void (*Callback)(const IICIT::status_t status);

  Speed m_speed;
  bool m_comm_active;

public:
  IICIT(IICIT const &) = delete;           // Forbid constructor
  void operator=(IICIT const &) = delete; // Forbid constructor

  // Return reference to IICIT for accessing methods
  static IICIT &GetInstance()  {
    static IICIT m_instance; // Guaranteed to be destroyed
    return m_instance;      // Instantiated on first use
  }

  // Return true when i2c peripheral is in use
  bool IsCommActive(void);
  // Set the timeout interval to wait for operation to complete
  void SetTimeoutMS(const uint16_t timeout);
  // Register new device with <device_address> and internal memory <address_size> at <speed>
  const Handle RegisterDevice(const uint8_t device_address, const uint8_t address_size, const Speed speed);
  // Write <bytes> amount of <data> to device with <delay_ms> pause between transmits
  IICIT::status_t Write(const Handle &handle, const uint8_t data[], const uint32_t bytes, const uint8_t delay_ms = 0);
  // Read <bytes> amount from device into <data>
  IICIT::status_t Read(const Handle &handle, uint8_t data[], const uint32_t bytes, void (*callback)(const uint8_t status) = nullptr);

private:
  IICIT(void); // Default constructor
  IICIT::status_t PrepareForTransfer(const Mode mode, const Handle &handle, const uint32_t address, const uint32_t bytes, const uint8_t capacity);
  static void ReadCallback(const IICIT::status_t status);
  IICIT::status_t WaitForComplete(void);
  IICIT::status_t WaitForCompleteTx(void);

  // I2C state and address variables
  volatile State m_state;            // Current I2C state
  volatile status_t m_status;        // Status
  volatile bool m_reset;             // Flag indicating reset needed on m_buffer_length
  uint16_t m_timeout;                // Time in ms to wait for blocking operations
  uint8_t m_device_address;          // 7 bit slave device address
  volatile uint32_t m_buffer_index;  // Buffer index
  volatile uint32_t m_buffer_length; // Buffer length
  uint8_t *m_buffer;                 // Data buffer

  CQueue<Packet> m_queue{SIZE_QUEUE}; // Queue of packets

public:
  // Get the number of vacant elements in the queue
  uint8_t GetQueueVacancy(void);
  // Set the I2C transaction bitrate
  void SetSpeed(const Speed speed);

  inline status_t getStatus(void) __attribute__((always_inline)) {
    return m_status;
  }
  // high-level I2C transaction commands
  // Add packet to queue - Must be used in non-blocking mode
  status_t MasterQueueNonBlocking(Packet &packet, const Register *const register_address = nullptr);

private:
  // Process any entries in queue
  void ProcessQueue(void);
  // Called after transmit complete
  void ProcessCallback(void);

  // Low-level I2C transaction commands
  // Send an I2C start condition in Master mode
  inline void SendStart(void) __attribute__((always_inline));
  // Send an I2C stop condition in Master mode
  void Stop(void);
  // Send an (address|R/W) combination or a data byte over I2C
  inline void SendByte(const uint8_t data) __attribute__((always_inline));
  // Send ACK
  inline void SendACK(void) __attribute__((always_inline));
  // Send NACK
  inline void SendNACK(void) __attribute__((always_inline));
  // Read byte
  inline uint8_t GetReceivedByte(void) __attribute__((always_inline));
  // Called by hardware I2C interrupt
  inline void InterruptHandler(void) __attribute__((always_inline));

public:
  static inline void InterruptHandlerWrapper(void) __attribute__((always_inline));

};

extern IICIT *gIIC;
extern IICIT::status_t g_status;
#endif
