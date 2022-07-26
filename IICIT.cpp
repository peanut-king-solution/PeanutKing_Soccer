/*
 * Copyright (c) 2021 PeanutKing
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
 * @file        IICIT.cpp
 * @summary     Generic I2C Master/Slave Interface
 * @version     1.0
 * @author      Jack Kwok
 * @data        30 July 2021
 */

#include "IICIT.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "avrlibdefs.h"

#define ASSERT_STATUS(X)             \
  do {                               \
    IICIT::status_t status = X;       \
    if (status) {                    \
      m_comm_active = false;         \
      return (IICIT::status_t)status; \
    }                                \
  } while (0)

// Assign global object pointer
IICIT *gIIC = &IICIT::GetInstance();
IICIT::status_t g_status = IICIT::status_t::STATUS_OK;

static IICIT *g_callback_object = nullptr; // Global pointer to class object

static volatile bool g_rx_complete = false;

IICIT::IICIT()
    : m_speed(IICIT::Speed::INIT), m_comm_active(false), m_timeout(100),
    m_state{State::IDLE}, m_status{STATUS_OK}, m_reset{true} {
  // Activate internal pullups for TWI
  // Required for 400KHz operation
  *portOutputRegister(digitalPinToPort(SDA)) |= digitalPinToBitMask(SDA);
  *portOutputRegister(digitalPinToPort(SCL)) |= digitalPinToBitMask(SCL);

  // set i2c bit rate
  SetSpeed(Speed::INIT);

  // enable TWI (two-wire interface)
  sbi(TWCR, TWEN);
  // enable TWI interrupt and slave address ACK
  sbi(TWCR, TWIE);
  sbi(TWCR, TWEA);

  g_callback_object = this;
}

bool IICIT::IsCommActive(void) {
  return m_comm_active;
};

const IICIT::Handle IICIT::RegisterDevice(const uint8_t device_address, const uint8_t address_size, const IICIT::Speed speed) {
  if (device_address < MAX_ADDRESS) {
    if ((address_size > 0) && (address_size < 5)) {
      return {device_address, address_size, speed};
    }
  }
  return {}; // Error
}

IICIT::status_t IICIT::Read(const Handle &handle, uint8_t data[], const uint32_t bytes, void (*callback)(const uint8_t error)) {
  const IICIT::Mode mode = Mode::READ;
  const uint32_t address = DISCARD_ADDRESS;
  const uint8_t delay_ms = 0;

  // Calculate buffer remaining capacity
  uint8_t capacity = (IICIT::size_t::SIZE_BUFFER - handle.address_size);

  // Determine if transfer should occur
  status_t status = PrepareForTransfer(mode, handle, address, bytes, capacity);

  if (status == STATUS_OK) {
    bool block = (callback == nullptr);
    g_rx_complete = false;

    // Create packet to read bytes
    IICIT::Packet packet{IICIT::Mode::READ, handle.device_address, data, bytes, 0, (block ? ReadCallback : callback)};

    ASSERT_STATUS(MasterQueueNonBlocking(packet));  // Queue read request

    if (block) {
      status_t wait_status = WaitForComplete();
      if (wait_status) {
        m_comm_active = false;
        return wait_status;
      }
    }
    m_comm_active = false;    // Relinquish access
    return STATUS_OK;
  }
  return status;              // BUSY or ERROR
}

IICIT::status_t IICIT::Write(const Handle &handle, const uint8_t data[], const uint32_t bytes, const uint8_t delay_ms) {
  const IICIT::Mode mode = Mode::WRITE;
  const uint32_t address = DISCARD_ADDRESS;

  // Calculate buffer remaining capacity
  uint8_t capacity = (IICIT::size_t::SIZE_BUFFER - handle.address_size);

  // Determine if transfer should occur
  status_t status = PrepareForTransfer(mode, handle, address, bytes, capacity);

  if (status == STATUS_OK) {
    uint32_t bytes_transmitted = 0;
    while (bytes_transmitted < bytes) {
      // Create device destination register definition
      IICIT::Register device_register{address + bytes_transmitted, handle.address_size};
      // Check if remaining byte count greater than buffer capacity
      uint32_t bytes_remaining = (bytes - bytes_transmitted);
      uint8_t bytes_to_transmit = (bytes_remaining > capacity) ? capacity : bytes_remaining;

      // Create data packet
      IICIT::Packet packet{IICIT::Mode::WRITE, handle.device_address, &data[bytes_transmitted], bytes_to_transmit, delay_ms, nullptr};
      // Queue write request
      ASSERT_STATUS(MasterQueueNonBlocking(packet, (address != DISCARD_ADDRESS) ? &device_register : nullptr));
      // Increment byte counter
      bytes_transmitted += bytes_to_transmit;
    }
    WaitForCompleteTx();
    m_comm_active = false;    // Relinquish access
    return STATUS_OK;
  }
  return status;              // BUSY or ERROR
}


IICIT::status_t IICIT::PrepareForTransfer(const Mode mode, const IICIT::Handle &handle, const uint32_t address, const uint32_t bytes, const uint8_t capacity) {
  if (!IsCommActive()) {              // Check if active
    if (handle.address_size > 0) {    // Check if handle valid
      if (!(SREG & 0x80)) {           // Check if interrupts are disabled
        uint8_t packet_count = 1;
        if (mode == Mode::WRITE) {
          if (bytes > capacity) {
            return STATUS_BUSY;
          }
        }
        else {
          if (address != DISCARD_ADDRESS) {
            packet_count++;
          }
        }
        // Check that required packets will not exceed queue availability
        if (packet_count > GetQueueVacancy()) {
          return STATUS_BUSY;
        }
      }
      m_comm_active = true;           // Flag as active
      if (m_speed != handle.speed)  { // Check if speed different
        SetSpeed((handle.speed == Speed::SLOW) ? IICIT::Speed::SLOW : IICIT::Speed::FAST);
        m_speed = handle.speed;
      }
      return STATUS_OK;
    }
    return STATUS_ERROR_MEMORY_ALLOCATION;
  }
  return STATUS_BUSY;
}

void IICIT::ReadCallback(const IICIT::status_t status) {
  g_status = (IICIT::status_t)status;
  g_rx_complete = true;
}

IICIT::status_t IICIT::WaitForComplete(void) {
  unsigned long end = millis() + (unsigned long)m_timeout;
  while (!g_rx_complete) {      // Wait while transmission incomplete
    if (m_timeout > 0) {        // Check if timeout is set
      if (millis() > end) {     // Check current time
        return STATUS_TIMEOUT;  // Timeout
      }
    }
  }
  return g_status;
}

IICIT::status_t IICIT::WaitForCompleteTx(void) {
  unsigned long end = millis() + (unsigned long)m_timeout;
  while (m_state != State::IDLE) {
    if (m_timeout > 0) {    // Check if timeout is set
      if (millis() > end) { // Check current time
        // AVR i2c state machine has potential to lockup
        // Reset i2c bus
        TWCR &= ~(_BV(TWEN));
        TWCR |= (_BV(TWEN));
        m_state = State::IDLE;
        return STATUS_TIMEOUT;
      }
    }
  }
  return m_status;
}



// 100k -> 72, // 400k -> 12
void IICIT::SetSpeed(const IICIT::Speed speed) {
  uint8_t bitrate_div;
  // set i2c bitrate
  // SCL freq = F_CPU/(16+2*TWBR))
#ifdef TWPS0
  // for processors with additional bitrate division (mega128)
  // SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
  // set TWPS to zero
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
#endif
  uint16_t value = ((speed == Speed::SLOW) ? 100 : 400);

  // calculate bitrate division
  bitrate_div = ((F_CPU / 1000l) / value);

  if (bitrate_div >= 16) {
    bitrate_div = (bitrate_div - 16) / 2;
  }
  outb(TWBR, bitrate_div);
}

void IICIT::SetTimeoutMS(const uint16_t timeout) {
  m_timeout = timeout;
}

uint8_t IICIT::GetQueueVacancy(void) {
  return m_queue.Vacancy();
}

// A locked state occurs if the queue is full and interrupts are disabled.
// This condition is prevented by the upper nI2C class.
IICIT::status_t IICIT::MasterQueueNonBlocking(IICIT::Packet &packet, const IICIT::Register *const register_address) {
  if (m_queue.IsFull()) {
    uint32_t timeout = 100000;
    while (m_queue.IsFull() && --timeout);    // Wait for queued transmit to complete
    if (timeout == 0) {
      Stop();
      ProcessQueue();
      return STATUS_BUSY;
    }
  }

  // Only writes have data to copy at this point
  if (packet.mode == Mode::WRITE) {
    uint8_t offset = 0;

    if (register_address != nullptr) {
      offset = register_address->address_size;
    }
    if ((packet.length + offset) > SIZE_BUFFER) {    // Boundary check
      return STATUS_ERROR_BUFFER_OVERFLOW;
    }

    uint8_t sreg = SREG;  // Save register
    cli();                // Memory operators are not reentrant - halt interrupts
    // Allocate memory for message copy
    uint8_t *message = new uint8_t[packet.length + offset];
    SREG = sreg; // Restore register

    // Verify allocation was successful
    if (message != nullptr) {
      // Copy data into message
      memcpy_fast(&message[offset], packet.data, packet.length);

      // Copy register (if defined) into message
      for (uint8_t index = 0; index < offset; index++) {
        message[index] = register_address->address.byte[offset - index - 1];
        packet.length++; // Add register byte
      }
      packet.data = message; // Re-assign data pointer
    }
    else {
      return STATUS_ERROR_MEMORY_ALLOCATION;
    }
  }

  // Interrupts are disabled to prevent reentry into the following section
  uint8_t sreg = SREG;                // Save register
  cli();
  bool is_empty = m_queue.IsEmpty();  // Save status
  m_queue.Push(packet);               // Copy packet to queue

  if (is_empty) {     // Kick-start transmit if single packet in queue
    ProcessQueue();
  }

  SREG = sreg;        // Restore register
  return m_status;
}

// Entry into this method must only be on completion of packet with a non-empty
// queue or when kick-starting transmit with previously empty queue.
void IICIT::ProcessQueue(void) {
  // Retrieve next packet
  Packet *packet = m_queue.Front();
  uint8_t RW;

  if (packet->mode == Mode::WRITE) {
    m_state = State::MASTER_TX;
    RW = TW_WRITE;
  }
  else {
    m_state = State::MASTER_RX;
    RW = TW_READ;
  }

  m_buffer = packet->data; // Assign pointer
  m_device_address = ((packet->device_address << 1) | RW);
  m_buffer_length = packet->length;
  m_buffer_index = 0;
  m_status = STATUS_OK;
  SendStart(); // Begin transmit
}

// Entry into this method is only via ISR
void IICIT::ProcessCallback(void) {
  Packet *packet = m_queue.Front();   // Retrieve current packet
  if (packet->mode == Mode::WRITE) {  // Only writes need to deallocate memory
    delete[] packet->data;            // Deallocate memory
  }

  if (packet->callback != nullptr) {
    packet->callback(m_status);       // Invoke user callback
  }

  // Optional user requested delay on success
  if ((packet->delay > 0) && !m_status) {
    if (SREG & 0x80) {    // Check if interrupts are enabled
      delay(packet->delay);
    }
    else {            // Calculate cycles needed for delay
      uint32_t count = ((F_CPU / 4000) * packet->delay);
      while (--count) {
        __asm__ __volatile__("nop");
      }
    }
  }
  m_queue.Pop();      // Remove packet

  // Are there more packets to process?
  if (!m_queue.IsEmpty()) {
    ProcessQueue();
  }
}

inline void IICIT::SendStart(void) {
  // send start condition
  outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT) | _BV(TWSTA));
}

inline void IICIT::Stop(void) {
  // transmit stop condition
  // leave with TWEA on for slave receiving
  outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT) | _BV(TWEA) | _BV(TWSTO));

  // Stop should execute very quickly
  uint8_t timeout = 100;

  // Wait for stop condition to be executed on bus
  while ((TWCR & _BV(TWSTO)) && --timeout);

  // Check if AVR i2c bus is locked
  if (timeout == 0) {
    // Reset i2c bus
    TWCR &= ~(_BV(TWEN));
    TWCR |= (_BV(TWEN));
  }

  m_state = State::IDLE;
}

inline void IICIT::SendByte(const uint8_t data) {
  outb(TWDR, data); // save data to the TWDR
}

inline void IICIT::SendACK(void) {
  outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT) | _BV(TWEA));
}

inline void IICIT::SendNACK(void) {
  outb(TWCR, (inb(TWCR) & MASK_TWCR_CMD) | _BV(TWINT));
}

inline uint8_t IICIT::GetReceivedByte(void) {
  // retrieve received data byte from i2c TWDR
  return (inb(TWDR));
}

inline void IICIT::InterruptHandlerWrapper(void) {
  g_callback_object->InterruptHandler();
}

//! I2C (TWI) interrupt service routine
ISR(TWI_vect) {
  IICIT::InterruptHandlerWrapper();
}

void IICIT::InterruptHandler(void) {
  switch (TW_STATUS) {
  // Master General
  case TW_START:                // 0x08: Sent start condition
  case TW_REP_START:            // 0x10: Sent repeated start condition
    SendByte(m_device_address); // send device address
    SendACK();                  // begin send
    break;

  // Master Transmitter & Receiver status codes
  case TW_MT_SLA_ACK:     // 0x18: Slave address acknowledged
  case TW_MT_DATA_ACK:    // 0x28: Data acknowledged
    if (m_buffer_index < m_buffer_length) {
      SendByte(m_buffer[m_buffer_index++]); // send data
      SendACK();                            // begin send
    }
    else {
      Stop();     // transmit stop condition, enable SLA ACK
      // i2c transmit is complete
      ProcessCallback(); // Remove packet from queue
    }
    break;

  case TW_MT_ARB_LOST:    // 0x38: Bus arbitration lost
  //case TW_MR_ARB_LOST:  // 0x38: Bus arbitration lost
    SendNACK();
    m_state = State::IDLE;
    //m_status = STATUS_ERROR_LOST_ARBITRATION; // Not a true error
    // Start condition will automatically transmit when bus becomes free
    // No need to process further
    break;

  case TW_MT_SLA_NACK:    // 0x20: Slave address not acknowledged
  case TW_MR_SLA_NACK:    // 0x48: Slave address not acknowledged
  case TW_MT_DATA_NACK:   // 0x30: Data not acknowledged
    m_status = ((TW_STATUS == TW_MT_DATA_NACK) ? STATUS_ERROR_DEVICE_REJECTED_DATA : STATUS_ERROR_NO_DEVICE_RESPONSE);
    Stop();               // transmit stop condition, enable SLA ACK
    ProcessCallback();    // Remove packet from queue
    break;

  case TW_MR_DATA_NACK:   // 0x58: Data received, NACK reply issued
    // prevent buffer overflow
    if (m_buffer_index < m_buffer_length) {
      // store final received data byte
      m_buffer[m_buffer_index++] = GetReceivedByte();
    }
    Stop();       // transmit stop condition, enable SLA ACK
    // i2c receive is complete
    ProcessCallback();  // Remove packet from queue
    break;

  case TW_MR_DATA_ACK:  // 0x50: Data acknowledged
    // prevent buffer overflow
    if (m_buffer_index < m_buffer_length) {
      // store received data byte
      m_buffer[m_buffer_index++] = GetReceivedByte();
    }
    // fall-through to see if more bytes will be received
    [[gnu::fallthrough]]; // Fall-through
  case TW_MR_SLA_ACK:     // 0x40: Slave address acknowledged
    if (m_buffer_index < (m_buffer_length - 1)) {
      // data byte will be received, reply with ACK (more bytes in transfer)
      SendACK();
    }
    else {
      // data byte will be received, reply with NACK (final byte in transfer)
      SendNACK();
    }

    break;
  // Misc
  default:
  case TW_NO_INFO:        // 0xF8: No relevant state information
    // do nothing
    break;

  case TW_BUS_ERROR:      // 0x00: Bus error due to illegal start or stop condition
    m_status = STATUS_ERROR_ILLEGAL_START_STOP;
    Stop();               // reset internal hardware and release bus
    ProcessCallback();    // Remove packet from queue
    break;
  }
}
