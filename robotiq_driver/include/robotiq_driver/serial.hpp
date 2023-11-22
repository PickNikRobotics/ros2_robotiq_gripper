// Copyright (c) 2023 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace robotiq_driver
{
/**
 * The driver talks to the hardware through an implementation of the
 * serial interface.
 * We can mock the serial connection to check that a high level command
 * (e.g activate) is converted into the correct sequence of bytes
 * including CRC.
 */
class Serial
{
public:
  virtual ~Serial() = default;

  /**
   * Opens the serial port as long as the port is set and the port isn't
   * already open.
   */
  virtual void open() = 0;

  /**
   * Gets the open status of the serial port.
   * @return Returns true if the port is open, false otherwise.
   */
  [[nodiscard]] virtual bool is_open() const = 0;

  /** Closes the serial port. */
  virtual void close() = 0;

  /**
   * Read a given amount of bytes from the serial port.
   * @param size The number of bytes to read.
   * @return The sequence of bytes.
   * @throw serial::PortNotOpenedException
   * @throw serial::SerialException
   */
  [[nodiscard]] virtual std::vector<uint8_t> read(size_t size) = 0;

  /**
   * Write a sequence of bytes to the serial port.
   * @param data A vector containing data to be written to the serial port.
   * @throw serial::PortNotOpenedException
   * @throw serial::SerialException
   * @throw serial::IOException
   */
  virtual void write(const std::vector<uint8_t>& data) = 0;

  /**
   * Sets the serial port identifier.
   * @param port A const std::string reference containing the address of the
   * serial port, which would be something like "COM1" on Windows and
   * "/dev/ttyS0" on Linux.
   * @throw std::invalid_argument
   */
  virtual void set_port(const std::string& port) = 0;

  /**
   * Gets the serial port identifier.
   * @see SerialInterface::setPort
   * @throw std::invalid_argument
   */
  [[nodiscard]] virtual std::string get_port() const = 0;

  /**
   * Set read timeout in milliseconds.
   * @param timeout Read timeout in milliseconds.
   */
  virtual void set_timeout(const std::chrono::milliseconds timeout) = 0;

  /**
   * Get read timeout in milliseconds.
   * @return Read timeout in milliseconds.
   */
  [[nodiscard]] virtual std::chrono::milliseconds get_timeout() const = 0;

  /**
   * Sets the baudrate for the serial port.
   * @param baudrate An integer that sets the baud rate for the serial port.
   */
  virtual void set_baudrate(uint32_t baudrate) = 0;

  /**
   * Gets the baudrate for the serial port.
   * @return An integer that sets the baud rate for the serial port.
   */
  [[nodiscard]] virtual uint32_t get_baudrate() const = 0;
};
}  // namespace robotiq_driver
