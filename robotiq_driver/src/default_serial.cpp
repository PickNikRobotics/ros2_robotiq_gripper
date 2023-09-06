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

#include <serial/serial.h>

#include <robotiq_driver/default_serial.hpp>

namespace robotiq_driver
{

DefaultSerial::DefaultSerial() : serial_{ std::make_unique<serial::Serial>() }
{
}

void DefaultSerial::open()
{
  serial_->open();
}

bool DefaultSerial::is_open() const
{
  return serial_->isOpen();
}

void DefaultSerial::close()
{
  serial_->close();
}

std::vector<uint8_t> DefaultSerial::read(size_t size)
{
  std::vector<uint8_t> data;
  size_t bytes_read = serial_->read(data, size);
  if (bytes_read != size)
  {
    const auto error_msg = "Requested " + std::to_string(size) + " bytes, but got " + std::to_string(bytes_read);
    THROW(serial::IOException, error_msg.c_str());
  }
  return data;
}

void DefaultSerial::write(const std::vector<uint8_t>& data)
{
  std::size_t num_bytes_written = serial_->write(data);
  serial_->flush();
  if (num_bytes_written != data.size())
  {
    const auto error_msg =
        "Attempted to write " + std::to_string(data.size()) + " bytes, but wrote " + std::to_string(num_bytes_written);
    THROW(serial::IOException, error_msg.c_str());
  }
}

void DefaultSerial::set_port(const std::string& port)
{
  serial_->setPort(port);
}

std::string DefaultSerial::get_port() const
{
  return serial_->getPort();
}

void DefaultSerial::set_timeout(std::chrono::milliseconds timeout)
{
  serial::Timeout simple_timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout.count()));
  serial_->setTimeout(simple_timeout);
}

std::chrono::milliseconds DefaultSerial::get_timeout() const
{
  uint32_t timeout = serial_->getTimeout().read_timeout_constant;
  return std::chrono::milliseconds{ timeout };
}

void DefaultSerial::set_baudrate(uint32_t baudrate)
{
  serial_->setBaudrate(baudrate);
}

uint32_t DefaultSerial::get_baudrate() const
{
  return serial_->getBaudrate();
}

}  // namespace robotiq_driver
