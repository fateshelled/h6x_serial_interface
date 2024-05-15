// Copyright 2022 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "h6x_serial_interface/port_handler.hpp"

namespace h6x_serial_interface
{
using namespace std::chrono_literals;

PortHandler::PortHandler(const std::string & dev)
: dev_(dev), timeout_ms_(30ms), is_timer_canceled_(false), io_() {}


void PortHandler::set_timeout_ms(const std::chrono::milliseconds timeout_ms)
{
  this->timeout_ms_ = timeout_ms;
}

std::chrono::milliseconds PortHandler::get_timeout_ms() const
{
  return this->timeout_ms_;
}

bool PortHandler::configure(const int baudrate)
{
  using namespace boost::asio;  // NOLINT
  try {
    this->port_ = std::make_unique<serial_port>(this->io_);
    this->port_->open(this->dev_);
    this->port_->set_option(serial_port_base::baud_rate(baudrate));
    this->port_->set_option(serial_port_base::character_size(8));
    this->port_->set_option(serial_port_base::parity(serial_port_base::parity::none));
    this->port_->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    this->port_->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    this->port_->close();

    this->timer_ = std::make_unique<boost::asio::steady_timer>(this->io_);

    this->io_.run();

  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

bool PortHandler::open()
{
  try {
    if (!this->port_->is_open()) {
      this->port_->open(this->dev_);

      const size_t tmp_len = 100;
      char tmp_buf[100];
      for (size_t i = 0; i < 10; ++i)
      {
        this->read(tmp_buf, tmp_len);
      }
    }
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

bool PortHandler::close()
{
  try {
    if (this->port_->is_open()) {
      this->port_->cancel();
      this->port_->close();
    }
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

ssize_t PortHandler::read(char * const buf, const size_t size)
{
  if (!this->port_->is_open()) {
    RCLCPP_ERROR(this->getLogger(), "%s: not opened", this->dev_.c_str());
    return -1;
  }

  boost::system::error_code timer_error;
  boost::system::error_code read_error;
  ssize_t len = 0;

  this->is_timer_canceled_ = false;
  boost::asio::async_read(
    *this->port_,
    boost::asio::buffer(buf, size),
    boost::asio::transfer_at_least(1),
    [this, &read_error, &len](const boost::system::error_code& error, std::size_t bytes_transferred)
      {
        // if (error == boost::asio::error::operation_aborted)
        if (error)
        {
          read_error = error;
        }
        else
        {
          // SUCCESS async_read
          this->timer_->cancel();
          this->is_timer_canceled_ = true;

          len = bytes_transferred;
        }
      }
  );

  this->timer_->expires_from_now(this->timeout_ms_);
  this->timer_->async_wait(
    [this, &timer_error](const boost::system::error_code& error)
    {
      if (!error && !this->is_timer_canceled_) {
          this->port_->cancel(); // 通信処理をキャンセルする。受信ハンドラがエラーになる
          timer_error = error;
      }

    }
  );

  this->io_.reset();
  this->io_.run();

  if (timer_error)
  {
    // timeout
    RCLCPP_ERROR(this->getLogger(), "TIMEOUT ERROR %s", timer_error.message().c_str());
    return -1;
  }
  if (read_error)
  {
    // read error
    RCLCPP_ERROR(this->getLogger(), "READ ERROR %s", read_error.message().c_str());
    return -1;
  }

  // RCLCPP_INFO(this->getLogger(), "Read %d bytes", len);
  return len;
}

ssize_t PortHandler::readUntil(std::stringstream & buf, const char delimiter) const
{
  if (!this->port_->is_open()) {
    RCLCPP_ERROR(this->getLogger(), "%s: not opened", this->dev_.c_str());
    return -1;
  }

  try {
    std::string tmp;
    const auto ret =
      boost::asio::read_until(*this->port_, boost::asio::dynamic_buffer(tmp), delimiter);
    buf << tmp;
    return ret;
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
  }
  return -1;
}

ssize_t PortHandler::write(char const * const buf, const size_t size) const
{
  if (!this->port_->is_open()) {
    RCLCPP_ERROR(this->getLogger(), "%s: not opened", this->dev_.c_str());
    return -1;
  }

  try {
    return this->port_->write_some(boost::asio::buffer(buf, size));
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
  }
  return -1;
}

const rclcpp::Logger PortHandler::getLogger() noexcept { return rclcpp::get_logger("PortHandler"); }
}  // namespace h6x_serial_interface
