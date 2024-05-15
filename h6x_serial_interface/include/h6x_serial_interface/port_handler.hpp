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

#pragma once

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/steady_timer.hpp>

#include "h6x_serial_interface/port_handler_base.hpp"

namespace h6x_serial_interface
{
class PortHandler final : public PortHandlerBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(PortHandler)

private:
  const std::string dev_;
  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::serial_port> port_;
  std::chrono::milliseconds timeout_ms_;
  std::unique_ptr<boost::asio::steady_timer> timer_;

public:
  explicit PortHandler(const std::string &);
  bool configure(const int = 115200);
  bool open();
  bool close();

  void set_timeout_ms(const std::chrono::milliseconds timeout_ms);
  std::chrono::milliseconds get_timeout_ms() const;

  ssize_t read(char * const, const size_t) override;
  ssize_t readUntil(std::stringstream &, const char = '\r') const override;
  ssize_t write(const char * const, const size_t) const override;

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace h6x_serial_interface
