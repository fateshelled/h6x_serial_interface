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

#include <gmock/gmock.h>

#include <string>

#include "h6x_serial_interface/port_handler_base.hpp"

using PortHandlerBase = h6x_serial_interface::PortHandlerBase;

ACTION_P(StrCpyToArg0, str)
{
  strcpy(arg0, str);  // NOLINT
}

ACTION_P(StreamCpyToArg0, str) { arg0 << str; }

class MockPortHandler : public PortHandlerBase
{
public:
  MockPortHandler() : PortHandlerBase() {}

  MOCK_METHOD(ssize_t, read, (char * const, const std::size_t), (override));
  MOCK_METHOD(ssize_t, readUntil, (std::stringstream &, const char), (const override));
  MOCK_METHOD(ssize_t, write, (const char * const, const size_t), (const override));
};
