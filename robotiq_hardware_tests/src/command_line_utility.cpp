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

#include <iostream>

#include "command_line_utility.hpp"

void CommandLineUtility::registerHandler(const std::string& parameter, ParameterHandler handler, bool isMandatory)
{
  handlers[parameter] = handler;
  if (isMandatory)
  {
    mandatoryParams.insert(parameter);
  }
}

bool CommandLineUtility::parse(int argc, char* argv[])
{
  for (int i = 1; i < argc; i++)
  {
    auto it = handlers.find(argv[i]);
    if (it != handlers.end())
    {
      receivedParams.insert(it->first);

      if (std::holds_alternative<LambdaWithValue>(it->second))
      {
        auto& handler = std::get<LambdaWithValue>(it->second);
        i++;
        if (i < argc)
        {
          handler(argv[i]);
        }
        else
        {
          std::cerr << it->first << " requires a value.\n";
        }
      }
      else if (std::holds_alternative<LambdaWithoutValue>(it->second))
      {
        auto& handler = std::get<LambdaWithoutValue>(it->second);
        handler();
      }
    }
    else
    {
      std::cerr << "Unknown argument: " << argv[i] << "\n";
      return false;
    }
  }

  for (const auto& param : mandatoryParams)
  {
    if (receivedParams.find(param) == receivedParams.end())
    {
      std::cerr << "Missing mandatory argument: " << param << "\n";
      return false;
    }
  }

  return true;
}
