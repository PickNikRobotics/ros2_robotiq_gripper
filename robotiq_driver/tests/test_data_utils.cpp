// Copyright (c) 2022 PickNik, Inc.
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
#include <gtest/gtest.h>
#include <robotiq_driver/data_utils.hpp>

namespace robotiq_driver::test
{
TEST(TestDataUtils, uint8_t_to_hex)
{
  ASSERT_EQ(data_utils::to_hex(std::vector<uint8_t>{ 255, 121, 56, 33, 125, 60 }), "FF 79 38 21 7D 3C");
}

TEST(TestDataUtils, uint16_t_to_hex)
{
  ASSERT_EQ(data_utils::to_hex(std::vector<uint16_t>{ 1169, 58544, 14917, 42884, 36112, 16512, 33207, 62584, 30418 }),
            "0491 E4B0 3A45 A784 8D10 4080 81B7 F478 76D2");
}

TEST(TestDataUtils, to_binary_string)
{
  ASSERT_EQ(data_utils::to_binary_string(155), "10011011");
}

TEST(TestDataUtils, get_msb)
{
  ASSERT_EQ(data_utils::get_msb(0x14A2), 0x14);
}

TEST(TestDataUtils, get_lsb)
{
  ASSERT_EQ(data_utils::get_lsb(0x14A2), 0xA2);
}
}  // namespace robotiq_driver::test
