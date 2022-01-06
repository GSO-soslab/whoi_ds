/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by zac on 12/7/17.
//

#include "ds_sensors/sbe49.h"

#include <list>
#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class Sbe49Test : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

// This test case should capture known-bad strings that should fail parsing.
TEST_F(Sbe49Test, failingParses)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto lines = std::list<std::string>{
    "4.>024,  3.30013,  863.29M-s,  34.4399, 1482.588\r\n",  // corruption
    "4.5078,  3.29269,  876.339,  34.4446\r\n",              // Strings can have 3 or 5 fields, never 4
    "4.5078,  3.29269\r\n",                                  // Too few fields
  };

  for (const auto& line : lines)
  {
    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    byte_msg.ds_header.io_time = ros::Time::now();
    byte_msg.data = std::vector<unsigned char>(std::begin(line), std::end(line));

    // Attempt to parse
    auto result = ds_sensors::Sbe49::parse_bytes(byte_msg);
    EXPECT_FALSE(result.first);
  }
}

// This test case should capture valid strings that we don't have parsers for
TEST_F(Sbe49Test, unimplementedParsers)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto pairs = std::list<std::pair<std::string, ds_sensors::Sbe49::OutputFormat>>{
    { "0A53711BC7220C14C17D82\r\n", ds_sensors::Sbe49::OutputFormat::RawHex },
    { "3385C40F42FE0186DE\r\n", ds_sensors::Sbe49::OutputFormat::EngineeringHex },
    { "676721, 7111.133, 791745, 2.4514\r\n", ds_sensors::Sbe49::OutputFormat::RawDecimal }
  };

  for (const auto& pair : pairs)
  {
    // Construct a ByteSequence message
    auto line = pair.first;
    auto format = pair.second;
    auto byte_msg = ds_core_msgs::RawData{};
    byte_msg.ds_header.io_time = ros::Time::now();
    byte_msg.data = std::vector<unsigned char>(std::begin(line), std::end(line));

    // Attempt to parse
    auto result = ds_sensors::Sbe49::parse_bytes(byte_msg, format);
    EXPECT_FALSE(result.first);
  }
}

TEST_F(Sbe49Test, validParses)
{
  // Simple function to create a ds_sensors_msg::Ctd from arguments
  auto ctd = [](float temp, float cond, float pres, float sal, float speed) {

    auto msg = ds_sensor_msgs::Ctd{};
    msg.temperature = temp;
    msg.conductivity = cond;
    msg.pressure = pres;
    msg.salinity = sal;
    msg.sound_speed = speed;

    return msg;
  };

  // Add new test cases here.  These should pass
  const auto test_pairs = std::list<std::pair<std::string, ds_sensor_msgs::Ctd>>{
    { "4.5078,  3.29269,  876.339,  34.4446, 1482.420\r\n", ctd(4.5078, 3.29269, 876.339, 34.4446, 1482.420) },

    { "4.4455,  3.28784,  884.217,  34.4486, 1482.298\r\n", ctd(4.4455, 3.28784, 884.217, 34.4486, 1482.298) },

    // 3-element strings are valid
    { "4.4455,  3.28784,  884.217\r\n",
      ctd(4.4455, 3.28784, 884.217, ds_sensor_msgs::Ctd::CTD_NO_DATA, ds_sensor_msgs::Ctd::CTD_NO_DATA) }
  };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto test_str = test_pair.first;
    auto test_ctd = test_pair.second;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    auto ok = false;
    auto ctd = ds_sensor_msgs::Ctd{};

    std::tie(ok, ctd) = ds_sensors::Sbe49::parse_bytes(byte_msg);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), ctd.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_ctd.temperature, ctd.temperature);
    EXPECT_FLOAT_EQ(test_ctd.conductivity, ctd.conductivity);
    EXPECT_FLOAT_EQ(test_ctd.pressure, ctd.pressure);
    EXPECT_FLOAT_EQ(test_ctd.salinity, ctd.salinity);
    EXPECT_FLOAT_EQ(test_ctd.sound_speed, ctd.sound_speed);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
