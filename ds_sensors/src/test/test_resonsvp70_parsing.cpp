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

#include "ds_sensors/resonsvp70.h"

#include <list>
#include <gtest/gtest.h>

using OutputFormat = ds_sensors::ResonSvp70::OutputFormat;

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class ResonSvp70Test : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

#if 0
// This test case should capture known-bad strings that should fail parsing.
TEST_F(ResonSvp70Test, failingParses)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto lines = std::list<std::string>  {
    "4.>024,  3.30013,  863.29M-s,  34.4399, 1482.588\r\n",   // corruption
    "4.5078,  3.29269,  876.339,  34.4446\r\n",  // Strings can have 3 or 5 fields, never 4
    "4.5078,  3.29269\r\n",                      // Too few fields
  };

  for(const auto& line: lines) {
    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    byte_msg.header.io_time = ros::Time::now();
    byte_msg.data = std::vector<unsigned char>(std::begin(line), std::end(line));

    // Attempt to parse
    auto result = ds_sensors::ResonSvp70::parse_bytes(byte_msg, 0, 10000);
    EXPECT_FALSE(result.first);
  }
}
#endif

// This test case should capture valid strings that we don't have parsers for
TEST_F(ResonSvp70Test, unimplementedParsers)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto pairs = std::list<std::pair<std::string, ds_sensors::ResonSvp70::OutputFormat>>{
    { "$PSOS 123234,1450.002,20.3,102.5,210,9,0,1\r\n", ds_sensors::ResonSvp70::OutputFormat::Svp7x },
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
    auto result = ds_sensors::ResonSvp70::parse_bytes(byte_msg, 0, 10000, format);
    EXPECT_FALSE(result.first);
  }
}

TEST_F(ResonSvp70Test, validParses)
{
  // Simple function to create a ds_sensors_msg::SoundSpeed from arguments
  auto sound_speed = [](float speed) {
    auto msg = ds_sensor_msgs::SoundSpeed{};
    msg.speed = speed;
    return msg;
  };

  // Add new test cases here.  These should pass
  auto test_tuples = std::list<std::tuple<std::string, OutputFormat, ds_sensor_msgs::SoundSpeed>>{};
  test_tuples.emplace_back(" 1543.39 \r\n", OutputFormat::Aml, sound_speed(1543.39));
  test_tuples.emplace_back(" 1543390\r\n", OutputFormat::Valeport, sound_speed(1543.39));
  test_tuples.emplace_back("1490803673128101010\r\n", OutputFormat::Reson, sound_speed(1490.8));

  // Loop through all provided cases
  for (const auto& test_tuple : test_tuples)
  {
    auto test_str = std::string{};
    auto format = OutputFormat{};
    auto test_msg = ds_sensor_msgs::SoundSpeed{};

    std::tie(test_str, format, test_msg) = test_tuple;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::SoundSpeed{};

    std::tie(ok, parsed_msg) = ds_sensors::ResonSvp70::parse_bytes(byte_msg, 0, 10000, format);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_msg.speed, parsed_msg.speed);
  }
}

TEST_F(ResonSvp70Test, clampingMin)
{
  // Simple function to create a ds_sensors_msg::SoundSpeed from arguments
  auto sound_speed = [](float speed) {
    auto msg = ds_sensor_msgs::SoundSpeed{};
    msg.speed = speed;
    return msg;
  };

  // Add new test cases here.  These should pass
  auto test_tuples = std::list<std::tuple<std::string, OutputFormat, ds_sensor_msgs::SoundSpeed>>{};
  test_tuples.emplace_back(" 1400.00 \r\n", OutputFormat::Aml, sound_speed(1450.00));

  // Loop through all provided cases
  for (const auto& test_tuple : test_tuples)
  {
    auto test_str = std::string{};
    auto format = OutputFormat{};
    auto test_msg = ds_sensor_msgs::SoundSpeed{};

    std::tie(test_str, format, test_msg) = test_tuple;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::SoundSpeed{};

    std::tie(ok, parsed_msg) = ds_sensors::ResonSvp70::parse_bytes(byte_msg, 1450, 1575, format);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_msg.speed, parsed_msg.speed);
  }
}

TEST_F(ResonSvp70Test, clampingMax)
{
  // Simple function to create a ds_sensors_msg::SoundSpeed from arguments
  auto sound_speed = [](float speed) {
    auto msg = ds_sensor_msgs::SoundSpeed{};
    msg.speed = speed;
    return msg;
  };

  // Add new test cases here.  These should pass
  auto test_tuples = std::list<std::tuple<std::string, OutputFormat, ds_sensor_msgs::SoundSpeed>>{};
  test_tuples.emplace_back(" 1700.00 \r\n", OutputFormat::Aml, sound_speed(1575.00));

  // Loop through all provided cases
  for (const auto& test_tuple : test_tuples)
  {
    auto test_str = std::string{};
    auto format = OutputFormat{};
    auto test_msg = ds_sensor_msgs::SoundSpeed{};

    std::tie(test_str, format, test_msg) = test_tuple;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::SoundSpeed{};

    std::tie(ok, parsed_msg) = ds_sensors::ResonSvp70::parse_bytes(byte_msg, 1450, 1575, format);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_msg.speed, parsed_msg.speed);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
