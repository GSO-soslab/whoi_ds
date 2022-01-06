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
// Created by jvaccaro on 11/19/18.
//

#include "ds_sensors/sbe03.h"

#include <list>
#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class Sbe03Test : public ::testing::Test
{
 public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

// This test case should capture known-bad strings that should fail parsing.
TEST_F(Sbe03Test, failingParses)
{
// Our bad strings.  Please add a brief indication why the string is bad if not obvious.
const auto lines = std::list<std::string>{
    {"tdhtfhtfh"},    // Junk
    {"hgfhhfd\r\n"},  // Junk with link terminator
    {"999.999"},      // No line terminator
    {"999.99TT\r\n"}, // Corrupted with line terminator
};

for (const auto& line : lines)
{
// Construct a ByteSequence message
auto byte_msg = ds_core_msgs::RawData{};
byte_msg.ds_header.io_time = ros::Time::now();
byte_msg.data = std::vector<unsigned char>(std::begin(line), std::end(line));

// Attempt to parse
auto result = ds_sensors::Sbe03::parse_bytes(byte_msg);
ROS_INFO_STREAM(line << " : " << result.first);
EXPECT_FALSE(result.first);
}
}

TEST_F(Sbe03Test, validParses)
{
// Simple function to create a frequency msg
auto freq = [](float hz) {

  auto msg = ds_sensor_msgs::Frequency{};
  msg.hz = hz;

  return msg;
};

// Add new test cases here.  These should pass
const auto test_pairs = std::list<std::pair<std::string, double>>{
{ "9999.999\r\n", 9999.999},
{ "1234.123\r\n", 1234.123},
{ "1577.597\r\n", 1577.597},
{ "0015.556\r\nABD", 15.556} // Doesnt catch junk after terminator
};

// Loop through all provided cases
for (const auto& test_pair : test_pairs)
{
auto test_str = test_pair.first;
auto hz = test_pair.second;

// Construct a ByteSequence message
auto byte_msg = ds_core_msgs::RawData{};
auto now = ros::Time::now();
byte_msg.ds_header.io_time = now;
byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

auto ok = false;
auto freq = ds_sensor_msgs::Frequency{};

std::tie(ok, freq) = ds_sensors::Sbe03::parse_bytes(byte_msg);

// Should have succeeded
EXPECT_TRUE(ok);

// All fields should match.
EXPECT_FLOAT_EQ(freq.hz, hz);
}
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



