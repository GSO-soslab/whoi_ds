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

#include "ds_sensors/aps1540.h"

#include <list>
#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class Aps1540Test : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

// This test case should capture known-bad strings that should fail parsing.
TEST_F(Aps1540Test, failingParses)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto lines = std::list<std::string>{
    "4.>024,  3.30013,  863.29M-s,  34.4399, 1482.588\n",  // corruption
    "4.5078,  3.29269\n",                                  // Too few fields
  };

  for (const auto& line : lines)
  {
    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    byte_msg.ds_header.io_time = ros::Time::now();
    byte_msg.data = std::vector<unsigned char>(std::begin(line), std::end(line));

    // Attempt to parse
    auto result = ds_sensors::Aps1540::parse_bytes(byte_msg);
    EXPECT_FALSE(result.first);
  }
}

TEST_F(Aps1540Test, validParses)
{
  // Simple function to create a ds_sensors_msg::VectorMagneticField from arguments
  auto mag = [](float x, float y, float z, float temp) {

    auto msg = ds_sensor_msgs::VectorMagneticField{};
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.temperature = temp;

    return msg;
  };

  // Add new test cases here.  These should pass
  const auto test_pairs = std::list<std::pair<std::string, ds_sensor_msgs::VectorMagneticField>>{
    { "+0.2836376      +0.1940482      +0.2217853        +1.094\n", mag(+0.2836376, 0.1940482, 0.2217853, 1.094) },
    { "+0.2740051      -0.1856363      -0.2074859        -0.356\n", mag(+0.2740051, -0.1856363, -0.2074859, -0.356) },
    { "+0.2835127      +0.1927167      +0.2087637        +0.026\n", mag(+0.2835127, +0.1927167, +0.2087637, +0.026) },
    { "+0.2838366      +0.1934168      +0.2213677        +1.079\n", mag(+0.2838366, +0.1934168, +0.2213677, +1.079) },
    { "+0.2743686      -0.1852370      -0.2076398        -0.356\n", mag(+0.2743686, -0.1852370, -0.2076398, -0.356) }

  };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto test_str = test_pair.first;
    auto test_msg = test_pair.second;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = std::vector<unsigned char>(std::begin(test_str), std::end(test_str));

    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::VectorMagneticField{};

    std::tie(ok, parsed_msg) = ds_sensors::Aps1540::parse_bytes(byte_msg);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_msg.x, parsed_msg.x);
    EXPECT_FLOAT_EQ(test_msg.y, parsed_msg.y);
    EXPECT_FLOAT_EQ(test_msg.z, parsed_msg.z);
    EXPECT_FLOAT_EQ(test_msg.temperature, parsed_msg.temperature);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
