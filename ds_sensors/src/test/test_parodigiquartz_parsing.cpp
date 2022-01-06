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
// Created by zac on 12/10/17.
//

#include "ds_sensors/parodigiquartz.h"

#include <list>
#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class ParoDigiquartzTest : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

TEST_F(ParoDigiquartzTest, validPressureParses)
{
  // Simple function to create a ds_sensors_msg::VectorMagneticField from arguments
  auto depth = [](double pressure) {
    auto msg = ds_sensor_msgs::DepthPressure{};
    msg.pressure_raw = pressure;
    msg.depth = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
    msg.latitude = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
    msg.tare = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
    return msg;
  };

  // Add new test cases here.  These should pass
  const auto test_pairs =
      std::list<std::pair<std::string, ds_sensor_msgs::DepthPressure>>{ { "*00012991.11891", depth(2991.11891) },
                                                                        { "*00012991.06579", depth(2991.06579) },
                                                                        { "*00012991.01074", depth(2991.01074) },
                                                                        { "*0001P4=16.7509,23.01532\r\n",
                                                                          depth(16.7509) } };

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
    auto parsed_msg = ds_sensor_msgs::DepthPressure{};

    std::tie(ok, parsed_msg) = ds_sensors::ParoDigiquartz::parse_pressure(byte_msg);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_msg.pressure_raw, parsed_msg.pressure_raw);
    EXPECT_FLOAT_EQ(test_msg.tare, parsed_msg.tare);
    EXPECT_FLOAT_EQ(test_msg.latitude, parsed_msg.latitude);
    EXPECT_FLOAT_EQ(test_msg.depth, parsed_msg.depth);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
