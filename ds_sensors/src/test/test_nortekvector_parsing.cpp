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
// Created by jvaccaro on 10/03/18.
//

#include "ds_sensors/nortekvector.h"

#include <gtest/gtest.h>

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class NortekVectorTest : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

TEST_F(NortekVectorTest, validVelocity3DParses)
{
  // Add new test cases here.  These should pass
  const auto test_pairs = std::list<std::pair<std::vector<unsigned char>, bool>>{
    // VELOCITY DATA PASS
    { { 0xa5, 0x10, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb3, 0xfd,
        0xb4, '"',  0xa1, 0xff, '6',  '7',  '5',  0x0b, '\r', 0x07, 0xb1, 0xf1 },
      true },
    { { 0xa5, 0x10, 0x00, 'r',  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00,
        'c',  0xfb, 0x80, 0xff, '7',  '6',  '4',  '&',  '!',  0x16, 0xb9, 0xa5 },
      true },
    { { 0xa5, 0x10, 0x00, 's',  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xfc,
        0xc3, 0x01, '+',  0x00, '7',  '7',  '6',  0x03, '\r', '&',  'J',  0x98 },
      true },
    { { 0xa5, 0x10, 0x00, 't',  0x00, 0x00, 0xbe, 0x02, 0x00, 0x00, 'v',  0xf2,
        0x1f, 0xd6, 0xf6, 0x00, '6',  '6',  '5',  0x14, 0x0e, 0x11, 0xf3, 'a' },
      true }
  };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto test_str = test_pair.first;
    auto test_res = test_pair.second;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = test_str;

    auto ok_velocity = false;
    auto msg_velocity = ds_sensor_msgs::Velocity3D{};
    auto ok_system = false;
    auto msg_system = ds_sensor_msgs::NortekVectorSystem{};

    std::tie(ok_velocity, ok_system, msg_velocity, msg_system) = ds_sensors::NortekVector::parse_bytes(byte_msg);

    // Should have succeeded
    EXPECT_EQ(test_res, ok_velocity);
    EXPECT_NE(test_res, ok_system);
  }
}

TEST_F(NortekVectorTest, validNortekVectorSystemParses)
{
  // Add new test cases here.  These should pass if true and fail if false
  const auto test_pairs = std::list<std::pair<std::vector<unsigned char>, bool>>{
    // VELOCITY DATA PASS
    // SYSTEM DATA PASS
    { { 0xa5, 0x11, 0x0e, 0x00, ')', 0x02, 0x03, 0x01, 0x02, 0x01, 0x93, 0x00, 0x98, ':',
        0xb6, 0x0b, 0xe7, 0xff, '1', 0xff, '3',  '\t', 0x00, '`',  0x00, 0x00, 0x99, 'z' },
      true }
  };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto test_str = test_pair.first;
    auto test_res = test_pair.second;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = test_str;

    auto ok_velocity = false;
    auto msg_velocity = ds_sensor_msgs::Velocity3D{};
    auto ok_system = false;
    auto msg_system = ds_sensor_msgs::NortekVectorSystem{};

    std::tie(ok_velocity, ok_system, msg_velocity, msg_system) = ds_sensors::NortekVector::parse_bytes(byte_msg);

    // Should have succeeded
    EXPECT_EQ(test_res, ok_system);
    EXPECT_NE(test_res, ok_velocity);
  }
}

TEST_F(NortekVectorTest, failingParses)
{
  // Add new test cases here.  These should fail
  const auto test_pairs = std::list<std::pair<std::vector<unsigned char>, bool>>{
    // VELOCITY DATA PASS
    // VELOCITY SYSTEM FAIL - CHECKSUM
    { { 0xa5, 0x10, 0x00, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb3, 0xfd,
        0xb4, '"',  0xa1, 0xff, '6',  '7',  '5',  0x0b, '\r', 0x07, 0xb1, 0xf2 },
      false },
    // VELOCITY SYSTEM FAIL - BAD SYNC
    { { 0xa6, 0x10, 0x00, 'r',  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x00,
        'c',  0xfb, 0x80, 0xff, '7',  '6',  '4',  '&',  '!',  0x16, 0xb9, 0xa5 },
      false },
    // VELOCTY SYSTEM FAIL - BAD ID
    { { 0xa5, 0x11, 0x00, 's',  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb1, 0xfc,
        0xc3, 0x01, '+',  0x00, '7',  '7',  '6',  0x03, '\r', '&',  'J',  0x98 },
      false },
    { { 0xa5, 0x15, 0x00, 't',  0x00, 0x00, 0xbe, 0x02, 0x00, 0x00, 'v',  0xf2,
        0x1f, 0xd6, 0xf6, 0x00, '6',  '6',  '5',  0x14, 0x0e, 0x11, 0xf3, 'a' },
      false },
    // SYSTEM DATA FAIL - BAD ID
    { { 0xa5, 0x12, 0x0e, 0x00, ')', 0x02, 0x03, 0x01, 0x02, 0x01, 0x93, 0x00, 0x98, ':',
        0xb6, 0x0b, 0xe7, 0xff, '1', 0xff, '3',  '\t', 0x00, '`',  0x00, 0x00, 0x99, 'z' },
      false },
    // Too short
    { { 0xa5 }, false },
    { {}, false },
    { { 0xa5, 0x11 }, false },
    { { 0xa5, 0x10 }, false }
  };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto test_str = test_pair.first;
    auto test_res = test_pair.second;

    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    auto now = ros::Time::now();
    byte_msg.ds_header.io_time = now;
    byte_msg.data = test_str;

    auto ok_velocity = false;
    auto msg_velocity = ds_sensor_msgs::Velocity3D{};
    auto ok_system = false;
    auto msg_system = ds_sensor_msgs::NortekVectorSystem{};

    std::tie(ok_velocity, ok_system, msg_velocity, msg_system) = ds_sensors::NortekVector::parse_bytes(byte_msg);

    // Should have failed
    EXPECT_EQ(test_res, ok_velocity);
    EXPECT_EQ(test_res, ok_system);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
