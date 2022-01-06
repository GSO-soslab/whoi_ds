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
#include "ds_sensors/remushtpgl.h"

#include <list>
#include <gtest/gtest.h>

class RemusHtpglTest : public ::testing::Test
{
public:
  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }
};

// This test case should capture known-bad strings that should fail parsing.
TEST_F(RemusHtpglTest, failingParses)
{
  // Our bad strings.  Please add a brief indication why the string is bad if not obvious.
  const auto lines = std::list<std::string>{
    "1019.3,2>8.8,42.6,0.0,0,0\r\n",                                                      // corruption
    "1019.3,28.8,0,0\r\n",                                                                // Too few fields
    "WHOI REMUS Environmental Board 107811 FW: v1.0.0 Aug 24 2016 13:45:15 L.Frey.\r\n",  // welcome message line 1
    "CAL: C1:41636 C2:44287 C3:25151 C4:26824 C5:33730 C6:26943 M:0.4878 B:-1.4634\r\n",  // welcome message line 2
    "DATA FORMAT:Press(mb),Temp(C),Humid(%),GF(%),ProbeFAIL?,Leak?\r\n",                  // welcome message line 3
  };

  for (const auto& line : lines)
  {
    // Construct a ByteSequence message
    auto byte_msg = ds_core_msgs::RawData{};
    byte_msg.ds_header.io_time = ros::Time::now();
    byte_msg.data = std::vector<unsigned char>(std::begin(line), std::end(line));

    // Attempt to parse
    auto result = ds_sensors::RemusHtpgl::parse_bytes(byte_msg);
    EXPECT_FALSE(result.first);
  }
}

TEST_F(RemusHtpglTest, validParses)
{
  // Simple function to create a ds_sensors_msg::VectorMagneticField from arguments
  auto htp = [](float h, float t, float p, float gfp, bool fault, bool leak) {

    auto msg = ds_hotel_msgs::HTPGL{};
    msg.htp.humidity = h;
    msg.htp.temperature = t;
    msg.htp.pressure = p;
    msg.ground_fault = gfp;
    msg.probe_fail = fault;
    msg.leak = leak;
    return msg;
  };

  // Add new test cases here.  These should pass
  const auto test_pairs = std::list<std::pair<std::string, ds_hotel_msgs::HTPGL>>{ { "1019.3,28.8,42.6,2.0,0,0\r\n",
                                                                                   htp(42.6, 28.8, 14.7836966, 2.0, false, false) } };

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
    auto parsed_msg = ds_hotel_msgs::HTPGL{};

    std::tie(ok, parsed_msg) = ds_sensors::RemusHtpgl::parse_bytes(byte_msg);

    // Should have succeeded
    EXPECT_TRUE(ok);

    // All fields should match.
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.header.stamp.toSec());
    EXPECT_FLOAT_EQ(now.toSec(), parsed_msg.htp.header.stamp.toSec());
    EXPECT_FLOAT_EQ(test_msg.htp.humidity, parsed_msg.htp.humidity);
    EXPECT_FLOAT_EQ(test_msg.htp.temperature, parsed_msg.htp.temperature);
    EXPECT_FLOAT_EQ(test_msg.htp.pressure, parsed_msg.htp.pressure);
    EXPECT_FLOAT_EQ(test_msg.ground_fault, parsed_msg.ground_fault);
    EXPECT_EQ(test_msg.leak, parsed_msg.leak);
    EXPECT_EQ(test_msg.probe_fail, parsed_msg.probe_fail);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}