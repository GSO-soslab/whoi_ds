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
// Created by JLV on 12/11/17.
//

#include "ds_sensors/rdidvl.h"
#include "../ds_sensors/rdidvl_private.h"

#include <list>
#include <gtest/gtest.h>
#include <ros/time.h>
#include "rdidvl_test_data.h"

// Test fixture for parsing, needed because we want to call ros::Time::init() before our tests.
class RdiDvlTest : public ::testing::Test
{
public:
  double beam_angle = 30.0 * M_PI / 180.0;
  bool phased_array = false;

  // This method runs ONCE before a text fixture is run (not once-per-test-case)
  static void SetUpTestCase()
  {
    ros::Time::init();
  }

  // take one of the test datagram literals from rdidvl_test_data.h and wrap it
  // in a nice RawData message
  ds_core_msgs::RawData buildRaw(const std::vector<uint8_t>& raw, size_t offset=0, size_t size=217) {
    ds_core_msgs::RawData ret;
    ret.header.stamp = ros::Time::now();
    ret.ds_header.io_time = ret.header.stamp;
    ret.data_direction = ds_core_msgs::RawData::DATA_IN;
    ret.data.resize(size);
    std::memcpy(ret.data.data(), raw.data()+offset, size);

    return ret;
  }

};

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(RdiDvlTest, totalParsePASS)
{
  auto ok = false;
  auto parsed_msg = ds_sensor_msgs::Dvl{};
  auto pd0 = ds_sensor_msgs::RdiPD0{};
  auto rng = ds_sensor_msgs::Ranges3D{};
  auto byte_msg = buildRaw(rdidvl_test_data::pass);
  std::tie(ok, parsed_msg, pd0, rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, phased_array);
  EXPECT_TRUE(ok);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(RdiDvlTest, headerParsePASS)
{
  bool result = false;
  auto byte_msg = buildRaw(rdidvl_test_data::pass);
  auto hdr = ds_sensors::rdidvl_structs::header();

  std::tie(result, hdr) = ds_sensors::RdiDvl::parseHeader(byte_msg.data.data(), byte_msg.data.size());

  EXPECT_TRUE(result);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Example fail test case
TEST_F(RdiDvlTest, failingParses)
{
  ROS_ERROR_STREAM("Failing by parsing");

  const auto test_nums =
      std::list<int>{
                            1, // start offset from header 7FD7
                               // INFO: ID not recognized

                            3, // start with a non-starting valid header 0000
                               // INFO: ID not recognized

                          107, // start with 7F7F in the middle of a packet
                               // Should parse header, then fail without sending message
                               // INFO: too many data, too many bytes, and checksum failure

                          218, // start with 7F7F at beginning, insert bad address value
                               // Should parse header, then fail when seeking packets
                               // INFO: too many data, checksum failure, address outside of buffer length

                          436, // start with 7F7F, then insert bad data after header
                               // Should parse header, then fail without sending message
                               // INFO: checksum failure

                          654, // start with 7F7F, then remove some data after header
                               // Should parse header, then fail without sending message
                               // INFO: checksum failure

                          870, // start with 7F7F, then tries to go back to the beginning
                               // Should parse header, and two variable leaders, then fail without sending message
                               // INFO: Addr ID not recognized
                      };

  for (const auto& test_num : test_nums)
  {
    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::Dvl{};
    auto parsed_pd0 = ds_sensor_msgs::RdiPD0{};
    auto parsed_rng = ds_sensor_msgs::Ranges3D{};

    auto byte_msg = buildRaw(rdidvl_test_data::fail, test_num);

    std::tie(ok, parsed_msg, parsed_pd0, parsed_rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, phased_array);

    // Should have failed
    EXPECT_FALSE(ok);
  }
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(RdiDvlTest, failingByLength_normal)
{
  ROS_ERROR_STREAM("Failing by length");

  const auto test_nums = std::list<int>{ // Give it length
                                               2,  // should fail
                                               4,  // should fail
                                             107,  // should fail
                                             217,  // should pass
                                             250,
                                             800};
  for (const auto& test_length : test_nums)
  {
    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::Dvl{};
    auto parsed_pd0 = ds_sensor_msgs::RdiPD0{};
    auto parsed_rng = ds_sensor_msgs::Ranges3D{};

    auto byte_msg = buildRaw(rdidvl_test_data::pass, 0, test_length);
    std::tie(ok, parsed_msg, parsed_pd0, parsed_rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, phased_array);

    // Should have failed
    if (test_length >= 217)
    {
      EXPECT_TRUE(ok);
    }
    else
    {
      EXPECT_FALSE(ok);
    }
  }
}


// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(RdiDvlTest, zerolength)
{
  ROS_ERROR_STREAM("Failing by zerolength");

  auto ok = true;
  auto parsed_msg = ds_sensor_msgs::Dvl{};
  auto parsed_pd0 = ds_sensor_msgs::RdiPD0{};
  auto parsed_rng = ds_sensor_msgs::Ranges3D{};

  // Construct a ByteSequence message
  auto byte_msg = ds_core_msgs::RawData{};
  ROS_ERROR_STREAM("Byte msg length:" << byte_msg.data.size());
  byte_msg.data.data();
  ROS_ERROR_STREAM("Passed byte_msg.data.data()");

  byte_msg.ds_header.io_time = ros::Time::now();
  std::tie(ok, parsed_msg, parsed_pd0, parsed_rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, phased_array);

  EXPECT_FALSE(ok);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(RdiDvlTest, validParses)
{
  // Simple function to create a ds_sensors_msg::Dvl from arguments
  auto dvl = [](double x, double y, double z, double r0, double r1, double r2, double r3, double alt, double course,
                double speed) {

    auto msg = ds_sensor_msgs::Dvl{};

    // Velocities [m/s]
    msg.velocity.x = -x;
    msg.velocity.y = -y;
    msg.velocity.z = -z;

    msg.raw_velocity[0] = -x;
    msg.raw_velocity[1] = -y;
    msg.raw_velocity[2] = -z;

    msg.range[0] = r0;
    msg.range[1] = r1;
    msg.range[2] = r2;
    msg.range[3] = r3;

    msg.altitude = alt;
    msg.course_gnd = course;
    msg.speed_gnd = speed;

    for (int i = 0; i < 9; i++)
      msg.velocity_covar[i] = -1;

    msg.num_good_beams = 3;
    msg.speed_sound = 1500;

    msg.beam_unit_vec[0].x = -0.5;
    msg.beam_unit_vec[1].x = .5;
    msg.beam_unit_vec[2].y = .5;
    msg.beam_unit_vec[3].y = -.5;

    for (int i = 0; i < 4; i++)
    {
      msg.beam_unit_vec[i].z = -0.866025403784;  // sqrt(3) / 2
      msg.range_covar[i] = -1;
      msg.raw_velocity_covar[i] = -1;
    }

    msg.velocity_mode = msg.DVL_MODE_BOTTOM;
    msg.coordinate_mode = msg.DVL_COORD_INSTRUMENT;

    return msg;
  };

  auto pd0 = [](u_int16_t ensemble_num, uint8_t second, double heading, double pitch, double roll, double temperature,
                float range2, float vel1, float vel2, float vel3, float corr2, uint8_t amp1, uint8_t amp2, uint8_t amp4,
                uint8_t int1, uint8_t int2, uint8_t int4) {

    auto msg_pd0 = ds_sensor_msgs::RdiPD0{};
    // FIXED LEADER FIELDS
    msg_pd0.fw_ver = 9;
    msg_pd0.fw_rev = 23;
    msg_pd0.config_khz = 300;
    msg_pd0.config_convex = true;
    msg_pd0.config_up = false;
    msg_pd0.config_beamangle = 30;
    msg_pd0.lag = 53;
    msg_pd0.beams = 4;
    msg_pd0.cells = 0;
    msg_pd0.cell_depth = 400;
    msg_pd0.blank = 300;
    msg_pd0.signal_proc = 1;
    msg_pd0.min_thresh = 0;
    msg_pd0.code_reps = 9;
    msg_pd0.min_good_pings = 0;
    msg_pd0.good_thresh = 2000;
    msg_pd0.ping_interval = ros::Duration(0.0);
    msg_pd0.coord_3beam = true;
    msg_pd0.coord_binmapping = true;
    msg_pd0.coord_tilts = false;
    msg_pd0.coord_mode = 1;
    msg_pd0.sensor_avail = 29;
    msg_pd0.sensor_src = 29;
    msg_pd0.bin1_dist = 722;
    msg_pd0.xmit_pulse_len = 400;
    msg_pd0.avg_start = 1;
    msg_pd0.avg_end = 5;
    msg_pd0.avg_false_thresh = 255;
    msg_pd0.trans_lag_dist = 45;
    msg_pd0.serial_num_cpu = { 0xd8, 0x00, 0x00, 0x03, 0x87, 0xb8, 0x17, 0x09 };

    // VARIABLE LEADER FIELDS
    msg_pd0.ensemble_num = ensemble_num;
    msg_pd0.rtc_year = 17;
    msg_pd0.rtc_month = 2;
    msg_pd0.rtc_day = 11;
    msg_pd0.rtc_hour = 12;
    msg_pd0.rtc_minute = 00;
    msg_pd0.rtc_second = second;
    msg_pd0.rtc_hundredth = 24;
    msg_pd0.sound_vel = 1500;
    msg_pd0.depth = 0;
    msg_pd0.heading = heading;
    msg_pd0.pitch = pitch;
    msg_pd0.roll = roll;
    msg_pd0.heading_std = 0;
    msg_pd0.pitch_std = 0;
    msg_pd0.roll_std = 0;
    msg_pd0.salinity = 35;
    msg_pd0.temperature = temperature;
    msg_pd0.pressure = 0;
    msg_pd0.pressure_variance = 0;

    // BOTTOM TRACK FIELDS- most important!
    msg_pd0.pings_per_ensemble = 1;
    msg_pd0.delay = 0;
    msg_pd0.corr_mag_min = 220;
    msg_pd0.eval_amp_min = 30;
    msg_pd0.percent_good_min = 0;
    msg_pd0.mode = 5;
    msg_pd0.err_vel_max = 1000;
    msg_pd0.range = { 182.92, range2, 0, 165.55 };
    msg_pd0.velocity = { vel1, vel2, vel3, 0 };
    msg_pd0.correlation = { 254, corr2, 0, 255 };
    msg_pd0.eval_amp = { amp1, amp2, 0, amp4 };
    msg_pd0.percent_good = { 100, 0, 0, 0 };
    msg_pd0.ref_min = 1600;
    msg_pd0.ref_near = 3200;
    msg_pd0.ref_far = 4800;
    msg_pd0.ref_velocity = { 32768, 32768, 32768, 32768 };
    msg_pd0.ref_correlation = { 0, 0, 0, 0 };
    msg_pd0.ref_intensity = { 0, 0, 0, 0 };
    msg_pd0.ref_percent_good = { 0, 0, 100, 0 };
    msg_pd0.depth_max = 25000;
    msg_pd0.rssi_amp = { int1, int2, 0, int4 };
    msg_pd0.gain = 255;

    return msg_pd0;
  };

  const auto test_pairs = std::list<std::pair<int, std::pair<ds_sensor_msgs::Dvl, ds_sensor_msgs::RdiPD0>>>{
    {
        0,
        { // location (bytes) in binary file2
          dvl(0.539, -0.465, 0.338, 211.21782, 257.50977, 0, 191.16068, 190.49333, 130.78474, 0.7118609),
          //    vx      vy      vz      r0      r1      r2    r3    alt         cog         sog
          pd0(60493, 0, 349.3, 1.53, 0.46, 2.17, 223.01, 0.539, -0.465, 0.338, 254, 54, 31, 74, 167, 151, 179) }
        // ens#  min  hdg    pitch roll  temp   r1     v1      v2       v3     c2  amp1 2   4   int1  2    4
    },
    { 217,
      { dvl(0.542, -0.478, 0.343, 211.21782, 251.33212, 0, 191.16068, 188.71, 131.40979, 0.72266728),
        pd0(60494, 1, 349.4, 1.41, 0.55, 2.14, 217.66, 0.542, -0.478, 0.343, 254, 59, 32, 72, 174, 149, 175) } },
    { 434,
      { dvl(0.532, -0.484, 0.34, 211.21782, 254.41518, 0, 191.16068, 189.60001, 132.29524, 0.71922177),
        pd0(60495, 2, 349.56, 1.16, 0.58, 2.16, 220.33, 0.532, -0.484, 0.340, 255, 56, 35, 73, 169, 154, 176) } },
    { 651,
      { dvl(0.537, -0.483, 0.339, 211.21782, 251.33212, 0, 191.16068, 188.71001, 131.96964, 0.72225899),
        pd0(60496, 3, 349.66, 1.05, 0.51, 2.19, 217.66, 0.537, -0.483, 0.339, 254, 57, 36, 74, 170, 155, 175) } }
  };

  // Loop through all provided cases
  for (const auto& test_pair : test_pairs)
  {
    auto test_msg = test_pair.second.first;
    auto test_pd0 = test_pair.second.second;

    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::Dvl{};
    auto parsed_pd0 = ds_sensor_msgs::RdiPD0{};
    auto parsed_rng = ds_sensor_msgs::Ranges3D{};
    auto byte_msg = buildRaw(rdidvl_test_data::pass, test_pair.first);

    std::tie(ok, parsed_msg, parsed_pd0, parsed_rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, phased_array);

    // Should have succeeded
    EXPECT_TRUE(ok);

    //      DVL ENTRIES -> lower priority but still important.
    // All fields should match.
    EXPECT_FLOAT_EQ(test_msg.velocity.x, parsed_msg.velocity.x);
    EXPECT_FLOAT_EQ(test_msg.velocity.y, parsed_msg.velocity.y);
    EXPECT_FLOAT_EQ(test_msg.velocity.z, parsed_msg.velocity.z);

    for (int i = 0; i < 9; i++)
    {
      EXPECT_FLOAT_EQ(test_msg.velocity_covar[0], parsed_msg.velocity_covar[i]);
    }
    EXPECT_FLOAT_EQ(test_msg.speed_sound, parsed_msg.speed_sound);
    EXPECT_FLOAT_EQ(test_msg.num_good_beams, parsed_msg.num_good_beams);
    for (int i = 0; i < 4; i++)
    {
      EXPECT_FLOAT_EQ(test_msg.beam_unit_vec[i].x, parsed_msg.beam_unit_vec[i].x);
      EXPECT_FLOAT_EQ(test_msg.beam_unit_vec[i].y, parsed_msg.beam_unit_vec[i].y);
      EXPECT_FLOAT_EQ(test_msg.beam_unit_vec[i].z, parsed_msg.beam_unit_vec[i].z);
      EXPECT_FLOAT_EQ(test_msg.range[i], parsed_msg.range[i]);
      EXPECT_FLOAT_EQ(test_msg.range_covar[i], parsed_msg.range_covar[i]);
      EXPECT_FLOAT_EQ(test_msg.raw_velocity[i], parsed_msg.raw_velocity[i]);
      EXPECT_FLOAT_EQ(test_msg.raw_velocity_covar[i], parsed_msg.raw_velocity_covar[i]);
    }

    EXPECT_FLOAT_EQ(test_msg.velocity_mode, parsed_msg.velocity_mode);
    EXPECT_FLOAT_EQ(test_msg.coordinate_mode, parsed_msg.coordinate_mode);
    EXPECT_FLOAT_EQ(test_msg.altitude, parsed_msg.altitude);
    EXPECT_FLOAT_EQ(test_msg.course_gnd, parsed_msg.course_gnd);
    EXPECT_FLOAT_EQ(test_msg.speed_gnd, parsed_msg.speed_gnd);

    // VARIABLE LEADER MATCHING
    EXPECT_FLOAT_EQ(test_pd0.ensemble_num, parsed_pd0.ensemble_num);
    EXPECT_FLOAT_EQ(test_pd0.rtc_year, parsed_pd0.rtc_year);
    EXPECT_FLOAT_EQ(test_pd0.rtc_month, parsed_pd0.rtc_month);
    EXPECT_FLOAT_EQ(test_pd0.rtc_day, parsed_pd0.rtc_day);
    EXPECT_FLOAT_EQ(test_pd0.rtc_hour, parsed_pd0.rtc_hour);
    EXPECT_FLOAT_EQ(test_pd0.rtc_minute, parsed_pd0.rtc_minute);
    EXPECT_FLOAT_EQ(test_pd0.rtc_second, parsed_pd0.rtc_second);
    EXPECT_FLOAT_EQ(test_pd0.rtc_hundredth, parsed_pd0.rtc_hundredth);

    EXPECT_FLOAT_EQ(test_pd0.sound_vel, parsed_pd0.sound_vel);
    EXPECT_FLOAT_EQ(test_pd0.depth, parsed_pd0.depth);
    EXPECT_FLOAT_EQ(test_pd0.heading, parsed_pd0.heading);
    EXPECT_FLOAT_EQ(test_pd0.pitch, parsed_pd0.pitch);
    EXPECT_FLOAT_EQ(test_pd0.roll, parsed_pd0.roll);
    EXPECT_FLOAT_EQ(test_pd0.heading_std, parsed_pd0.heading_std);
    EXPECT_FLOAT_EQ(test_pd0.pitch_std, parsed_pd0.pitch_std);
    EXPECT_FLOAT_EQ(test_pd0.roll_std, parsed_pd0.roll_std);
    EXPECT_FLOAT_EQ(test_pd0.salinity, parsed_pd0.salinity);
    EXPECT_FLOAT_EQ(test_pd0.temperature, parsed_pd0.temperature);
    EXPECT_FLOAT_EQ(test_pd0.pressure, parsed_pd0.pressure);
    EXPECT_FLOAT_EQ(test_pd0.pressure_variance, parsed_pd0.pressure_variance);

    // SS - computation of unixtime
    EXPECT_FLOAT_EQ(1486814400.240 + test_pd0.rtc_second, parsed_pd0.dvl_time);

    // FIXED LEADER MATCHING
    EXPECT_FLOAT_EQ(test_pd0.fw_ver, parsed_pd0.fw_ver);
    EXPECT_FLOAT_EQ(test_pd0.fw_rev, parsed_pd0.fw_rev);
    EXPECT_FLOAT_EQ(test_pd0.config_khz, parsed_pd0.config_khz);
    EXPECT_FLOAT_EQ(test_pd0.config_convex, parsed_pd0.config_convex);
    EXPECT_FLOAT_EQ(test_pd0.config_up, parsed_pd0.config_up);
    EXPECT_FLOAT_EQ(test_pd0.config_beamangle, parsed_pd0.config_beamangle);
    EXPECT_FLOAT_EQ(test_pd0.lag, parsed_pd0.lag);
    EXPECT_FLOAT_EQ(test_pd0.beams, parsed_pd0.beams);
    EXPECT_FLOAT_EQ(test_pd0.cell_depth, parsed_pd0.cell_depth);
    EXPECT_FLOAT_EQ(test_pd0.blank, parsed_pd0.blank);
    EXPECT_FLOAT_EQ(test_pd0.signal_proc, parsed_pd0.signal_proc);
    EXPECT_FLOAT_EQ(test_pd0.min_thresh, parsed_pd0.min_thresh);
    EXPECT_FLOAT_EQ(test_pd0.code_reps, parsed_pd0.code_reps);
    EXPECT_FLOAT_EQ(test_pd0.min_good_pings, parsed_pd0.min_good_pings);
    EXPECT_FLOAT_EQ(test_pd0.good_thresh, parsed_pd0.good_thresh);
    EXPECT_FLOAT_EQ(test_pd0.ping_interval.toSec(), parsed_pd0.ping_interval.toSec());
    EXPECT_FLOAT_EQ(test_pd0.coord_3beam, parsed_pd0.coord_3beam);
    EXPECT_FLOAT_EQ(test_pd0.coord_binmapping, parsed_pd0.coord_binmapping);
    EXPECT_FLOAT_EQ(test_pd0.coord_tilts, parsed_pd0.coord_tilts);
    EXPECT_FLOAT_EQ(test_pd0.coord_mode, parsed_pd0.coord_mode);
    EXPECT_FLOAT_EQ(test_pd0.sensor_avail, parsed_pd0.sensor_avail);
    EXPECT_FLOAT_EQ(test_pd0.sensor_src, parsed_pd0.sensor_src);

    EXPECT_FLOAT_EQ(test_pd0.bin1_dist, parsed_pd0.bin1_dist);
    EXPECT_FLOAT_EQ(test_pd0.xmit_pulse_len, parsed_pd0.xmit_pulse_len);
    EXPECT_FLOAT_EQ(test_pd0.avg_start, parsed_pd0.avg_start);
    EXPECT_FLOAT_EQ(test_pd0.avg_end, parsed_pd0.avg_end);
    EXPECT_FLOAT_EQ(test_pd0.avg_false_thresh, parsed_pd0.avg_false_thresh);
    EXPECT_FLOAT_EQ(test_pd0.trans_lag_dist, parsed_pd0.trans_lag_dist);
    for (int k = 0; k < 8; k++)
    {
      EXPECT_FLOAT_EQ(test_pd0.serial_num_cpu[k], parsed_pd0.serial_num_cpu[k]);
    }

    // BOTTOM TRACK MATCHING
    EXPECT_FLOAT_EQ(test_pd0.pings_per_ensemble, parsed_pd0.pings_per_ensemble);
    EXPECT_FLOAT_EQ(test_pd0.delay, parsed_pd0.delay);
    EXPECT_FLOAT_EQ(test_pd0.corr_mag_min, parsed_pd0.corr_mag_min);
    EXPECT_FLOAT_EQ(test_pd0.eval_amp_min, parsed_pd0.eval_amp_min);
    EXPECT_FLOAT_EQ(test_pd0.percent_good_min, parsed_pd0.percent_good_min);
    EXPECT_FLOAT_EQ(test_pd0.mode, parsed_pd0.mode);
    EXPECT_FLOAT_EQ(test_pd0.err_vel_max, parsed_pd0.err_vel_max);
    for (int j = 0; j < 4; j++)
    {
      EXPECT_FLOAT_EQ(test_pd0.range[j], parsed_pd0.range[j]);
      EXPECT_FLOAT_EQ(test_pd0.velocity[j], parsed_pd0.velocity[j]);
      EXPECT_FLOAT_EQ(test_pd0.correlation[j], parsed_pd0.correlation[j]);
      EXPECT_FLOAT_EQ(test_pd0.eval_amp[j], parsed_pd0.eval_amp[j]);
      EXPECT_FLOAT_EQ(test_pd0.percent_good[j], parsed_pd0.percent_good[j]);
      EXPECT_FLOAT_EQ(test_pd0.ref_velocity[j], parsed_pd0.ref_velocity[j]);
      EXPECT_FLOAT_EQ(test_pd0.ref_intensity[j], parsed_pd0.ref_intensity[j]);
      EXPECT_FLOAT_EQ(test_pd0.ref_percent_good[j], parsed_pd0.ref_percent_good[j]);
      EXPECT_FLOAT_EQ(test_pd0.rssi_amp[j], parsed_pd0.rssi_amp[j]);
    }

    EXPECT_FLOAT_EQ(test_pd0.ref_min, parsed_pd0.ref_min);
    EXPECT_FLOAT_EQ(test_pd0.ref_near, parsed_pd0.ref_near);
    EXPECT_FLOAT_EQ(test_pd0.ref_far, parsed_pd0.ref_far);
    EXPECT_FLOAT_EQ(test_pd0.depth_max, parsed_pd0.depth_max);
    EXPECT_FLOAT_EQ(test_pd0.gain, parsed_pd0.gain);

    // ADDITIONAL FIELDS
    EXPECT_FALSE(parsed_pd0.highres_valid);
    EXPECT_FALSE(parsed_pd0.btrange_valid);
    EXPECT_FALSE(parsed_pd0.navp_valid);
  }
}

TEST_F(RdiDvlTest, BadSoundVelocity)
{
  const auto test_pairs = std::list<std::pair<int, bool>>{ { 0, true },
                                                           { 217, true },
                                                           { 434, true },
                                                           { 651, false },  // This is the only one that fails. Why?
                                                           { 868, true } };
  for (const auto& test_pair : test_pairs)
  {
    auto ok = false;
    auto parsed_msg = ds_sensor_msgs::Dvl{};
    auto parsed_pd0 = ds_sensor_msgs::RdiPD0{};
    auto parsed_rng = ds_sensor_msgs::Ranges3D{};

    auto byte_msg = buildRaw(rdidvl_test_data::badsoundspeed, test_pair.first);
    std::tie(ok, parsed_msg, parsed_pd0, parsed_rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, phased_array);

    // Should pass
    EXPECT_EQ(ok, test_pair.second);
    if (test_pair.second)
    {
      EXPECT_EQ(parsed_msg.speed_sound, 1500);
    }
  }
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
TEST_F(RdiDvlTest, pioneerFields) {
  auto byte_msg = buildRaw(rdidvl_test_data::pioneer, 0, rdidvl_test_data::pioneer.size());

  auto ok = true;
  auto parsed_msg = ds_sensor_msgs::Dvl{};
  auto parsed_pd0 = ds_sensor_msgs::RdiPD0{};
  auto parsed_rng = ds_sensor_msgs::Ranges3D{};
  std::tie(ok, parsed_msg, parsed_pd0, parsed_rng) = ds_sensors::RdiDvl::parse_bytes(byte_msg, beam_angle, true);

  ASSERT_TRUE(ok);

  // now check fields

  // /////////////////////////////////////////////////////////////////////// //
  // highres
  EXPECT_TRUE(parsed_pd0.highres_valid);

  EXPECT_FLOAT_EQ( 0.022949999, parsed_pd0.highres_bt_velocity[0]);
  EXPECT_FLOAT_EQ( 0.014240000, parsed_pd0.highres_bt_velocity[1]);
  EXPECT_FLOAT_EQ(-0.017999999, parsed_pd0.highres_bt_velocity[2]);
  EXPECT_FLOAT_EQ(-0.0079600001, parsed_pd0.highres_bt_velocity[3]);

  EXPECT_FLOAT_EQ(-1.645470, parsed_pd0.highres_bt_dmg[0]);
  EXPECT_FLOAT_EQ( 0.80550998, parsed_pd0.highres_bt_dmg[1]);
  EXPECT_FLOAT_EQ(-3.7834001, parsed_pd0.highres_bt_dmg[2]);
  EXPECT_FLOAT_EQ(-0.2436500, parsed_pd0.highres_bt_dmg[3]);

  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_velocity[0]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_velocity[1]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_velocity[2]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_velocity[3]);

  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_dmg[0]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_dmg[1]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_dmg[2]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.highres_wm_dmg[3]);

  EXPECT_FLOAT_EQ(1500.0000, parsed_pd0.highres_sound_vel);

  // /////////////////////////////////////////////////////////////////////// //
  // btrange
  EXPECT_TRUE(parsed_pd0.btrange_valid);

  EXPECT_FLOAT_EQ(13.937900,parsed_pd0.btrange_slant_range);
  EXPECT_FLOAT_EQ( 0.053599998, parsed_pd0.btrange_axis_delta_range);
  EXPECT_FLOAT_EQ(13.937900, parsed_pd0.btrange_vertical_range);
  EXPECT_EQ(100, parsed_pd0.btrange_pct_good_4beam);
  EXPECT_EQ(0, parsed_pd0.btrange_pct_good_beam12);
  EXPECT_EQ(0, parsed_pd0.btrange_pct_good_beam34);

  EXPECT_FLOAT_EQ(13.567500, parsed_pd0.btrange_raw_range[0]);
  EXPECT_FLOAT_EQ(14.385700, parsed_pd0.btrange_raw_range[1]);
  EXPECT_FLOAT_EQ(16.294701, parsed_pd0.btrange_raw_range[2]);
  EXPECT_FLOAT_EQ(12.135800, parsed_pd0.btrange_raw_range[3]);

  EXPECT_EQ(51, parsed_pd0.btrange_max_filter[0]);
  EXPECT_EQ(38, parsed_pd0.btrange_max_filter[1]);
  EXPECT_EQ(45, parsed_pd0.btrange_max_filter[2]);
  EXPECT_EQ(57, parsed_pd0.btrange_max_filter[3]);

  EXPECT_EQ(156, parsed_pd0.btrange_max_amp[0]);
  EXPECT_EQ(134, parsed_pd0.btrange_max_amp[1]);
  EXPECT_EQ(154, parsed_pd0.btrange_max_amp[2]);
  EXPECT_EQ(155, parsed_pd0.btrange_max_amp[3]);

  // /////////////////////////////////////////////////////////////////////// //
  // navp
  EXPECT_TRUE(parsed_pd0.navp_valid);

  EXPECT_FLOAT_EQ(0.012708333, parsed_pd0.navp_time_to_bottom[0]);
  EXPECT_FLOAT_EQ(0.013333334, parsed_pd0.navp_time_to_bottom[1]);
  EXPECT_FLOAT_EQ(0.014791667, parsed_pd0.navp_time_to_bottom[2]);
  EXPECT_FLOAT_EQ(0.011614583, parsed_pd0.navp_time_to_bottom[3]);

  EXPECT_FLOAT_EQ(0.0080000004, parsed_pd0.navp_bottomtrack_stddev[0]);
  EXPECT_FLOAT_EQ(0.0080000004, parsed_pd0.navp_bottomtrack_stddev[1]);
  EXPECT_FLOAT_EQ(0.0080000004, parsed_pd0.navp_bottomtrack_stddev[2]);
  EXPECT_FLOAT_EQ(0.0080000004, parsed_pd0.navp_bottomtrack_stddev[3]);

  EXPECT_FLOAT_EQ(0.068255998, parsed_pd0.navp_bottomtrack_valid_time[0]);
  EXPECT_FLOAT_EQ(0.067630999, parsed_pd0.navp_bottomtrack_valid_time[1]);
  EXPECT_FLOAT_EQ(0.066171996, parsed_pd0.navp_bottomtrack_valid_time[2]);
  EXPECT_FLOAT_EQ(0.069348998, parsed_pd0.navp_bottomtrack_valid_time[3]);

  EXPECT_FALSE(parsed_pd0.navp_bottomtrack_shallow_mode);

  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_time_to_watermass[0]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_time_to_watermass[1]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_time_to_watermass[2]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_time_to_watermass[3]);

  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_stddev[0]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_stddev[1]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_stddev[2]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_stddev[3]);

  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_valid_time[0]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_valid_time[1]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_valid_time[2]);
  EXPECT_FLOAT_EQ(0, parsed_pd0.navp_watertrack_valid_time[3]);

  EXPECT_FLOAT_EQ(0.010416667, parsed_pd0.navp_watertrack_range);
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  //ros::init(argc, argv, "test_rdidvlparse");
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  //ros::shutdown();
  return ret;
}
