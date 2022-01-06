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
// Created by jvaccaro on 9/28/18.
//

#include "ds_sensors/sbe03.h"
#include "sbe03_private.h"

namespace ds_sensors
{
Sbe03::Sbe03() : SensorBase(), d_ptr_(std::unique_ptr<Sbe03Private>(new Sbe03Private))
{
}

Sbe03::Sbe03(int argc, char* argv[], const std::string& name)
    : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<Sbe03Private>(new Sbe03Private))
{
}

Sbe03::~Sbe03() = default;

//std::pair<bool, ds_sensor_msgs::Frequency> Sbe03::parse_bytes(const ds_core_msgs::RawData& bytes)
//{
//
//  auto msg = ds_sensor_msgs::Frequency{};
//
//  // From 'parse_bytes' in rov::seabird.cpp.  Does not guard against some types of malformed strings (like having
//  // 'extra' numbers because we missed a line terminator
//  const auto n_parsed = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "%lf\r\n",
//                               &msg.hz);
//
//  if (n_parsed < 1)
//  {
//    ROS_DEBUG("Expected at least 1 values, found %d", n_parsed);
//    return { false, msg };
//  }
//
//  msg.ds_header = bytes.ds_header;
//  msg.header.stamp = msg.ds_header.io_time;
//  return { true, msg };
//}

std::pair<bool, ds_sensor_msgs::Frequency> Sbe03::parse_bytes(const ds_core_msgs::RawData& bytes)
{
  std::string byte_msg(reinterpret_cast<const char*>(bytes.data.data()), bytes.data.size());
  auto regex_ = boost::regex{ "([0-9]{4})\\.([0-9]{3})\r\n" };

  auto msg = ds_sensor_msgs::Frequency{};

  boost::smatch results;
  if (!boost::regex_search(byte_msg, results, regex_))
  {
    ROS_INFO_STREAM("Unable to parse SBE03 string");
    ROS_INFO_STREAM(byte_msg);
    return { false, msg };
  }

  unsigned long raw_ones = std::strtoul(results[1].str().c_str(), nullptr, 10);
  unsigned long raw_dec = std::strtoul(results[2].str().c_str(), nullptr, 10);

  msg.hz = raw_ones + raw_dec / 1000.0;

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = msg.ds_header.io_time;
  return { true, msg };
}

void Sbe03::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(Sbe03);
  d->freq_pub_ = nodeHandle().advertise<ds_sensor_msgs::Frequency>(ros::this_node::getName() + "/freq", 10);
}

void Sbe03::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_sensor_msgs::Frequency{};
  DS_D(Sbe03);

  std::tie(ok, msg) = parse_bytes(bytes);

  if (!ok)
  {
    return;
  }

  FILL_SENSOR_HDR_IOTIME(msg, bytes.ds_header.io_time);
  d->freq_pub_.publish(msg);
  updateTimestamp("freq", msg.header.stamp);
}

}  // namespace ds_sensors