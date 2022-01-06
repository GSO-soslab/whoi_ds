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
// Created by zac on 12/9/17.
//

#include "ds_sensors/parodigiquartz.h"
#include "ds_util/fofonoff_depth.h"

#include "parodigiquartz_private.h"
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <ds_core_msgs/VoidCmd.h>
#include "ds_util/ds_util.h"

namespace ds_sensors
{
ParoDigiquartz::ParoDigiquartz()
  : ds_sensors::DepthSensor(), d_ptr_(std::unique_ptr<ParoDigiquartzPrivate>(new ParoDigiquartzPrivate))
{
}

ParoDigiquartz::ParoDigiquartz(int argc, char* argv[], const std::string& name)
  : ds_sensors::DepthSensor(argc, argv, name), d_ptr_(std::unique_ptr<ParoDigiquartzPrivate>(new ParoDigiquartzPrivate))
{
}

ParoDigiquartz::~ParoDigiquartz() = default;

std::pair<bool, ds_sensor_msgs::DepthPressure> ParoDigiquartz::parse_pressure(const ds_core_msgs::RawData& bytes)
{
  // Like most records, pressure messages look like *HHDDffff.ffff
  // where * indicates the start of the message
  //       HH is a 2-chracter host ID
  //       DD is a 2-character
  //       ffff.fff is some number of characters representing the pressure value

  // Paro 8CB 4000 series looks like *HHDDCC=ffff.ffff,tttt.tttt
  // where * indicates the start of the message
  //       HH is a 2-chracter host ID
  //       DD is a 2-character
  //       CC is a 2-character command
  //       ffff.ffff is some number of characters representing the pressure value
  //       tttt.tttt is some number of characters representing the temperature value

  if (bytes.data.size() < 6)
  {
    return { false, {} };
  }

  auto msg = ds_sensor_msgs::DepthPressure{};
  auto host = 0;
  auto id = 0;
  const auto n = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "*%*02d%*02d%lf", &msg.pressure_raw);

  if (n != 1)
  {
    // Try alternate parsing method for 8CB 4000
    const auto m = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "*%*02d%*02dP4=%lf", &msg.pressure_raw);
    if (m != 1)
      return { false, {} };
  }

  // Set some sensible defaults
  msg.depth = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
  msg.latitude = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
  msg.tare = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = msg.ds_header.io_time;
  return { true, msg };
}

void ParoDigiquartz::start(uint8_t id)
{
  auto ostream = std::ostringstream{};
  ostream << "*" << ds_util::int_to_hex(id) << "00P4\r\n";
  const auto msg = ostream.str();

  //  DS_D(ParoDigiquartz);
  ROS_INFO_STREAM("Sending '" << msg << "'");
  sendCommand(msg, "instrument");
}

void ParoDigiquartz::sendCommand(std::string command, std::string connection)
{
  if (command.empty())
  {
    return;
  }

  if (!boost::algorithm::starts_with(command, "*"))
  {
    command.insert(std::begin(command), '*');
  }

  DepthSensor::sendCommand(command, connection);
}

void ParoDigiquartz::setupParameters()
{
  DepthSensor::setupParameters();

  DS_D(ParoDigiquartz);

  d->send_startup_command_ = ros::param::param<bool>("~send_startup_command", false);
}

void ParoDigiquartz::setup()
{
  DepthSensor::setup();
  DS_D(ParoDigiquartz);
  if (d->send_startup_command_)
  {
    start();
  }
}

std::pair<bool, ds_sensor_msgs::DepthPressure> ParoDigiquartz::parse_message(const ds_core_msgs::RawData& bytes) {
  return parse_pressure(bytes);
}


}  // namespace ds_sensors
