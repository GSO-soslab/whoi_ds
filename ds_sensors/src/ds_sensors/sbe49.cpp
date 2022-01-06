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
// Created by zac on 12/5/17.
//

#include "ds_sensors/sbe49.h"
#include "sbe49_private.h"

#include <sstream>

namespace ds_sensors
{
Sbe49::Sbe49() : SensorBase(), d_ptr_(std::unique_ptr<Sbe49Private>(new Sbe49Private))
{
}

Sbe49::Sbe49(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<Sbe49Private>(new Sbe49Private))
{
}

Sbe49::~Sbe49() = default;

void Sbe49::setOutputFormat(Sbe49::OutputFormat format) noexcept
{
  DS_D(Sbe49);
  d->format_ = format;
}

Sbe49::OutputFormat Sbe49::outputFormat() const noexcept
{
  const DS_D(Sbe49);
  return d->format_;
}

std::pair<bool, ds_sensor_msgs::Ctd> Sbe49::parse_bytes(const ds_core_msgs::RawData& bytes, OutputFormat format)
{
  // Only Format 3 (engineering units in decimal) is supported at the moment
  if (format != OutputFormat::EngineeringDecimal)
  {
    ROS_WARN("Unable to parse output format %d", static_cast<int>(format));
    return { false, {} };
  }

  auto msg = ds_sensor_msgs::Ctd{};

  // From 'parse_bytes' in rov::seabird.cpp.  Does not guard against some types of malformed strings (like having
  // 'extra' numbers because we missed a line terminator
  const auto n_parsed = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "%lf, %lf, %lf, %lf, %lf",
                               &msg.temperature, &msg.conductivity, &msg.pressure, &msg.salinity, &msg.sound_speed);

  if (n_parsed < 3)
  {
    ROS_DEBUG("Expected at least 3 values, found %d", n_parsed);
    return { false, msg };
  }

  if (n_parsed == 4)
  {
    ROS_DEBUG("Parsed 4 values -- should parse 3 or 5");
    return { false, msg };
  }

  if (n_parsed == 3)
  {
    msg.salinity = ds_sensor_msgs::Ctd::CTD_NO_DATA;
    msg.sound_speed = ds_sensor_msgs::Ctd::CTD_NO_DATA;
  }

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = msg.ds_header.io_time;
  return { true, msg };
}

void Sbe49::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(Sbe49);
  d->ctd_pub_ = nodeHandle().advertise<ds_sensor_msgs::Ctd>(ros::this_node::getName() + "/ctd", 10);
}

void Sbe49::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_sensor_msgs::Ctd{};
  DS_D(Sbe49);

  std::tie(ok, msg) = parse_bytes(bytes, d->format_);

  if (!ok)
  {
    return;
  }

  FILL_SENSOR_HDR_IOTIME(msg, bytes.ds_header.io_time);
  d->ctd_pub_.publish(msg);
  updateTimestamp("ctd", msg.header.stamp);
}

}  // namespace ds_sensors
