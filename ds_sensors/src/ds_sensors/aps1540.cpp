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
// Created by zac on 12/8/17.
//

#include "ds_sensors/aps1540.h"
#include "aps1540_private.h"

#include <sstream>

namespace ds_sensors
{
Aps1540::Aps1540() : SensorBase(), d_ptr_(std::unique_ptr<Aps1540Private>(new Aps1540Private))
{
}

Aps1540::Aps1540(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<Aps1540Private>(new Aps1540Private))
{
}

Aps1540::~Aps1540() = default;

std::pair<bool, ds_sensor_msgs::VectorMagneticField> Aps1540::parse_bytes(const ds_core_msgs::RawData& bytes)
{
  auto msg = ds_sensor_msgs::VectorMagneticField{};

  // From 'parse_aps_maggie' in rov::maggie_functions.cpp.  Does not guard against some types of malformed strings
  // (like having 'extra' numbers because we missed a line terminator)
  //
  // Message units for fields are nT, temperature is C
  // APS provides measurements in nT and C

  const auto n_parsed = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "%lf %lf %lf %lf", &msg.x, &msg.y,
                               &msg.z, &msg.temperature);

  if (n_parsed < 4)
  {
    ROS_DEBUG("Expected at least 4 values, found %d", n_parsed);
    return { false, msg };
  }

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = msg.ds_header.io_time;
  return { true, msg };
}

void Aps1540::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(Aps1540);
  d->vector_mag_field_pub_ = nodeHandle().advertise<ds_sensor_msgs::VectorMagneticField>(
      ros::this_node::getName() + "/vector_magnetic_field", 10);
}

void Aps1540::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_sensor_msgs::VectorMagneticField{};

  std::tie(ok, msg) = Aps1540::parse_bytes(bytes);

  if (!ok)
  {
    return;
  }

  FILL_SENSOR_HDR_IOTIME(msg, msg.ds_header.io_time);

  DS_D(Aps1540);
  d->vector_mag_field_pub_.publish(msg);
  updateTimestamp("vector_magnetic_field", msg.header.stamp);
}
}
