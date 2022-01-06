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
// Created by zac on 12/11/17.
//

#include "ds_sensors/resonsvp70.h"
#include "resonsvp70_private.h"

namespace ds_sensors
{
ResonSvp70::ResonSvp70() : SensorBase(), d_ptr_(std::unique_ptr<ResonSvp70Private>(new ResonSvp70Private))
{
}

ResonSvp70::ResonSvp70(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<ResonSvp70Private>(new ResonSvp70Private))
{
}

ResonSvp70::~ResonSvp70() = default;

void ResonSvp70::setOutputFormat(ResonSvp70::OutputFormat format) noexcept
{
  DS_D(ResonSvp70);
  d->format_ = format;
}

ResonSvp70::OutputFormat ResonSvp70::outputFormat() const noexcept
{
  const DS_D(ResonSvp70);
  return d->format_;
}

std::pair<bool, ds_sensor_msgs::SoundSpeed> ResonSvp70::parse_bytes(const ds_core_msgs::RawData& bytes,
                                                                    double clamping_min, double clamping_max,
                                                                    OutputFormat format)
{
  // Only Format 3 (engineering units in decimal) is supported at the moment
  auto ok = false;
  auto msg = ds_sensor_msgs::SoundSpeed{};
  auto ptr = reinterpret_cast<const char*>(bytes.data.data());

  switch (format)
  {
    case OutputFormat::Aml:
    {
      const auto n = sscanf(ptr, "%lf", &msg.speed);
      ok = (n == 1);
      break;
    }
    case OutputFormat::Valeport:
    {
      // Valeport format has speed in mm/s
      const auto n = sscanf(ptr, "%lf", &msg.speed);
      if (n == 1)
      {
        ok = true;
        msg.speed /= 1000.0;
      }
      break;
    }
    case OutputFormat::Reson:
    {
      auto depth = 0;
      auto temp = 0;
      auto speed = 0;
      auto battery = 0;
      // Reson is in decimeters/s
      const auto n = sscanf(ptr, "%05d%05d%03d%02d%*04d", &speed, &depth, &temp, &battery);
      if (n == 4)
      {
        ok = true;
        msg.speed = static_cast<float>(speed / 10.0);
      }
      break;
    }
    case OutputFormat::Svp7x:
    {
      ROS_WARN("Svp7x output format is not yet implemented.");
      break;
    }
  }

  if (!ok)
  {
    ROS_WARN("Failed to parse output type %d message.", static_cast<int>(format));
    return { false, {} };
  }

  msg.clamping_status = ds_sensor_msgs::SoundSpeed::SV_STATUS_NOT_CLAMPED;
  msg.raw_speed = msg.speed;
  if (msg.speed < clamping_min)
  {
    msg.speed = clamping_min;
    msg.clamping_status = ds_sensor_msgs::SoundSpeed::SV_STATUS_CLAMPED;
  }
  if (msg.speed > clamping_max)
  {
    msg.speed = clamping_max;
    msg.clamping_status = ds_sensor_msgs::SoundSpeed::SV_STATUS_CLAMPED;
  }

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = msg.ds_header.io_time;
  return { true, msg };
}

void ResonSvp70::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(ResonSvp70);
  d->sound_speed_pub_ =
      nodeHandle().advertise<ds_sensor_msgs::SoundSpeed>(ros::this_node::getName() + "/sound_speed", 10);
};

void ResonSvp70::setupParameters()
{
  SensorBase::setupParameters();
  DS_D(ResonSvp70);
  speed_clamping_min_ = ros::param::param<double>("~speed_clamping_min", 1450.0);
  speed_clamping_max_ = ros::param::param<double>("~speed_clamping_max", 1575.0);
};

void ResonSvp70::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_sensor_msgs::SoundSpeed{};
  DS_D(ResonSvp70);

  std::tie(ok, msg) = ResonSvp70::parse_bytes(bytes, speed_clamping_min_, speed_clamping_max_, d->format_);

  if (!ok)
  {
    return;
  }

  FILL_SENSOR_HDR_IOTIME(msg, bytes.ds_header.io_time);
  d->sound_speed_pub_.publish(msg);
  updateTimestamp("sound_speed", msg.header.stamp);
}
}  // namespace ds_sensors
