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

#include "ds_sensors/anderaaoxyoptode.h"
#include "anderaaoxyoptode_private.h"

namespace ds_sensors
{
AnderaaOxyOptode::AnderaaOxyOptode()
  : SensorBase(), d_ptr_(std::unique_ptr<AnderaaOxyOptodePrivate>(new AnderaaOxyOptodePrivate))
{
}

AnderaaOxyOptode::AnderaaOxyOptode(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<AnderaaOxyOptodePrivate>(new AnderaaOxyOptodePrivate))
{
}

AnderaaOxyOptode::~AnderaaOxyOptode() = default;

std::pair<bool, ds_sensor_msgs::OxygenConcentration> AnderaaOxyOptode::parse_bytes(const ds_core_msgs::RawData& bytes,
                                                                                   OutputFormat format)
{
  // Only Format 3 (engineering units in decimal) is supported at the moment
  if (format != OutputFormat::EngineeringDecimal)
  {
    ROS_WARN("Unable to parse output format %d", static_cast<int>(format));
    return { false, {} };
  }

  auto msg = ds_sensor_msgs::OxygenConcentration{};

  // The anderaa message is a mess in ascii:
  // MEASUREMENT 4330    20      O2Concentration(uM)     155.469 AirSaturation(%)        36.054  Temperature(Deg.C)
  // 2.024   CalPhase(Deg)   45.442  TCPhase(Deg)    44.868  C1RPh(Deg) 48.889  C2RPh(Deg)      4.021   C1Amp(mV)
  // 1041.7  C2Amp(mV)       808.9   RawTemp(mV)     691.1\r\n
  //
  // Split on white space and pull specific indexes
  auto str = std::string{ reinterpret_cast<const char*>(bytes.data.data()), bytes.data.size() };
  auto buf = std::istringstream{ str };
  const auto fields =
      std::vector<std::string>{ std::istream_iterator<std::string>(buf), std::istream_iterator<std::string>() };

  if (fields.size() < 9)
  {
    ROS_DEBUG("Need at least 9 fields for a valid message");
    return { false, {} };
  }

  auto ok = true;

  //
  // Our float parsing function.
  //
  auto parse_float = [&ok](const std::string& field) -> float {

    const auto c_str = field.c_str();
    // Our expected pointer position after parsing is at the very end of the field.
    const auto expected_end = c_str + field.size();
    auto end = static_cast<char*>(nullptr);

    auto result = std::strtof(c_str, &end);
    // Update our 'ok' parameter.  One failure fails everything.
    ok &= (end == expected_end);
    return result;
  };

  msg.concentration = parse_float(fields.at(4));
  msg.air_saturation = parse_float(fields.at(6));
  msg.temperature = parse_float(fields.at(8));

  if (!ok)
  {
    ROS_WARN_STREAM("Failed to parse " << str);
    return { false, {} };
  }

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = msg.ds_header.io_time;
  return { true, msg };
}

void AnderaaOxyOptode::setupPublishers()
{
  DsProcess::setupPublishers();
  DS_D(AnderaaOxyOptode);
  auto nh = nodeHandle();
  d->oxygen_concentration_pub_ =
      nh.advertise<ds_sensor_msgs::OxygenConcentration>(ros::this_node::getName() + "/oxygen_concentration", 10);
};

void AnderaaOxyOptode::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_sensor_msgs::OxygenConcentration{};

  std::tie(ok, msg) = AnderaaOxyOptode::parse_bytes(bytes);

  if (!ok)
  {
    return;
  }

  FILL_SENSOR_HDR_IOTIME(msg, bytes.ds_header.io_time);

  DS_D(AnderaaOxyOptode);
  d->oxygen_concentration_pub_.publish(msg);
  updateTimestamp("oxygen_concentration", msg.header.stamp);
}

}  // namespace ds_sensors
