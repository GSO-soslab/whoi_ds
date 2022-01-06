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
// Created by zac on 1/22/18.
//

#include <ds_nmea_parsers/util.h>
#include "ds_sensors/xeosgps.h"
#include "xeosgps_private.h"
#include <iomanip>

#include "ds_nmea_parsers/Vtg.h"
#include "ds_nmea_parsers/Gga.h"

namespace ds_sensors
{
XeosGps::XeosGps() : SensorBase(), d_ptr_(std::unique_ptr<XeosGpsPrivate>(new XeosGpsPrivate))
{
}
XeosGps::XeosGps(int argc, char** argv, const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<XeosGpsPrivate>(new XeosGpsPrivate))
{
}

XeosGps::~XeosGps() = default;

void XeosGps::setupPublishers()
{
  DsProcess::setupPublishers();
  DS_D(XeosGps);
  auto nh = nodeHandle();
  d->nmea_gga_pub_ = nh.advertise<ds_nmea_msgs::Gga>(ros::this_node::getName() + "/nmea_gga", 10, false);
  d->nmea_vtg_pub_ = nh.advertise<ds_nmea_msgs::Vtg>(ros::this_node::getName() + "/nmea_vtg", 10, false);
}

void XeosGps::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto stream = std::stringstream(std::string{ std::begin(bytes.data), std::end(bytes.data) });
  auto nmea_msg = std::string{};

  DS_D(XeosGps);
  // Loop over lines
  while (std::getline(stream, nmea_msg))
  {
    // Find the start of a nmea message, trim leading characters if needed.
    auto begin_it = nmea_msg.find('$');
    if (begin_it == std::string::npos)
    {
      continue;
    }

    if (begin_it != 0)
    {
      nmea_msg = nmea_msg.substr(begin_it);
    }
    uint8_t calc_checksum = ds_nmea_msgs::calculate_checksum(nmea_msg);

    // Loop over messages types of interest.
    if (nmea_msg.find("GGA") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::Gga{};
      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $--GGA from string: " << nmea_msg);
        continue;
      }
      if (msg.checksum != calc_checksum)
      {
        ROS_ERROR_STREAM("Checksum mismatch on NMEA String \"" << nmea_msg << "\" (calc=" << std::hex << calc_checksum
                                                               << std::fixed << ")");
        continue;
      }

      d->nmea_gga_pub_.publish(msg);
      updateTimestamp("nmea_gga");
    }
    else if (nmea_msg.find("VTG") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::Vtg{};
      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $--VTG from string: " << nmea_msg);
        continue;
      }
      if (msg.checksum != calc_checksum)
      {
        ROS_ERROR_STREAM("Checksum mismatch on NMEA String \"" << nmea_msg << "\" (calc=" << std::hex << calc_checksum
                                                               << std::fixed << ")");
        continue;
      }

      d->nmea_vtg_pub_.publish(msg);
      updateTimestamp("nmea_vtg");
    }
    else
    {
      ROS_WARN_STREAM("Ignoring message: " << nmea_msg);
    }
  }
}
}
