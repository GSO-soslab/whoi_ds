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
// Created by Stefano Suman on 12/2/20.
//

#include "ds_sensors/sonardyne_usbl.h"

#include <sstream>

namespace ds_sensors
{
  SonardyneUsbl::SonardyneUsbl() : SensorBase()
  {
  }

  SonardyneUsbl::SonardyneUsbl(int argc, char* argv[], const std::string& name)
    : SensorBase(argc, argv, name)
  {
  }

  SonardyneUsbl::~SonardyneUsbl() = default;

  void SonardyneUsbl::setupPublishers()
  {
    SensorBase::setupPublishers();

    navsat_pub_ = nodeHandle().advertise<sensor_msgs::NavSatFix>(ros::this_node::getName() + "/navsat", 10);
  }

  std::pair<bool, sensor_msgs::NavSatFix> SonardyneUsbl::parse_bytes(const ds_core_msgs::RawData& bytes)
  {

    auto msg = sensor_msgs::NavSatFix{};

    // $PSONLLD,170004.53,ALVIN 914,A,28.8114807,-87.80925696,1951.156,13.96,13.96,0.00,2.62,,,,*6F
    //          ToV       Beacon      lat        lon          depth    err_ellipse      dep_error
    int hr;
    int min;
    int sec;
    int hsec;
    char bname[1024];
    int bid;
    char valid;
    double lat;
    double lon;
    double depth;
    double err_ellipse_semimajor_1sd;
    double err_ellipse_semiminor_1sd;
    double err_ellipse_direction;
    double depth_error_1sd;
    
    const auto n_parsed = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "$PSONLLD,%2d%2d%2d.%2d,%s %d,%c,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
				 &hr,
				 &min,
				 &sec,
				 &hsec,
				 bname,
				 &bid,
				 &valid,
				 &lat,
				 &lon,
				 &depth,
				 &err_ellipse_semimajor_1sd,
				 &err_ellipse_semiminor_1sd,
				 &err_ellipse_direction,
				 &depth_error_1sd);

    if (n_parsed < 14)
      {
	ROS_WARN_STREAM("Expected at least 14 values, found " << n_parsed);
	msg.header.stamp = bytes.ds_header.io_time;
	return { false, msg };
      }
    else
      {
	ROS_WARN_STREAM("Parse OK");
	msg.latitude = lat;
	msg.longitude = lon;
	msg.altitude = -depth;
	// In practice for usbl fixes err_ellipse_semimajor = err_ellipse_semiminor, this simplifies the covariance matrix calculation
	msg.position_covariance[0] = err_ellipse_semimajor_1sd * err_ellipse_semimajor_1sd;
	msg.position_covariance[4] = err_ellipse_semimajor_1sd * err_ellipse_semimajor_1sd;
	msg.position_covariance[8] = depth_error_1sd * depth_error_1sd;
        msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
	msg.status.status = msg.status.STATUS_FIX;
	msg.status.service = msg.status.SERVICE_GPS;

	// Here we should take the ToV fix time, convert it to boost posix time, and then make it the timestamp sent in the message
	//msg.header.stamp = ros::Time::fromBoost(const boost::posix_time::ptime &t);
	msg.header.stamp = ros::Time::now();
      }

    return { true, msg };
  }

  void SonardyneUsbl::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
  {
    auto ok = false;
    auto msg = sensor_msgs::NavSatFix{};

    ROS_WARN_STREAM("USBL data received: " << bytes.data.data());
    
    std::tie(ok, msg) = SonardyneUsbl::parse_bytes(bytes);

    if (!ok)
      {
	return;
      }

    navsat_pub_.publish(msg);

  }
}
