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
#include "remushtpgl_private.h"

#include <sstream>

namespace ds_sensors
{
const double MILLIBAR_TO_PSI = 0.0145037738;

RemusHtpgl::RemusHtpgl() : SensorBase(), d_ptr_(std::unique_ptr<RemusHtpglPrivate>(new RemusHtpglPrivate))
{
}

RemusHtpgl::RemusHtpgl(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<RemusHtpglPrivate>(new RemusHtpglPrivate))
{
}

RemusHtpgl::~RemusHtpgl()
{
  DS_D(RemusHtpgl);
  d->htp_pub_.shutdown();
  d->htpgl_pub_.shutdown();
}

std::pair<bool, ds_hotel_msgs::HTPGL> RemusHtpgl::parse_bytes(const ds_core_msgs::RawData& bytes)
{
  auto msg = ds_hotel_msgs::HTPGL{};

  // dummy fields, included mostly for documentation
  int probe_fail = 0;
  int leak = 0;

  const auto n_parsed = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "%lf,%lf,%lf,%f,%d,%d",
      &msg.htp.pressure, &msg.htp.temperature, &msg.htp.humidity,
      &msg.ground_fault, &probe_fail, &leak);

  if (n_parsed < 6)
  {
    ROS_ERROR("Expected at least 6 values from REMUS HTPGL board, got %d", n_parsed);
    return { false, msg };
  }

  // convert to bool
  msg.probe_fail = (probe_fail != 0);
  msg.leak = (leak != 0);

  msg.ds_header = bytes.ds_header;
  msg.header.stamp = bytes.ds_header.io_time;

  // we're going to send the htp, so be sure it has its own header
  msg.htp.header = msg.header;
  msg.htp.ds_header = msg.ds_header;

  msg.htp.pressure *= MILLIBAR_TO_PSI;

  return { true, msg };
}

void RemusHtpgl::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(RemusHtpgl);
  d->htp_pub_ = nodeHandle().advertise<ds_hotel_msgs::HTP>(ros::this_node::getName() + "/htp", 10);
  d->htpgl_pub_ = nodeHandle().advertise<ds_hotel_msgs::HTPGL>(ros::this_node::getName() + "/htpgl", 10);
}

void RemusHtpgl::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_hotel_msgs::HTPGL{};

  std::tie(ok, msg) = RemusHtpgl::parse_bytes(bytes);

  if (!ok)
  {
    return;
  }

  FILL_SENSOR_HDR_IOTIME(msg, msg.ds_header.io_time);

  DS_D(RemusHtpgl);
  d->htp_pub_.publish(msg.htp);
  d->htpgl_pub_.publish(msg);
  updateTimestamp("htp", msg.header.stamp);
  updateTimestamp("htpgl", msg.header.stamp);
}
}