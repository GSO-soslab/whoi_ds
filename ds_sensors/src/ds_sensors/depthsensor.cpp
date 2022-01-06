/**
* Copyright 2019 Woods Hole Oceanographic Institution
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
// Created by ivaughn on 8/6/19.
//


#include "ds_sensors/depthsensor.h"
#include "ds_util/fofonoff_depth.h"

#include "depthsensor_private.h"
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <ds_core_msgs/VoidCmd.h>
#include "ds_util/ds_util.h"

namespace ds_sensors
{
DepthSensor::DepthSensor()
  : ds_base::SensorBase(), d_ptr_(std::unique_ptr<DepthSensorPrivate>(new DepthSensorPrivate))
{
}

DepthSensor::DepthSensor(int argc, char* argv[], const std::string& name)
  : ds_base::SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<DepthSensorPrivate>(new DepthSensorPrivate))
{
}

DepthSensor::~DepthSensor() = default;

DepthSensor::PressureUnit DepthSensor::pressureUnit() const noexcept
{
  const DS_D(DepthSensor);
  return d->pressure_unit_;
}
void DepthSensor::setPressureUnit(DepthSensor::PressureUnit unit)
{
  DS_D(DepthSensor);
  d->pressure_unit_ = unit;
}
double DepthSensor::getTare() const noexcept
{
  const DS_D(DepthSensor);
  return d->tare_;
}
void DepthSensor::setLatitude(double latitude) noexcept
{
  DS_D(DepthSensor);
  d->latitude_ = latitude;
}
double DepthSensor::latitude() const noexcept
{
  const DS_D(DepthSensor);
  return d->latitude_;
}

void DepthSensor::setupPublishers()
{
  DsProcess::setupPublishers();
  DS_D(DepthSensor);

  d->depth_pressure_pub_ =
      nodeHandle().advertise<ds_sensor_msgs::DepthPressure>(ros::this_node::getName() + "/depth", 10);
}

void DepthSensor::setupServices()
{
  SensorBase::setupServices();
  DS_D(DepthSensor);

  d->depth_set_depth_zero_srv_ =
      nodeHandle().advertiseService<ds_core_msgs::VoidCmd::Request, ds_core_msgs::VoidCmd::Response>(
          ros::this_node::getName() + "/zero_depth", boost::bind(&DepthSensor::set_depth_zero, this, _1, _2));
}

void DepthSensor::setupSubscriptions()
{
  SensorBase::setupSubscriptions();
  DS_D(DepthSensor);

  if (!d->latitude_topic_.empty())
  {
    d->latitude_sub_ = nodeHandle().subscribe(d->latitude_topic_, 10, &DepthSensor::_latitude_callback, this);
  }
}

void DepthSensor::_latitude_callback(const sensor_msgs::NavSatFixPtr& latlon)
{
  DS_D(DepthSensor);

  if (std::isfinite(latlon->latitude))
    d->latitude_ = latlon->latitude;
}

bool DepthSensor::set_depth_zero(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp)
{
  DS_D(DepthSensor);
  if (!d->last_msg_time_.isValid() || d->last_msg_time_.isZero())
  {
    ROS_ERROR_STREAM("No valid depth pressure data since startup");
    return false;
  }

  ros::Duration dt = ros::Time::now() - d->last_msg_time_;
  if (dt < timeout())
  {
    ROS_WARN_STREAM("Resetting depth TARE to " << d->last_pressure_dbar_ <<" dBar");
    d->tare_ = d->last_pressure_dbar_;
    d->save_tare(d->tare_file, d->tare_);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("No valid depth pressure data since " << d->last_msg_time_);
    return false;
  }
}

void DepthSensor::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok = false;
  auto msg = ds_sensor_msgs::DepthPressure{};

  std::tie(ok, msg) = parse_message(bytes);

  if (!ok)
  {
    return;
  }
  DS_D(DepthSensor);

  FILL_SENSOR_HDR_IOTIME(msg, bytes.ds_header.io_time);

  // Fill out the rest of the fields and calculate depth
  msg.pressure_raw_unit = static_cast<uint8_t>(d->pressure_unit_);
  msg.tare = d->tare_;
  msg.latitude = d->latitude_;

  auto to_decibar = 0.0;
  switch (d->pressure_unit_)
  {
    case PressureUnit::Psi:
      to_decibar = 0.6894757;
      break;
    case PressureUnit::Bar:
      to_decibar = 10;
      break;
    case PressureUnit::hPa:
      to_decibar = 0.01;
      break;
    case PressureUnit::kPa:
      to_decibar = 0.1;
      break;
    case PressureUnit::MPa:
      to_decibar = 100.0;
      break;
    case PressureUnit::inHg:
      to_decibar = 0.338638;
      break;
    case PressureUnit::mmHg:
      to_decibar = 0.0133322;
      break;
    case PressureUnit::mH2O:
      to_decibar = 0.00980638;
      break;
  }

  double pressure_raw_dbar = msg.pressure_raw * to_decibar;
  if (msg.tare == ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA)
  {
    msg.pressure = pressure_raw_dbar;
    msg.tare = 0;
  }
  else
  {
    msg.pressure = pressure_raw_dbar - msg.tare;
  }

  // Use the default latitude defined in the DepthPressure message if there's no latitude set for the
  // instrument.
  if (msg.latitude == ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA)
  {
    msg.depth = ds_util::fofonoff_depth(msg.pressure, ds_sensor_msgs::DepthPressure::DEFAULT_LATITUDE);
    msg.latitude = ds_sensor_msgs::DepthPressure::DEFAULT_LATITUDE;
  }
  else
  {
    msg.depth = ds_util::fofonoff_depth(msg.pressure, msg.latitude);
  }

  //ROS_ERROR("USING HACK FOR SKIPPING MULTIPLE PARO MESSAGES.  REMOVE BEFORE USE");
  // Remvoe from here ---------
  if (!d->last_msg_time_.isValid())
  {
    d->last_msg_time_ = msg.header.stamp;
    return;
  }

  if ((msg.header.stamp - d->last_msg_time_) < ros::Duration(0.05))
  {
    //ROS_INFO("Actually using hack to skip multiple depth messages...");
    return;
  }
  // to here ------------

  // we need to store these so we can reset
  d->last_pressure_dbar_ = pressure_raw_dbar;
  d->last_msg_time_ = msg.header.stamp;

  // Apply the median filter here
  push_depth_buffer(msg.depth);
  msg.depth_buffer = d->depth_buffer_;
  double tmp[d->median_length_];
  for (int i = 0; i < d->median_length_; ++i)
    tmp[i] = d->depth_buffer_[i];
  d->median_depth_ = ds_util::median(tmp, d->median_length_);
  msg.median_depth = d->median_depth_;
  msg.median_tol = d->median_tol_;

  // Criteria for assignment of depth to output
  if (fabs(msg.depth - d->median_depth_) < d->median_tol_)
  {
    msg.median_depth_filter_ok = true;
  }
  else
  {
    msg.median_depth_filter_ok = false;
  }

  d->depth_pressure_pub_.publish(msg);
  updateTimestamp("depth", msg.header.stamp);
}

void DepthSensor::push_depth_buffer(double depth)
{
  DS_D(DepthSensor);

  for (int i = d->median_length_ - 1; i > 0; i--)
  {
    d->depth_buffer_[i] = d->depth_buffer_[i - 1];
  }
  d->depth_buffer_[0] = depth;
}

void DepthSensor::setPressureUnit(const std::string& pressure_unit)
{
  const auto unit = boost::algorithm::to_lower_copy(pressure_unit);

  if (unit == "psi")
  {
    setPressureUnit(PressureUnit::Psi);
  }
  else if (unit == "bar")
  {
    setPressureUnit(PressureUnit::Bar);
  }
  else if (unit == "hpa")
  {
    setPressureUnit(PressureUnit::hPa);
  }
  else if (unit == "kpa")
  {
    setPressureUnit(PressureUnit::kPa);
  }
  else if (unit == "mpa")
  {
    setPressureUnit(PressureUnit::MPa);
  }
  else if (unit == "inhg")
  {
    setPressureUnit(PressureUnit::inHg);
  }
  else if (unit == "mmhg")
  {
    setPressureUnit(PressureUnit::mmHg);
  }
  else if (unit == "mh2o")
  {
    setPressureUnit(PressureUnit::mH2O);
  } 
  else 
  {
    ROS_ERROR_STREAM("Unknown pressure unit: " << pressure_unit);
  }
}

void DepthSensor::setupParameters()
{
  SensorBase::setupParameters();
  auto depth_unit = ros::param::param<std::string>("~depth_unit", "psi");
  setPressureUnit(depth_unit);

  DS_D(DepthSensor);
  if (ros::param::has("~latitude_deg"))
  {
    ros::param::get("~latitude_deg", d->latitude_);
  }
  d->latitude_topic_ = ros::param::param<std::string>("~latitude_topic", "");

  d->median_length_ = ros::param::param<int>("~median_filter_length", 10);
  d->median_tol_ = ros::param::param<double>("~median_filter_tolerance", 2.0);
  for (int i = 0; i < d->median_length_; i++)
    d->depth_buffer_.resize(d->median_length_);
  d->median_depth_ = 0;
  d->median_depth_valid_ = false;

  if (ros::param::has("~tare_file_path")) {
    std::string tmp;
    ros::param::get("~tare_file_path", tmp);
    // boost filesystem is too ridiculous DUMB to handle ~ at the start of file names, because why
    // would a cross-platform library every support that?
    if (tmp[0] == '~') {
        const char* home = getenv("HOME");
        if (home == NULL) {
          ROS_ERROR_STREAM("HOME directory variable not set (this shouldn't happen!)");
        } else {
          tmp = std::string(home) + tmp.substr(1, std::string::npos);
          d->tare_file = boost::filesystem::path(tmp);
        }
    } else {
      d->tare_file = boost::filesystem::path(tmp);
    }
  }
  d->load_tare(d->tare_, d->tare_file);

}

}  // namespace ds_sensors
