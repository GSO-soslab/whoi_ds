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

#ifndef DS_SENSORS_DEPTHSENSOR_PRIVATE_H
#define DS_SENSORS_DEPTHSENSOR_PRIVATE_H

#include "ds_sensors/depthsensor.h"
#include "ds_sensor_msgs/DepthPressure.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>

namespace ds_sensors
{
struct DepthSensorPrivate
{
  DepthSensorPrivate()
  {
  }

  DepthSensor::PressureUnit pressure_unit_ = DepthSensor::PressureUnit::Psi;  //!< Reporting pressure unit
  uint16_t series_;
  double tare_ = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;      //!< Tare value applied to measurements
  double latitude_ = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;  //!< Current latitude for depth calculation
  std::string latitude_topic_;

  ros::Publisher depth_pressure_pub_;
  ros::Subscriber latitude_sub_;
  ros::ServiceServer depth_set_depth_zero_srv_;
  ros::Time last_msg_time_;
  double last_pressure_dbar_;

  int median_length_;
  std::vector<double> depth_buffer_;
  double median_depth_;
  bool median_depth_valid_;
  double median_tol_;

  boost::filesystem::path tare_file;

  static void load_tare(double& tare, const boost::filesystem::path& tare_file) {
    if (tare_file.empty() || !boost::filesystem::exists(tare_file)) {
      ROS_INFO_STREAM("Depth sensor has no tare file at " <<tare_file <<", not loading!");
      return;
    }

    std::ifstream ifile(tare_file.c_str());
    ifile >> tare;
    ifile.close();
    ROS_INFO_STREAM("Read TARE value of " <<tare);
  }
  static void save_tare(const boost::filesystem::path& tare_file, double tare) {
    if (tare_file.empty() || !boost::filesystem::exists(tare_file.parent_path())) {
      ROS_INFO_STREAM("Depth sensor has no tare path defined or location invalid: "
      <<tare_file <<", not writing!");
      return;
    }

    ROS_INFO_STREAM("Writing TARE value of " <<tare <<"to " <<tare_file);
    std::ofstream ofile(tare_file.c_str(), std::ios::out | std::ios::trunc);
    ofile <<std::fixed <<std::setprecision(15) <<tare <<std::endl;
    ofile.close();
  }
};
}
#endif //DS_SENSORS_DEPTHSENSOR_PRIVATE_H
