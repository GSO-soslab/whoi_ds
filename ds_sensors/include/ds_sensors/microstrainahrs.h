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
// Created by ivaughn on 11/4/19.
//

#ifndef DS_SENSORS_MICROSTRAINAHRS_H
#define DS_SENSORS_MICROSTRAINAHRS_H

#include <ds_base/sensor_base.h>
#include <ds_sensor_msgs/MemsImu.h>
#include <sensor_msgs/Imu.h>
#include <ds_sensor_msgs/Gyro.h>

#include <boost/optional/optional.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>

namespace ds_sensors {

struct MicrostrainAhrsPrivate;

class MicrostrainAhrs : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(MicrostrainAhrs);

 public:
  explicit MicrostrainAhrs();
  MicrostrainAhrs(int argc, char* argv[], const std::string& name);
  ~MicrostrainAhrs() override;
  DS_DISABLE_COPY(MicrostrainAhrs);

 protected:
  void setupParameters() override;
  void setupPublishers() override;
  void setupTimers() override;
  void setup() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

  static boost::optional<ds_sensor_msgs::MemsImu> parseMipPacket(const ds_core_msgs::RawData& bytes);
  static ds_sensor_msgs::MemsImu parseImuData(const uint8_t* buf, size_t buflen, const ros::Time& iot);
  static ds_sensor_msgs::Gyro convertToGyro(const ds_sensor_msgs::MemsImu& fullimu);
  static sensor_msgs::Imu convertToImu(const ds_sensor_msgs::MemsImu& fullimu);
  void timeGenStartCallback(const ros::WallTimerEvent& evt);
  void timeGenCallback(const ros::WallTimerEvent& evt);
  void sendTimeMsg(const ros::WallTime& now);
  void sendConfigCmd();

 private:
  std::unique_ptr<MicrostrainAhrsPrivate> d_ptr_;
};

} // namespace ds_sensors

#endif //DS_SENSORS_MICROSTRAINAHRS_H
