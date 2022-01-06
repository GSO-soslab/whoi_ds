//
// Created by ivaughn on 8/23/19.
//

#ifndef DS_SENSORS_PHINSBIN_H
#define DS_SENSORS_PHINSBIN_H

#include <ds_base/sensor_base.h>
#include <ds_sensor_msgs/PhinsStdbin3.h>
#include <sensor_msgs/Imu.h>
#include <ds_sensor_msgs/Gyro.h>
#include <ds_sensor_msgs/Compass.h>
#include <ds_sensor_msgs/PhinsStatus.h>

namespace ds_sensors {

struct PhinsBinPrivate;

class PhinsBin : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(PhinsBin);

 public:
  explicit PhinsBin();
  PhinsBin(int argc, char* argv[], const std::string& name);
  ~PhinsBin() override;
  DS_DISABLE_COPY(PhinsBin);

  static std::pair<bool, ds_sensor_msgs::PhinsStdbin3> parseStdbin3Msg(const ds_core_msgs::RawData& bytes);
  static ds_sensor_msgs::Gyro makeGyroMsg(const ds_sensor_msgs::PhinsStdbin3& msg);
  static ds_sensor_msgs::Compass makeCompassMsg(const ds_sensor_msgs::PhinsStdbin3& msg);
  static sensor_msgs::Imu makeImuMsg(const ds_sensor_msgs::PhinsStdbin3& msg);
  static ds_sensor_msgs::PhinsStatus makePhinsStatusMsg(const ds_sensor_msgs::PhinsStdbin3& msg);

 protected:
  void setupParameters() override;
  void setupPublishers() override;
  void checkProcessStatus(const ros::TimerEvent& event) override;

  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

 private: std::unique_ptr<PhinsBinPrivate> d_ptr_;
};

}

#endif //DS_SENSORS_PHINSBIN_H
