//
// Created by ivaughn on 8/23/19.
//

#ifndef DS_SENSORS_PHINSBINPRIVATE_H
#define DS_SENSORS_PHINSBINPRIVATE_H

#include "ds_sensors/phinsbin.h"

#include <ds_sensor_msgs/PhinsStdbin3.h>
#include <sensor_msgs/Imu.h>
#include <ds_sensor_msgs/Gyro.h>
#include <ds_sensor_msgs/PhinsStatus.h>

namespace ds_sensors {

struct PhinsBinPrivate {

  ros::Publisher fullimu_pub_;
  ros::Publisher phinsbin_pub_;
  ros::Publisher phinsstatus_pub_;
  ros::Publisher gyro_pub_;
  ros::Publisher compass_pub_;
  ros::Publisher imu_pub_;

  // The downsample parameter lets us record full phinsbin messages more frequently
  // than teh rate we publish gyro/compass data
  int downsample;

  // part of the downsampling implementation; number of samples skipped so far
  int skipped;

  std::array<uint32_t, 2> sensor_status;
  std::array<uint32_t, 4> algo_status;
  std::array<uint32_t, 3> system_status;
  std::array<uint32_t, 1> user_status;

  void update_status(const ds_sensor_msgs::PhinsStdbin3& msg) {
    sensor_status[0] = msg.sensor_status[0];
    sensor_status[1] = msg.sensor_status[1];

    algo_status[0] = msg.ins_algo_status[0];
    algo_status[1] = msg.ins_algo_status[1];
    algo_status[2] = msg.ins_algo_status[2];
    algo_status[3] = msg.ins_algo_status[3];

    system_status[0] = msg.ins_system_status[0];
    system_status[1] = msg.ins_system_status[1];
    system_status[2] = msg.ins_system_status[2];

    user_status[0] = msg.ins_user_status;
  }

  // maximum clock offset before we use I/O time instead of the phins internal time
  double max_clock_offset;
};

}

#endif //DS_SENSORS_PHINSBINPRIVATE_H
