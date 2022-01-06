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
// Created by SS on 1/9/2018.
//

#include "ds_sensors/phins.h"
#include "phins_private.h"

#include "ds_nmea_parsers/Hdt.h"
#include "ds_nmea_parsers/PixseAlgsts.h"
#include "ds_nmea_parsers/PixseAtitud.h"
#include "ds_nmea_parsers/PixseStatus.h"
#include "ds_nmea_parsers/PixseHtsts.h"
#include "ds_nmea_parsers/PixseStdhrp.h"

#include <sstream>

namespace ds_sensors
{
Phins::Phins() : SensorBase(), d_ptr_(std::unique_ptr<PhinsPrivate>(new PhinsPrivate))
{
}

Phins::Phins(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<PhinsPrivate>(new PhinsPrivate))
{
}

Phins::~Phins() = default;

void Phins::setupConnections()
{
  DS_D(Phins);
  auto phins_standard = addConnection("phins_standard", boost::bind(&Phins::parseReceivedPhinsStandard, this, _1));
  connections().insert(std::make_pair("phins_standard", phins_standard));
}

void Phins::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(Phins);
  auto nh = nodeHandle();
  d->gyro_pub_ = nh.advertise<ds_sensor_msgs::Gyro>(ros::this_node::getName() + "/gyro", 10);
  d->compass_pub_ = nh.advertise<ds_sensor_msgs::Compass>(ros::this_node::getName() + "/compass", 10);
  d->phins_status_pub_ = nh.advertise<ds_sensor_msgs::PhinsStatus>(ros::this_node::getName() + "/phins_status", 1);
}

void Phins::checkProcessStatus(const ros::TimerEvent& event)
{
  DS_D(Phins);
  auto status = statusMessage();

  // Status flag checking
  // Still aligning?
  if (d->algorithm_status_ &
      (AlgorithmStatus::ALG_OPERATION_MODE_ALIGNMENT | AlgorithmStatus::ALG_OPERATION_MODE_FINE_ALIGNMENT))
  {
    ROS_WARN("PHINS it is still aligning");
    status.status = ds_core_msgs::Status::STATUS_WARN;
  }

  // Sensor problems?
  if (d->system_status_ & SystemStatus::SYSTEM_HRP_DEGRADED)
  {
    ROS_WARN("PHINS reports degraded HPR values");
    status.status = ds_core_msgs::Status::STATUS_WARN;
  }

  if (d->user_status_ & UserStatus::USER_SYSTEM_WARNING)
  {
    ROS_WARN("PHINS SYSTEM_WARNING flag set");
    status.status = ds_core_msgs::Status::STATUS_WARN;
  }

  if (d->system_status_ & (SystemStatus::SYSTEM_SERIAL_INPUT_R_ERROR | SystemStatus::SYSTEM_SERIAL_INPUT_A_ERROR |
                           SystemStatus::SYSTEM_SERIAL_INPUT_B_ERROR | SystemStatus::SYSTEM_SERIAL_INPUT_C_ERROR |
                           SystemStatus::SYSTEM_SERIAL_INPUT_D_ERROR | SystemStatus::SYSTEM_SERIAL_INPUT_E_ERROR))
  {
    ROS_ERROR("PHINS SERIAL_INPUT ERROR flag is set");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->system_status_ & SystemStatus::SYSTEM_HRP_NOT_VALID)
  {
    ROS_ERROR("PHIN reporting invalid HRP");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->user_status_ & UserStatus::USER_SYSTEM_ERROR)
  {
    ROS_WARN("PHINS SYSTEM_ERROR flag set");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  publishStatus(status);
}

// The PHINS STANDARD message is a group of NMEA messages that arrive in
// a burst.  This burst will trigger individual calls to parseReceviedPhinsStandard
// when we're connectecd via serial (due to the multiple line terminations).  However,
// the message arrives as a single large string when connected via UDP.
//
// This callback deals with the asymetric reception issue with a little state machine:
//
// - Split incoming data into individual chunks separated by newlines
// - Attempt to parse a nmea message from a line
//   - If parsing succeeded:
//     - If we've already parsed a message of this type:
//        - clear the type reception flag
//     - Mark this type as received in the reception flag
//     - Insert data into various outgoing message types
void Phins::parseReceivedPhinsStandard(const ds_core_msgs::RawData& bytes)
{
  DS_D(Phins);
  auto ok = false;
  // First, cast the data to a string.
  auto stream = std::stringstream(std::string{ std::begin(bytes.data), std::end(bytes.data) });
  auto nmea_msg = std::string{};

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

    // Loop over messages types of interest.
    if (nmea_msg.find("ATITUD") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::PixseAtitud{};
      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $PIXSE,ATITUD from string: " << nmea_msg);
        continue;
      }

      if (d->phins_standard_reception_status_ & PhinsPrivate::PhinsStandardParts::ATITUD)
      {
        d->phins_standard_reception_status_ = 0;
      }
      d->phins_standard_reception_status_ |= PhinsPrivate::PhinsStandardParts::ATITUD;

      FILL_SENSOR_HDR_IOTIME(d->gyro_, bytes.ds_header.io_time);

      // SS - convert to radians
      d->gyro_.roll = msg.roll * M_PI / 180.0;
      d->gyro_.pitch = msg.pitch * M_PI / 180.0;

      // SS - compute quaternion from Phins native frame to E-N-U world frame
      Eigen::Quaterniond q(Eigen::AngleAxisd(-(d->gyro_.heading) + (M_PI / 2.0), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(d->gyro_.pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(d->gyro_.roll, Eigen::Vector3d::UnitX()));
      d->gyro_.orientation.x = q.x();
      d->gyro_.orientation.y = q.y();
      d->gyro_.orientation.z = q.z();
      d->gyro_.orientation.w = q.w();

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_GYRO_MASK) ==
          PhinsPrivate::PHINS_STANDARD_GYRO_MASK)
      {
        d->gyro_pub_.publish(d->gyro_);
      }
    }

    else if (nmea_msg.find("HEHDT") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::Hdt{};
      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $HEHDT from string: " << nmea_msg);
        continue;
      }

      if (d->phins_standard_reception_status_ & PhinsPrivate::PhinsStandardParts::HEHDT)
      {
        d->phins_standard_reception_status_ = 0;
      }

      d->phins_standard_reception_status_ |= PhinsPrivate::PhinsStandardParts::HEHDT;

      FILL_SENSOR_HDR_IOTIME(d->compass_, bytes.ds_header.io_time);

      // SS - convert to radians
      d->compass_.heading = msg.heading * M_PI / 180.0;
      d->compass_.is_true_heading = msg.is_true;

      // SS - convert to radians
      d->gyro_.heading = msg.heading * M_PI / 180.0;

      // SS - compute quaternion from Phins native frame to E-N-U world frame
      Eigen::Quaterniond q(Eigen::AngleAxisd(-(d->gyro_.heading) + (M_PI / 2.0), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(d->gyro_.pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(d->gyro_.roll, Eigen::Vector3d::UnitX()));
      d->gyro_.orientation.x = q.x();
      d->gyro_.orientation.y = q.y();
      d->gyro_.orientation.z = q.z();
      d->gyro_.orientation.w = q.w();

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_GYRO_MASK) ==
          PhinsPrivate::PHINS_STANDARD_GYRO_MASK)
      {
        d->gyro_pub_.publish(d->gyro_);
      }

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_COMPASS_MASK) ==
          PhinsPrivate::PHINS_STANDARD_COMPASS_MASK)
      {
        d->compass_pub_.publish(d->compass_);
      }
    }

    else if (nmea_msg.find("STDHRP") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::PixseStdhrp{};
      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $PIXSE,STDHRP from string: " << nmea_msg);
        continue;
      }

      if (d->phins_standard_reception_status_ & PhinsPrivate::PhinsStandardParts::STDHRP)
      {
        d->phins_standard_reception_status_ = 0;
      }

      d->phins_standard_reception_status_ |= PhinsPrivate::PhinsStandardParts::STDHRP;

      // SS - convert to radians
      d->compass_.heading_covar = msg.heading * M_PI / 180.0;
      d->gyro_.heading_covar = msg.heading * M_PI / 180.0;
      d->gyro_.pitch_covar = msg.pitch * M_PI / 180.0;
      d->gyro_.roll_covar = msg.roll * M_PI / 180.0;

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_GYRO_MASK) ==
          PhinsPrivate::PHINS_STANDARD_GYRO_MASK)
      {
        d->gyro_pub_.publish(d->gyro_);
      }

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_COMPASS_MASK) ==
          PhinsPrivate::PHINS_STANDARD_COMPASS_MASK)
      {
        d->compass_pub_.publish(d->compass_);
      }
    }

    else if (nmea_msg.find("ALGSTS") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::PixseAlgsts{};

      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $PIXSE,ALGSTS from string: " << nmea_msg);
        continue;
      }

      if (d->phins_standard_reception_status_ & PhinsPrivate::PhinsStandardParts::ALGSTS)
      {
        d->phins_standard_reception_status_ = 0;
      }

      d->phins_standard_reception_status_ |= PhinsPrivate::PhinsStandardParts::ALGSTS;

      d->algorithm_status_ = static_cast<Phins::AlgorithmStatus>(msg.status);

      FILL_SENSOR_HDR_IOTIME(d->phins_status_, bytes.ds_header.io_time);
      d->phins_status_.algorithm.status = msg.status;

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_STATUS_MASK) ==
          PhinsPrivate::PHINS_STANDARD_STATUS_MASK)
      {
        d->phins_status_pub_.publish(d->phins_status_);
      }
    }

    else if (nmea_msg.find("STATUS") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::PixseStatus{};

      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $PIXSE,STATUS from string: " << nmea_msg);
        continue;
      }

      if (d->phins_standard_reception_status_ & PhinsPrivate::PhinsStandardParts::STATUS)
      {
        d->phins_standard_reception_status_ = 0;
      }

      d->phins_standard_reception_status_ |= PhinsPrivate::PhinsStandardParts::STATUS;

      d->system_status_ = static_cast<Phins::SystemStatus>(msg.status);
      FILL_SENSOR_HDR_IOTIME(d->phins_status_, bytes.ds_header.io_time);
      d->phins_status_.system.status = msg.status;

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_STATUS_MASK) ==
          PhinsPrivate::PHINS_STANDARD_STATUS_MASK)
      {
        d->phins_status_pub_.publish(d->phins_status_);
      }
    }

    else if (nmea_msg.find("HT_STS") != std::string::npos)
    {
      auto msg = ds_nmea_msgs::PixseHtsts{};

      if (!ds_nmea_msgs::from_string(msg, nmea_msg))
      {
        ROS_WARN_STREAM("Unable to parse $PIXSE,HT_STS from string: " << nmea_msg);
        continue;
      }

      if (d->phins_standard_reception_status_ & PhinsPrivate::PhinsStandardParts::HT_STS)
      {
        d->phins_standard_reception_status_ = 0;
      }

      d->phins_standard_reception_status_ |= PhinsPrivate::PhinsStandardParts::HT_STS;

      d->user_status_ = static_cast<Phins::UserStatus>(msg.status);
      FILL_SENSOR_HDR_IOTIME(d->phins_status_, bytes.ds_header.io_time);
      d->phins_status_.user.status = msg.status;

      if ((d->phins_standard_reception_status_ & PhinsPrivate::PHINS_STANDARD_STATUS_MASK) ==
          PhinsPrivate::PHINS_STANDARD_STATUS_MASK)
      {
        d->phins_status_pub_.publish(d->phins_status_);
      }
    }

    ROS_DEBUG_STREAM("Phins Ignoring: " << nmea_msg);
  }
}
}
