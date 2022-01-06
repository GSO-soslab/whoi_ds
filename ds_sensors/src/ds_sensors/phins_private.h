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

#ifndef DS_SENSORS_PHINS_PRIVATE_H
#define DS_SENSORS_PHINS_PRIVATE_H

#include "ds_sensors/phins.h"
#include "ds_nmea_msgs/PixseAlgsts.h"

#include "ds_sensor_msgs/Gyro.h"
#include "ds_sensor_msgs/Compass.h"
#include "ds_sensor_msgs/PhinsStatus.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ds_sensors
{
struct PhinsPrivate
{
  PhinsPrivate() = default;
  ~PhinsPrivate() = default;

  /// @brief Enum for all the different parts of the Phins Standard message type
  ///
  /// This enum is used to create masks to check if all the needed parts have arrived
  /// for a specific type of outgoing message.
  enum PhinsStandardParts
  {
    HEHDT = 0x00000001,
    ATITUD = 0x00000002,
    POSITI = 0x00000004,
    SPEED = 0x00000008,
    UTMWGS = 0x00000010,
    HEAVE = 0x00000020,
    STDHRP = 0x00000040,
    STDPOS = 0x00000080,
    STDSPD = 0x00000100,
    TIME = 0x00000200,
    LOGIN = 0x00000400,
    LOGDVL = 0x00000800,
    LOGWAT = 0x00001000,
    GPSIN = 0x00002000,
    GP2IN = 0x00004000,
    GPMIN = 0x00008000,
    DEPIN = 0x00010000,
    USBIN = 0x00020000,
    LBLIN = 0x00040000,
    UTCIN = 0x00080000,
    LMNIN = 0x00100000,
    DDRECK = 0x00200000,
    ALGSTS = 0x00400000,
    STATUS = 0x00800000,
    HT_STS = 0x01000000
  };

  /// The Gyro message needs heading, pitch, roll, and the errors
  static const auto PHINS_STANDARD_GYRO_MASK =
      PhinsStandardParts::HEHDT | PhinsStandardParts::ATITUD | PhinsStandardParts::STDHRP;

  /// The Compass message needs heading and it's error
  static const auto PHINS_STANDARD_COMPASS_MASK = PhinsStandardParts::HEHDT | PhinsStandardParts::STDHRP;

  /// The phins status message combines three individual status messages.
  static const auto PHINS_STANDARD_STATUS_MASK =
      PhinsStandardParts::ALGSTS | PhinsStandardParts::STATUS | PhinsStandardParts::HT_STS;

  ///@brief Read callback for the "phins_standard" connection
  /// \param bytes
  void parseReceivedPhinsStandard(const ds_core_msgs::RawData& bytes);

  ds_sensor_msgs::Gyro gyro_;
  ds_sensor_msgs::Compass compass_;
  ds_sensor_msgs::PhinsStatus phins_status_;

  uint64_t phins_standard_reception_status_;  //!< Bitflag tracking the reception of nmea messages.

  Phins::AlgorithmStatus algorithm_status_;  //!< Algorithm Status
  Phins::SystemStatus system_status_;        //!< Sensor Status
  Phins::UserStatus user_status_;            //!< User Status

  ros::Publisher gyro_pub_;
  ros::Publisher compass_pub_;
  ros::Publisher phins_status_pub_;
};
}

#endif  // DS_SENSORS_PHINS_PRIVATE_H
