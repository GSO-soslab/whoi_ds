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

#ifndef DS_SENSORS_PHINS_H
#define DS_SENSORS_PHINS_H

#include "ds_base/sensor_base.h"
#include "ds_core_msgs/RawData.h"

#include "ds_nmea_msgs/PixseAlgsts.h"
#include "ds_nmea_msgs/PixseStatus.h"
#include "ds_nmea_msgs/PixseHtsts.h"

namespace ds_sensors
{
struct PhinsPrivate;

/// @brief Sensor class for the IxBlue Phins Inertial Navigation System
///
/// Topic                         Message Type                          Description
/// ---------                     ----------------------------          ----------------------------------
/// $name/raw                     ds_core_msgs::RawData                 all raw bytes received from sensor
/// $name/gyro                    ds_sensor_msgs::Gyro                  Roll/Pitch/Heading data
/// $name/compass                 ds_sensor_msgs::Compass               Heading data
/// $name/phins_status            ds_nmea_msgs::PixseAlgsts             Phins Algorithm status
class Phins : public ds_base::SensorBase
{
protected:
  DS_DECLARE_PRIVATE(Phins)

public:
  explicit Phins();
  Phins(int argc, char* argv[], const std::string& name);
  ~Phins() override;
  DS_DISABLE_COPY(Phins)

  /// @brief Built-in test status flags for PHINS algorithms.
  enum AlgorithmStatus : uint64_t
  {
    ALGORITHM_STATUS_NONE = 0,
    //
    // ---- 32 LSB of ALGSTS flags -----
    //
    ALG_OPERATION_MODE_NAVIGATION = ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_NAVIGATION,
    ALG_OPERATION_MODE_ALIGNMENT = ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_ALIGNMENT,
    ALG_OPERATION_MODE_FINE_ALIGNMENT = ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_FINE_ALIGNMENT,
    ALG_OPERATION_MODE_DECK_RECKONING = ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_DECK_RECKONING,

    ALG_ALTITUDE_USING_GPS = ds_nmea_msgs::PixseAlgsts::ALTITUDE_USING_GPS,
    ALG_ALTITUDE_USING_DEPTH = ds_nmea_msgs::PixseAlgsts::ALTITUDE_USING_DEPTH,
    ALG_ALTITUDE_SABLILIZED = ds_nmea_msgs::PixseAlgsts::ALTITUDE_SABLILIZED,
    ALG_ALTITUDE_HYDRO = ds_nmea_msgs::PixseAlgsts::ALTITUDE_HYDRO,

    ALG_LOG_USED = ds_nmea_msgs::PixseAlgsts::LOG_USED,
    ALG_LOG_DATA_VALID = ds_nmea_msgs::PixseAlgsts::LOG_DATA_VALID,
    ALG_LOG_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::LOG_WAITING_FOR_DATA,
    ALG_LOG_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::LOG_DATA_REJECTED,

    ALG_GPS_USED = ds_nmea_msgs::PixseAlgsts::GPS_USED,
    ALG_GPS_DATA_VALID = ds_nmea_msgs::PixseAlgsts::GPS_DATA_VALID,
    ALG_GPS_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::GPS_WAITING_FOR_DATA,
    ALG_GPS_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::GPS_DATA_REJECTED,

    ALG_USBL_USED = ds_nmea_msgs::PixseAlgsts::USBL_USED,
    ALG_USBL_DATA_VALID = ds_nmea_msgs::PixseAlgsts::USBL_DATA_VALID,
    ALG_USBL_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::USBL_WAITING_FOR_DATA,
    ALG_USBL_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::USBL_DATA_REJECTED,

    ALG_DEPTH_USED = ds_nmea_msgs::PixseAlgsts::DEPTH_USED,
    ALG_DEPTH_DATA_VALID = ds_nmea_msgs::PixseAlgsts::DEPTH_DATA_VALID,
    ALG_DEPTH_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::DEPTH_WAITING_FOR_DATA,
    ALG_DEPTH_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::DEPTH_DATA_REJECTED,

    ALG_LBL_USED = ds_nmea_msgs::PixseAlgsts::LBL_USED,
    ALG_LBL_DATA_VALID = ds_nmea_msgs::PixseAlgsts::LBL_DATA_VALID,
    ALG_LBL_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::LBL_WAITING_FOR_DATA,
    ALG_LBL_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::LBL_DATA_REJECTED,

    ALTITUDE_SATURATED = ds_nmea_msgs::PixseAlgsts::ALTITUDE_SATURATED,
    SPEED_SATURATED = ds_nmea_msgs::PixseAlgsts::SPEED_SATURATED,
    // RESERVED_1                         = ds_nmea_msgs::PixseAlgsts::RESERVED_1,
    // RESERVED_2                         = ds_nmea_msgs::PixseAlgsts::RESERVED_2,

    //
    // ---- 32 MSB of ALGSTS flags -----
    //

    ALG_WATER_TRACK_USED = ds_nmea_msgs::PixseAlgsts::WATER_TRACK_USED,
    ALG_WATER_TRACK_DATA_VALID = ds_nmea_msgs::PixseAlgsts::WATER_TRACK_DATA_VALID,
    ALG_WATER_TRACK_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::WATER_TRACK_WAITING_FOR_DATA,
    ALG_WATER_TRACK_REJECTED = ds_nmea_msgs::PixseAlgsts::WATER_TRACK_REJECTED,

    ALG_GPS2_USED = ds_nmea_msgs::PixseAlgsts::GPS2_USED,
    ALG_GPS2_DATA_VALID = ds_nmea_msgs::PixseAlgsts::GPS2_DATA_VALID,
    ALG_GPS2_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::GPS2_WAITING_FOR_DATA,
    ALG_GPS2_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::GPS2_DATA_REJECTED,

    ALG_METROLOGY_USED = ds_nmea_msgs::PixseAlgsts::METROLOGY_USED,
    ALG_METROLOGY_DATA_VALID = ds_nmea_msgs::PixseAlgsts::METROLOGY_DATA_VALID,
    ALG_METROLOGY_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::METROLOGY_WAITING_FOR_DATA,
    ALG_METROLOGY_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::METROLOGY_DATA_REJECTED,

    ALG_ALTITUDE_USED = ds_nmea_msgs::PixseAlgsts::ALTITUDE_USED,
    ALG_ALTITUDE_DATA_VALID = ds_nmea_msgs::PixseAlgsts::ALTITUDE_DATA_VALID,
    ALG_ALTITUDE_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::ALTITUDE_WAITING_FOR_DATA,
    ALG_ALTITUDE_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::ALTITUDE_DATA_REJECTED,

    ALG_OPERATION_MODE_ZUP = ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_ZUP,
    ALG_ZUP_VALID = ds_nmea_msgs::PixseAlgsts::ZUP_VALID,
    ALG_OPERATION_MODE_ZUP_VALID = ds_nmea_msgs::PixseAlgsts::OPERATION_MODE_ZUP_VALID,
    ALG_ZUP_BENCH_VALID = ds_nmea_msgs::PixseAlgsts::ZUP_BENCH_VALID,

    ALG_STATIC_ALIGNMENT = ds_nmea_msgs::PixseAlgsts::STATIC_ALIGNMENT,
    ALG_GO_TO_NAV = ds_nmea_msgs::PixseAlgsts::GO_TO_NAV,
    // RESERVED_1                         = ds_nmea_msgs::PixseAlgsts::RESERVED_1,
    // RESERVED_2                         = ds_nmea_msgs::PixseAlgsts::RESERVED_2,

    ALG_EM_LOG_USED = ds_nmea_msgs::PixseAlgsts::EM_LOG_USED,
    ALG_EM_LOG_DATA_VALID = ds_nmea_msgs::PixseAlgsts::EM_LOG_DATA_VALID,
    ALG_EM_LOG_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::EM_LOG_WAITING_FOR_DATA,
    ALG_EM_LOG_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::EM_LOG_DATA_REJECTED,

    ALG_GPS_MANUAL_USED = ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_USED,
    ALG_GPS_MANUAL_DATA_VALID = ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_DATA_VALID,
    ALG_GPS_MANUAL_WAITING_FOR_DATA = ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_WAITING_FOR_DATA,
    ALG_GPS_MANUAL_DATA_REJECTED = ds_nmea_msgs::PixseAlgsts::GPS_MANUAL_DATA_REJECTED,
  };

  /// @brief Built-in test status flags for Phins gyro and accelerometers
  enum SystemStatus : uint64_t
  {
    SYSTEM_SERIAL_INPUT_R_ERROR = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_R_ERROR,
    SYSTEM_SERIAL_INPUT_A_ERROR = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_A_ERROR,
    SYSTEM_SERIAL_INPUT_B_ERROR = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_B_ERROR,
    SYSTEM_SERIAL_INPUT_C_ERROR = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_C_ERROR,

    SYSTEM_SERIAL_INPUT_D_ERROR = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_D_ERROR,
    SYSTEM_SERIAL_INPUT_E_ERROR = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_E_ERROR,
    SYSTEM_RESERVED_01 = ds_nmea_msgs::PixseStatus::RESERVED_01,
    SYSTEM_RESERVED_02 = ds_nmea_msgs::PixseStatus::RESERVED_02,

    SYSTEM_SERIAL_INPUT_R_ACTIVITY = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_R_ACTIVITY,
    SYSTEM_SERIAL_INPUT_A_ACTIVITY = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_A_ACTIVITY,
    SYSTEM_SERIAL_INPUT_B_ACTIVITY = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_B_ACTIVITY,
    SYSTEM_SERIAL_INPUT_C_ACTIVITY = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_C_ACTIVITY,

    SYSTEM_SERIAL_INPUT_D_ACTIVITY = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_D_ACTIVITY,
    SYSTEM_SERIAL_INPUT_E_ACTIVITY = ds_nmea_msgs::PixseStatus::SERIAL_INPUT_E_ACTIVITY,
    SYSTEM_RESERVED_03 = ds_nmea_msgs::PixseStatus::RESERVED_03,
    SYSTEM_RESERVED_04 = ds_nmea_msgs::PixseStatus::RESERVED_04,

    SYSTEM_SERIAL_OUTPUT_R_FULL = ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_R_FULL,
    SYSTEM_SERIAL_OUTPUT_A_FULL = ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_A_FULL,
    SYSTEM_SERIAL_OUTPUT_B_FULL = ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_B_FULL,
    SYSTEM_SERIAL_OUTPUT_C_FULL = ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_C_FULL,

    SYSTEM_SERIAL_OUTPUT_D_FULL = ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_D_FULL,
    SYSTEM_SERIAL_OUTPUT_E_FULL = ds_nmea_msgs::PixseStatus::SERIAL_OUTPUT_E_FULL,
    SYSTEM_RESERVED_05 = ds_nmea_msgs::PixseStatus::RESERVED_05,
    SYSTEM_RESERVED_06 = ds_nmea_msgs::PixseStatus::RESERVED_06,

    SYSTEM_RESERVED_07 = ds_nmea_msgs::PixseStatus::RESERVED_07,
    SYSTEM_RESERVED_08 = ds_nmea_msgs::PixseStatus::RESERVED_08,
    SYSTEM_ETHERNET_ACTIVITY = ds_nmea_msgs::PixseStatus::ETHERNET_ACTIVITY,
    SYSTEM_USER_CONTROL_BIT_A = ds_nmea_msgs::PixseStatus::USER_CONTROL_BIT_A,

    SYSTEM_USERECONTROL_BIT_B = ds_nmea_msgs::PixseStatus::USERECONTROL_BIT_B,
    SYSTEM_USER_CONTROL_BIT_C = ds_nmea_msgs::PixseStatus::USER_CONTROL_BIT_C,
    SYSTEM_USER_CONTROL_BIT_D = ds_nmea_msgs::PixseStatus::USER_CONTROL_BIT_D,
    SYSTEM_RESERVED_09 = ds_nmea_msgs::PixseStatus::RESERVED_09,

    SYSTEM_DVL_BOTTOM_TRACK_DETECTED = ds_nmea_msgs::PixseStatus::DVL_BOTTOM_TRACK_DETECTED,
    SYSTEM_DVL_WATER_TRACK_DETECTED = ds_nmea_msgs::PixseStatus::DVL_WATER_TRACK_DETECTED,
    SYSTEM_GPS1_DETECTED = ds_nmea_msgs::PixseStatus::GPS1_DETECTED,
    SYSTEM_GPS2_DETECTED = ds_nmea_msgs::PixseStatus::GPS2_DETECTED,

    SYSTEM_USBL_DETECTED = ds_nmea_msgs::PixseStatus::USBL_DETECTED,
    SYSTEM_LBL_DETECTED = ds_nmea_msgs::PixseStatus::LBL_DETECTED,
    SYSTEM_DEPTH_DETECTED = ds_nmea_msgs::PixseStatus::DEPTH_DETECTED,
    SYSTEM_LOG_EM_DETECTED = ds_nmea_msgs::PixseStatus::LOG_EM_DETECTED,

    SYSTEM_ODOMETER_DETECTED = ds_nmea_msgs::PixseStatus::ODOMETER_DETECTED,
    SYSTEM_UTC_DETECTED = ds_nmea_msgs::PixseStatus::UTC_DETECTED,
    SYSTEM_ALTITUDE_DETECTED = ds_nmea_msgs::PixseStatus::ALTITUDE_DETECTED,
    SYSTEM_PPS_DETECTED = ds_nmea_msgs::PixseStatus::PPS_DETECTED,

    SYSTEM_ZUP_ACTIVATED = ds_nmea_msgs::PixseStatus::ZUP_ACTIVATED,
    SYSTEM_METROLOGY_DETECTED = ds_nmea_msgs::PixseStatus::METROLOGY_DETECTED,
    SYSTEM_MANUAL_GPS_DETECTED = ds_nmea_msgs::PixseStatus::MANUAL_GPS_DETECTED,
    SYSTEM_CTD_DETECTED = ds_nmea_msgs::PixseStatus::CTD_DETECTED,

    SYSTEM_HRP_DEGRADED = ds_nmea_msgs::PixseStatus::HRP_DEGRADED,
    SYSTEM_HRP_NOT_VALID = ds_nmea_msgs::PixseStatus::HRP_NOT_VALID,
    SYSTEM_RESERVED_10 = ds_nmea_msgs::PixseStatus::RESERVED_10,
    SYSTEM_RESERVED_11 = ds_nmea_msgs::PixseStatus::RESERVED_11,

    SYSTEM_RESERVED_12 = ds_nmea_msgs::PixseStatus::RESERVED_12,
    SYSTEM_RESERVED_13 = ds_nmea_msgs::PixseStatus::RESERVED_13,
    SYSTEM_RESERVED_14 = ds_nmea_msgs::PixseStatus::RESERVED_14,
    SYSTEM_RESERVED_15 = ds_nmea_msgs::PixseStatus::RESERVED_15,

    SYSTEM_RESERVED_16 = ds_nmea_msgs::PixseStatus::RESERVED_16,
    SYSTEM_RESERVED_17 = ds_nmea_msgs::PixseStatus::RESERVED_17,
    SYSTEM_RESERVED_18 = ds_nmea_msgs::PixseStatus::RESERVED_18,
    SYSTEM_MPC_OVERLOAD = ds_nmea_msgs::PixseStatus::MPC_OVERLOAD,

    SYSTEM_FAULT_ALARM = ds_nmea_msgs::PixseStatus::FAULT_ALARM,
    SYSTEM_MANUFACTURES_MODE = ds_nmea_msgs::PixseStatus::MANUFACTURES_MODE,
    SYSTEM_CONFIGURATION_SAVED = ds_nmea_msgs::PixseStatus::CONFIGURATION_SAVED,
    SYSTEM_SYSTEM_RESTARTED = ds_nmea_msgs::PixseStatus::SYSTEM_RESTARTED,

  };

  /// @brief Built-in test status fusing the Algorithm and Sensor status flags.
  enum UserStatus : uint32_t
  {

    USER_SYSTEM_OK = ds_nmea_msgs::PixseHtsts::SYSTEM_OK,
    USER_ALIGNMENT_IN_PROGRESS = ds_nmea_msgs::PixseHtsts::ALIGNMENT_IN_PROGRESS,
    USER_SYSTEM_ERROR = ds_nmea_msgs::PixseHtsts::SYSTEM_ERROR,
    USER_SYSTEM_WARNING = ds_nmea_msgs::PixseHtsts::SYSTEM_WARNING,

    USER_SERIAL_INPUT_OK = ds_nmea_msgs::PixseHtsts::SERIAL_INPUT_OK,
    USER_SERIAL_INPUT_ERROR = ds_nmea_msgs::PixseHtsts::SERIAL_INPUT_ERROR,
    USER_SERIAL_OUTPUT_OK = ds_nmea_msgs::PixseHtsts::SERIAL_OUTPUT_OK,
    USER_SERIAL_OUTPUT_ERROR = ds_nmea_msgs::PixseHtsts::SERIAL_OUTPUT_ERROR,

    USER_ELECTRONIC_OK = ds_nmea_msgs::PixseHtsts::ELECTRONIC_OK,
    USER_ELECTRONIC_ERROR = ds_nmea_msgs::PixseHtsts::ELECTRONIC_ERROR,
    USER_FOG_OK = ds_nmea_msgs::PixseHtsts::FOG_OK,
    USER_FOG_ERROR = ds_nmea_msgs::PixseHtsts::FOG_ERROR,

    USER_ACCEL_OK = ds_nmea_msgs::PixseHtsts::ACCEL_OK,
    USER_ACCEL_ERROR = ds_nmea_msgs::PixseHtsts::ACCEL_ERROR,
    USER_CPU_OK = ds_nmea_msgs::PixseHtsts::CPU_OK,
    USER_CPU_ERROR = ds_nmea_msgs::PixseHtsts::CPU_ERROR,

    USER_TEMP_OK = ds_nmea_msgs::PixseHtsts::TEMP_OK,
    USER_TEMP_ERROR = ds_nmea_msgs::PixseHtsts::TEMP_ERROR,
    USER_NO_GPS1_DETECTED = ds_nmea_msgs::PixseHtsts::NO_GPS1_DETECTED,
    USER_NO_GPS2_DETECTED = ds_nmea_msgs::PixseHtsts::NO_GPS2_DETECTED,

    USER_NO_MANUAL_GPS_DETECTED = ds_nmea_msgs::PixseHtsts::NO_MANUAL_GPS_DETECTED,
    USER_NO_DVL_BOTTOM_TRACK_DETECTED = ds_nmea_msgs::PixseHtsts::NO_DVL_BOTTOM_TRACK_DETECTED,
    USER_NO_DVL_WATER_TRACK_DETECTED = ds_nmea_msgs::PixseHtsts::NO_DVL_WATER_TRACK_DETECTED,
    USER_NO_EM_LOG_DETECTED = ds_nmea_msgs::PixseHtsts::NO_EM_LOG_DETECTED,

    USER_NO_DEPTH_DETECTED = ds_nmea_msgs::PixseHtsts::NO_DEPTH_DETECTED,
    USER_NO_USBL_DETECTED = ds_nmea_msgs::PixseHtsts::NO_USBL_DETECTED,
    USER_NO_LBL_DETECTED = ds_nmea_msgs::PixseHtsts::NO_LBL_DETECTED,
    USER_NO_ALITITUDE_DETECTED = ds_nmea_msgs::PixseHtsts::NO_ALITITUDE_DETECTED,

    USER_NO_UTC_SYNC_DETECTED = ds_nmea_msgs::PixseHtsts::NO_UTC_SYNC_DETECTED,
    USER_NO_PPS_SYNC_DETECTED = ds_nmea_msgs::PixseHtsts::NO_PPS_SYNC_DETECTED,
    USER_NO_CTD_DETECTED = ds_nmea_msgs::PixseHtsts::NO_CTD_DETECTED,
    USER_ZUP_MODE_ACTIVATED = ds_nmea_msgs::PixseHtsts::ZUP_MODE_ACTIVATED,

  };

protected:
  void setupConnections() override;
  void setupPublishers() override;
  void checkProcessStatus(const ros::TimerEvent& event) override;

  void parseReceivedPhinsStandard(const ds_core_msgs::RawData& bytes);

private:
  std::unique_ptr<PhinsPrivate> d_ptr_;
};
}
#endif  // DS_SENSORS_PHINS_H
