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
// Created by jvaccaro on 9/28/18.
//

#ifndef PROJECT_NORTEKVECTOR_H
#define PROJECT_NORTEKVECTOR_H

#include "ds_base/sensor_base.h"
#include "ds_core_msgs/RawData.h"
#include "ds_sensor_msgs/Velocity3D.h"
#include "ds_sensor_msgs/NortekVectorSystem.h"
#include "ds_core_msgs/VoidCmd.h"
#include "ds_core_msgs/StringCmd.h"
#include "ds_util/ds_util.h"

namespace ds_sensors
{
struct NortekVectorPrivate;

/// \brief Device driver for the NortekVector Velocimeter, which measures water speed
///
/// \pub velocity3D includes velocity measurements, amplitudes, and correlations
/// \pub system includes orientation data and system health data
///
/// \srv set_mode_acquisition sets the device to spit out data regularly
/// \srv set_mode_command sets the device to standby for commands
/// \srv set_mode_shutdown sets the device to low power until set to command mode
///
/// \srv send_command sends a direct passthrough to the device, evaluates whether it was received
class NortekVector : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(NortekVector)

public:
  explicit NortekVector();
  NortekVector(int argc, char* argv[], const std::string& name);
  ~NortekVector() override;
  DS_DISABLE_COPY(NortekVector);

  /// \brief Main parsing function run whenever a new message is received
  /// Static for easy parsing unit tests
  /// If successful, returns a 1-hot bool and a single populated message to send
  /// If failure, returns two false bools and no populated messages
  static std::tuple<bool, bool, ds_sensor_msgs::Velocity3D, ds_sensor_msgs::NortekVectorSystem>
  parse_bytes(const ds_core_msgs::RawData& bytes);

protected:
  void setupPublishers() override;
  void setupServices() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  std::unique_ptr<NortekVectorPrivate> d_ptr_;

  enum class DATA_ID
  {
    HARDWARE_CONFIG = 0x05,
    HEAD_CONFIG = 0x04,
    USER_CONFIG = 0x00,
    //  AQUADOPP_VELO_DATA = 0x01,
    //  AQUADOPP_DIAG_DATA_HDR = 0x06,
    //  AQUADOPP_DIAG_DATA = 0x80,
    VECTOR_VELO_DATA_HDR = 0x12,
    VECTOR_VELO_DATA = 0x10,
    VECTOR_SYSTEM_DATA = 0x11,
    VECTOR_PROBE_CHECK_DATA = 0x07
  };

  /// \brief set mode to "acquisition"
  ///
  /// This means that the data should start spitting out on both topics within a few seconds
  /// \return True if completed
  bool set_mode_acquisition(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp);

  /// \brief set mode to "command"
  ///
  /// Instrument should stop spitting data and be ready to receive commands
  /// \return True if completed
  bool set_mode_command(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp);

  /// \brief set mode to "shutdown"
  ///
  /// Instrument should stop spitting data and be on standby
  /// \return True if completed
  bool set_mode_shutdown(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp);

  /// \brief send a passthrough command directly on the serial port to the NortekVector
  ///
  /// Should assume that if the mode is not "command" that the command was unsuccessful
  /// \return
  bool send_command(const ds_core_msgs::StringCmd::Request& req, ds_core_msgs::StringCmd::Response& resp);

  /// \brief recast raw data to velocity struct, then construct a Velocity3D msg
  ///
  /// \param raw
  /// \return True if successful and passes checksum
  static std::pair<bool, ds_sensor_msgs::Velocity3D> parseVelocityData(const uint8_t* raw);

  /// \brief recast raw data to system struct, then construct a NortekVectorSystem msg
  ///
  /// \param raw
  /// \return True if successful and passes checksum
  static std::pair<bool, ds_sensor_msgs::NortekVectorSystem> parseSystemData(const uint8_t* raw);

  /// \brief Confirms that NortekVector type checksum passes
  ///
  /// \param length length of the data (positive even integer)
  /// \param raw pointer to the actual data getting checked
  /// \param chksum the expected checksum value, read from the data
  /// \return
  static bool checksumPassed(uint16_t length, const uint8_t* raw, uint16_t chksum);
};
}

#endif  // PROJECT_NORTEKVECTOR_H
