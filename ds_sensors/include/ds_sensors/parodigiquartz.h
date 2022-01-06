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
// Created by zac on 12/9/17.
//

#ifndef DS_SENSOR_PARODIGIQUARTZ_H
#define DS_SENSOR_PARODIGIQUARTZ_H

#include "depthsensor.h"

namespace ds_sensors
{
struct ParoDigiquartzPrivate;

/// @brief Sensor class for the Paro Scientifi Digiquartz depth sensor
///
/// Reads `OutputType=3` messages with 3 or 5 fields.
///
/// Topic       Message Type                     Description
/// ----------- ----------------------------     ----------------------------------
/// $name/raw   ds_core_msgs::RawData            all raw bytes received from sensor
/// $name/depth ds_sensor_msgs::DepthPressure    successfully parsed messages
 class ParoDigiquartz : public ds_sensors::DepthSensor
{
  DS_DECLARE_PRIVATE(ParoDigiquartz)

public:

  explicit ParoDigiquartz();
  ParoDigiquartz(int argc, char* argv[], const std::string& name);
  ~ParoDigiquartz() override;
  DS_DISABLE_COPY(ParoDigiquartz)

  /// @brief Parse a pressure reading message.
  ///
  /// This does *not* populate the entire DepthPressure message.  Quantities such as latitude, pressure unit,
  /// and tare value are required and are not part of the Paros pressure record.
  ///
  /// \param bytes
  /// \return
  static std::pair<bool, ds_sensor_msgs::DepthPressure> parse_pressure(const ds_core_msgs::RawData& bytes);

  /// @brief Start periodic sampling (send P4 command)
  ///
  /// \return
  void start(uint8_t id = 1);

  /// @brief Send a command on the "instrument" connection
  ///
  /// A leading '*' and trailing '\r\n' are automatically added if either are missing.
  ///
  /// \param command
  void sendCommand(std::string command, std::string connection) override;

protected:
  void setup() override;
  void setupParameters() override;

  // simply a virtually-bound wrapper around parse_pressure
  std::pair<bool, ds_sensor_msgs::DepthPressure> parse_message(const ds_core_msgs::RawData& bytes) override;

private:
  std::unique_ptr<ParoDigiquartzPrivate> d_ptr_;
};

}  // namespace ds_sensors

#endif  // DS_SENSOR_PARODIGIQUARTZ_H
