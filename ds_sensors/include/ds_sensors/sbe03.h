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

#ifndef DS_SENSOR_SBE03_H
#define DS_SENSOR_SBE03_H

#include "ds_base/sensor_base.h"
#include "ds_sensor_msgs/Frequency.h"

namespace ds_sensors
{
struct Sbe03Private;

/// @brief Sensor class for the SeaBird SBE03 Temperature Sensor with Freq converter
///
///
/// Topic       Message Type                     Description
/// ---------   ----------------------------     ----------------------------------
/// $name/raw   ds_core_msgs::RawData            all raw bytes received from sensor
/// $name/hz    ds_sensor_msgs::Frequency        successfully parsed messages
class Sbe03 : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(Sbe03)

 public:
  explicit Sbe03();
  Sbe03(int argc, char* argv[], const std::string& name);
  ~Sbe03() override;
  DS_DISABLE_COPY(Sbe03)


  ///@brief Parse a message from the SBE03
  ///
  ///
  /// Returns a std::pair<bool, ds_msgs::Frequency>.  The boolean indicates whether parsing was successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \param format   Format type (only output type 3 supported)
  /// \return
  static std::pair<bool, ds_sensor_msgs::Frequency> parse_bytes(const ds_core_msgs::RawData& bytes);

 protected:
  void setupPublishers() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes);

 private:
  std::unique_ptr<Sbe03Private> d_ptr_;
};

}  // namespace ds_sensors
#endif  // DS_SENSOR_SBE03_H
