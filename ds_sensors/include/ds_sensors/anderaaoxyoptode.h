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
// Created by zac on 12/11/17.
//

#ifndef DS_SENSOR_ANDERAA02OPTODE_H
#define DS_SENSOR_ANDERAA02OPTODE_H

#include "ds_base/sensor_base.h"
#include "ds_core_msgs/RawData.h"
#include "ds_sensor_msgs/OxygenConcentration.h"

namespace ds_sensors
{
struct AnderaaOxyOptodePrivate;

/// @brief Sensor class for the Anderra Oxygen Optode models 4330, 4835, and 4831
///
/// Topic                       Message Type                         Description
/// ---------                   ----------------------------         ----------------------------------
/// $name/raw                   ds_core_msgs::RawData                all raw bytes received from sensor
/// $name/oxygen_concentration  ds_sensor_msgs::OxygenConcentration  successfully parsed messages
class AnderaaOxyOptode : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(AnderaaOxyOptode)

public:
  explicit AnderaaOxyOptode();
  AnderaaOxyOptode(int argc, char* argv[], const std::string& name);
  ~AnderaaOxyOptode() override;
  DS_DISABLE_COPY(AnderaaOxyOptode)

  enum class OutputFormat
  {
    RawHex = 0,
    EngineeringHex,
    RawDecimal,
    EngineeringDecimal
  };

  ///@brief Parse a message from the Anderra Optode
  ///
  ///
  /// Returns a std::pair<bool, ds_msgs::OxygenConcentration>.  The boolean indicates whether parsing was successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \param format   Format type (only output type 3 supported)
  /// \return
  static std::pair<bool, ds_sensor_msgs::OxygenConcentration>
  parse_bytes(const ds_core_msgs::RawData& bytes, OutputFormat format = OutputFormat::EngineeringDecimal);

protected:
  void setupPublishers() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  std::unique_ptr<AnderaaOxyOptodePrivate> d_ptr_;
};

}  // namespace ds_sensors

#endif  // DS_SENSOR_ANDERAA02OPTODE_H
