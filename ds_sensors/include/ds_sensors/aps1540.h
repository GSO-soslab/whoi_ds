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
// Created by zac on 12/8/17.
//

#ifndef DS_SENSORS_APS1540_H
#define DS_SENSORS_APS1540_H

#include "ds_base/sensor_base.h"
#include "ds_core_msgs/RawData.h"
#include "ds_sensor_msgs/VectorMagneticField.h"

namespace ds_sensors
{
struct Aps1540Private;

/// @brief Sensor class for the Applied Physics 1540 Vector Magnetometer
///
/// Topic                         Message Type                          Description
/// ---------                     ----------------------------          ----------------------------------
/// $name/raw                     ds_core_msgs::RawData                 all raw bytes received from sensor
/// $name/vector_magnetic_field   ds_sensor_msgs::VectorMagneticField   successfully parsed messages
class Aps1540 : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(Aps1540)

public:
  explicit Aps1540();
  Aps1540(int argc, char* argv[], const std::string& name);
  ~Aps1540() override;
  DS_DISABLE_COPY(Aps1540)
  ///@brief Parse a message from the Aps1540
  ///
  ///
  /// Returns a std::pair<bool, ds_sensor_msgs::VectorMagneticField>.  The boolean indicates whether parsing was
  /// successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \return
  static std::pair<bool, ds_sensor_msgs::VectorMagneticField> parse_bytes(const ds_core_msgs::RawData& bytes);

protected:
  void setupPublishers() override;

  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  std::unique_ptr<Aps1540Private> d_ptr_;
};
}
#endif  // DS_SENSORS_APS1540_H
