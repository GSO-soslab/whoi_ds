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

#ifndef DS_SENSOR_RESONSVP70_H
#define DS_SENSOR_RESONSVP70_H

#include "ds_base/sensor_base.h"
#include "ds_sensor_msgs/SoundSpeed.h"

namespace ds_sensors
{
struct ResonSvp70Private;

/// @brief Sensor class for the Reson 70/71 Sound Velocity Probe
///
/// Topic             Message Type                     Description
/// ---------         ----------------------------     ----------------------------------
/// $name/raw         ds_core_msgs::RawData            all raw bytes received from sensor
/// $name/sound_speed ds_sensor_msgs::SoundSpeed       successfully parsed messages
class ResonSvp70 : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(ResonSvp70)

public:
  enum class OutputFormat
  {
    Aml = 2,
    Valeport = 3,
    Reson = 4,
    Svp7x = 5,
  };

  explicit ResonSvp70();
  ResonSvp70(int argc, char* argv[], const std::string& name);
  ~ResonSvp70() override;
  DS_DISABLE_COPY(ResonSvp70)

  ///@brief Parse a message from the SBE49
  ///
  ///
  /// Returns a std::pair<bool, ds_msgs::SoundSpeed>.  The boolean indicates whether parsing was successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \param format   Format type (only output type 3 supported)
  /// \return
  static std::pair<bool, ds_sensor_msgs::SoundSpeed> parse_bytes(const ds_core_msgs::RawData& bytes,
                                                                 double clamping_min, double clamping_max,
                                                                 OutputFormat format = OutputFormat::Aml);

  ///@brief Set the output format for records
  ///
  /// This only sets the *expected* output format.  It does not change the
  /// `OutputFormat` parameter on the SBE49 itself.
  ///
  /// **NOTE** Only output format `EngineeringDecimal` (`OutputFormat=3`) is supported at present.
  /// \param format
  void setOutputFormat(OutputFormat format) noexcept;

  ///@brief Get the output format for records
  ///
  /// \return
  OutputFormat outputFormat() const noexcept;

protected:
  void setupPublishers() override;
  void setupParameters() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  double speed_clamping_min_;
  double speed_clamping_max_;

  std::unique_ptr<ResonSvp70Private> d_ptr_;
};

}  // namespace ds_sensors

#endif  // DS_SENSOR_RESONSVP70_H
