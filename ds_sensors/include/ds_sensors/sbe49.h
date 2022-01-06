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
// Created by zac on 12/5/17.
//

#ifndef DS_SENSOR_SBE49_H
#define DS_SENSOR_SBE49_H

#include "ds_base/sensor_base.h"
#include "ds_sensor_msgs/Ctd.h"

namespace ds_sensors
{
struct Sbe49Private;

/// @brief Sensor class for the SeaBird SBE49 CTD
///
/// Reads `OutputType=3` messages with 3 or 5 fields.
///
/// Topic       Message Type                     Description
/// ---------   ----------------------------     ----------------------------------
/// $name/raw   ds_core_msgs::RawData            all raw bytes received from sensor
/// $name/ctd   ds_sensor_msgs::Ctd              successfully parsed messages
class Sbe49 : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(Sbe49)

public:
  explicit Sbe49();
  Sbe49(int argc, char* argv[], const std::string& name);
  ~Sbe49() override;
  DS_DISABLE_COPY(Sbe49)

  enum class OutputFormat
  {
    RawHex = 0,
    EngineeringHex,
    RawDecimal,
    EngineeringDecimal
  };

  ///@brief Parse a message from the SBE49
  ///
  ///
  /// Returns a std::pair<bool, ds_msgs::Ctd>.  The boolean indicates whether parsing was successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \param format   Format type (only output type 3 supported)
  /// \return
  static std::pair<bool, ds_sensor_msgs::Ctd> parse_bytes(const ds_core_msgs::RawData& bytes,
                                                          OutputFormat format = OutputFormat::EngineeringDecimal);

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
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes);

private:
  std::unique_ptr<Sbe49Private> d_ptr_;
};

}  // namespace ds_sensors
#endif  // DS_SENSOR_SBE49_H
