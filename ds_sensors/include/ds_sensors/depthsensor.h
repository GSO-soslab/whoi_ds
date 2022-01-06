/**
* Copyright 2019 Woods Hole Oceanographic Institution
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
// Created by ivaughn on 8/6/19.
//

#ifndef DS_SENSORS_DEPTHSENSOR_H
#define DS_SENSORS_DEPTHSENSOR_H
#include "ds_base/sensor_base.h"
#include "ds_util/median.h"
#include "ds_core_msgs/RawData.h"
#include "ds_sensor_msgs/DepthPressure.h"
#include <ds_core_msgs/VoidCmd.h>
#include <sensor_msgs/NavSatFix.h>

namespace ds_sensors
{
struct DepthSensorPrivate;

/// @brief Base class for a pressure-based depth sensor.  Handles pressure-to-depth conversion, etc
/// Entirely pulled out of Zac's Paroscientific driver from the early days of the Sentry ROS upgrade
///
/// Reads `OutputType=3` messages with 3 or 5 fields.
///
/// Topic       Message Type                     Description
/// ----------- ----------------------------     ----------------------------------
/// $name/raw   ds_core_msgs::RawData            all raw bytes received from sensor
/// $name/depth ds_sensor_msgs::DepthPressure    successfully parsed messages
class DepthSensor : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(DepthSensor)

public:
  enum class PressureUnit
  {
    Psi = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_PSI,
    hPa = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_HPA,
    Bar = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_BAR,
    kPa = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_KPA,
    MPa = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_MPA,
    inHg = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_INHG,
    mmHg = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_MMHG,
    mH2O = ds_sensor_msgs::DepthPressure::UNIT_PRESSURE_MH2O
  };

  explicit DepthSensor();
  DepthSensor(int argc, char* argv[], const std::string& name);
  ~DepthSensor() override;
  DS_DISABLE_COPY(DepthSensor)

  /// @brief get the pressure unit used by the instrument.
  ///
  /// \return
  PressureUnit pressureUnit() const noexcept;

  /// @brief set the pressure unit used by the instrument.
  ///
  /// **Note:** This method only changes the pressure unit used in converting pressure to depth, it does not
  /// presently change the pressure value reported by the instrument.
  ///
  /// \param unit
  void setPressureUnit(PressureUnit unit);

  /// @brief set the pressure unit used by the instrument.
  ///
  /// **Note:** This method only changes the pressure unit used in converting pressure to depth, it does not
  /// presently change the pressure value reported by the instrument.
  ///
  /// Valid strings are:
  ///    - Psi
  ///    - hPa
  ///    - Bar
  ///    - kPa
  ///    - MPa
  ///    - inHg
  ///    - mmHg
  ///    - mH2O
  /// \param unit
  void setPressureUnit(const std::string& unit);
  /// @brief get the tare value applied to all pressure measurements
  ///
  /// \return
  double getTare() const noexcept;

  /// @brief Set the latitude for depth conversions.
  ///
  /// \param latitude
  void setLatitude(double latitude) noexcept;

  /// @brief Get the latitude used for depth conversions.
  ///
  /// \return
  double latitude() const noexcept;

  void push_depth_buffer(double depth);

protected:
  void setupParameters() override;
  void setupPublishers() override;
  void setupSubscriptions() override;
  void setupServices() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;
  bool set_depth_zero(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp);

  void _latitude_callback(const sensor_msgs::NavSatFixPtr& latlon);

  /// @brief Parse a message from the sensor and update the pressure field of the record
  ///
  /// This does *not* populate the entire DepthPressure message.  Quantities such as latitude, pressure unit,
  /// and tare value are required and are not part of the pressure record.
  ///
  /// Those will automagically get filled in later by the depth sensor class.  This is the method you need to fill
  /// in to get your depth sensor to parse.  Typically, this wraps a static method that makes unit testing easier.
  ///
  /// \param bytes
  /// \return
  virtual std::pair<bool, ds_sensor_msgs::DepthPressure> parse_message(const ds_core_msgs::RawData& bytes)=0;

private:
  std::unique_ptr<DepthSensorPrivate> d_ptr_;
};

}  // namespace ds_sensors

#endif //DS_SENSORS_DEPTHSENSOR_H
