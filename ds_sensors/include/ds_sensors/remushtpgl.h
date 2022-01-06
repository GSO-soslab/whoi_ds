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
// Created by ivaughn on 3/18/18.
//

#ifndef PROJECT_REMUS_HTPGL_H
#define PROJECT_REMUS_HTPGL_H

#include "ds_base/sensor_base.h"
#include "ds_hotel_msgs/HTP.h"
#include "ds_hotel_msgs/HTPGL.h"

namespace ds_sensors
{
struct RemusHtpglPrivate;

/// @brief Sensor class for Remus 107811 Environmental Monitor Board
///
/// Based on PCB Rev. A, Firmware v.1.0.0, dated 2016 Oct 18
///
/// Publishes an HTP message so that this board can function as a
/// drop-in replacement for the existing HTP sensor
class RemusHtpgl : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(RemusHtpgl);

public:
  explicit RemusHtpgl();
  RemusHtpgl(int argc, char* argv[], const std::string& name);
  ~RemusHtpgl() override;
  DS_DISABLE_COPY(RemusHtpgl);

  ///@brief Parse a message from the Remus HTPGL message
  ///
  ///
  /// Returns a std::pair<bool, ds_hotel_msgs::HTPGL>.  The boolean indicates whether parsing was
  /// successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \return
  static std::pair<bool, ds_hotel_msgs::HTPGL> parse_bytes(const ds_core_msgs::RawData& bytes);

protected:
  void setupPublishers() override;
  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  std::unique_ptr<RemusHtpglPrivate> d_ptr_;
};
}

#endif  // PROJECT_REMUS_HTPGL_H
