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
// Created by Stefano Suman on 12/2/20.
//

#ifndef DS_SENSORS_SONARDYNE_USBL_H
#define DS_SENSORS_SONARDYNE_USBL_H

#include "ds_base/sensor_base.h"
#include "ds_core_msgs/RawData.h"
#include <sensor_msgs/NavSatFix.h>

namespace ds_sensors
{
class SonardyneUsbl : public ds_base::SensorBase
{

public:
  explicit SonardyneUsbl();
  SonardyneUsbl(int argc, char* argv[], const std::string& name);
  ~SonardyneUsbl() override;
  DS_DISABLE_COPY(SonardyneUsbl)

  static std::pair<bool, sensor_msgs::NavSatFix> parse_bytes(const ds_core_msgs::RawData& bytes);

protected:
  void setupPublishers() override;

  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  ros::Publisher navsat_pub_;
};
}
#endif  // DS_SENSORS_APS1540_H
