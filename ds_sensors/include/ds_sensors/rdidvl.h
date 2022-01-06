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
// file:  include/ds_sensors/rdidvl.h
#ifndef DS_SENSORS_RDIDVL_H
#define DS_SENSORS_RDIDVL_H

#include "ds_core_msgs/RawData.h"
#include "ds_base/sensor_base.h"
#include "ds_sensor_msgs/Dvl.h"
#include "ds_sensor_msgs/RdiPD0.h"
#include "ds_sensor_msgs/Ranges3D.h"
#include "../../src/ds_sensors/rdidvl_structs.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/date_time/gregorian/gregorian.hpp>

// Our sensors live in the ds_sensors namespace
namespace ds_sensors
{
struct RdiDvlPrivate;

class RdiDvl : public ds_base::SensorBase
{
  DS_DECLARE_PRIVATE(RdiDvl)

public:
  // Our constructor
  explicit RdiDvl();
  RdiDvl(int argc, char* argv[], const std::string& name);
  ~RdiDvl() override;

  DS_DISABLE_COPY(RdiDvl)

  ///*   TOP FUNCTIONS: receive incoming raw data, create messages, and publish
  static std::tuple<bool, ds_sensor_msgs::Dvl, ds_sensor_msgs::RdiPD0, ds_sensor_msgs::Ranges3D>
  parse_bytes(const ds_core_msgs::RawData& bytes, double beam_angle, bool phased_array);
  static bool parseHeaderID(const uint8_t* buffer, uint16_t offset, size_t recv_len, ds_sensor_msgs::RdiPD0* big_msg);
  static bool parseAddrID(const uint8_t* buffer, uint16_t offset, size_t len, ds_sensor_msgs::RdiPD0* big_msg);
  static void msg_to_dvl(ds_sensor_msgs::Dvl* dvldata, ds_sensor_msgs::RdiPD0* big_msg, double beam_angle, bool phased_array);
  static void msg_to_rng(ds_sensor_msgs::Ranges3D* rngdata, ds_sensor_msgs::RdiPD0* big_msg, double beam_angle, bool phased_array);

  ///*   MEMORY PARSERS: read from buffer into structs
  static std::pair<bool, rdidvl_structs::header> parseHeader(const uint8_t* raw, size_t recv_len);

  ///*   DATA CONVERSION PARSERS: generate PD0 from memory parsers
  static void fl_to_msg(const rdidvl_structs::fixedleader* fl, size_t len, ds_sensor_msgs::RdiPD0* big_msg);
  static void vl_to_msg(const rdidvl_structs::variableleader* vl, size_t len, ds_sensor_msgs::RdiPD0* big_msg);
  static void bt_to_msg(const rdidvl_structs::bottomtrack* bt, size_t len, ds_sensor_msgs::RdiPD0* big_msg);
  static void highres_to_msg(const rdidvl_structs::velocity_highres* highres, size_t len, ds_sensor_msgs::RdiPD0* big_msg);
  static void btr_to_msg(const rdidvl_structs::bottomtrack_range* btr, size_t len, ds_sensor_msgs::RdiPD0* big_msg);
  static void np_to_msg(const rdidvl_structs::nav_parameters* np, size_t len, ds_sensor_msgs::RdiPD0* big_msg);

  static bool checksum(uint16_t length, const uint8_t* buffer);

  static double seconds_from_epoch(boost::posix_time::ptime const& t);

protected:
  void setupConnections() override;
  void setupPublishers() override;
  void setupParameters() override;

  void parseReceivedBytes(const ds_core_msgs::RawData& bytes) override;

private:
  std::unique_ptr<RdiDvlPrivate> d_ptr_;
};

}  // end namespace ds_sensors
#endif  // DS_SENSORS_RDIDVL_H
