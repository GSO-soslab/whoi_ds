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
#include "ds_sensors/rdidvl.h"
#include "ds_sensors/rdidvl_header_match.h"

#include "rdidvl_private.h"

#include <sstream>

using namespace boost::posix_time;
using namespace boost::gregorian;

/// ds_sensors/src/ds_sensors/rdidvl.cpp
/// =============================================
/// DATE ------ WHO -------- WHAT ---------------
/// 12-15-17    J Vaccaro    Created and written

namespace ds_sensors
{
RdiDvl::RdiDvl() : ds_base::SensorBase(), d_ptr_(std::unique_ptr<RdiDvlPrivate>(new RdiDvlPrivate))
{
}

RdiDvl::RdiDvl(int argc, char* argv[], const std::string& name)
  : ds_base::SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<RdiDvlPrivate>(new RdiDvlPrivate))
{
}

RdiDvl::~RdiDvl() = default;

double RdiDvl::seconds_from_epoch(ptime const& t)
{
  boost::posix_time::ptime const EPOCH(date(1970, 1, 1));
  boost::posix_time::time_duration delta(t - EPOCH);
  return (delta.total_microseconds() / 1000000.0);
}

std::tuple<bool, ds_sensor_msgs::Dvl, ds_sensor_msgs::RdiPD0, ds_sensor_msgs::Ranges3D>
RdiDvl::parse_bytes(const ds_core_msgs::RawData& bytes, double beam_angle, bool phased_array)
/// creates parsed dvldata and Rdipd0 messages. Do not use returned messages when success=false
{
  auto buffer = bytes.data.data();
  bool success = false;
  auto msg = ds_sensor_msgs::Dvl{};
  auto pd0 = ds_sensor_msgs::RdiPD0{};
  auto rng = ds_sensor_msgs::Ranges3D{};

  if (bytes.data.size() < 2)
  {
    // Check for zero or small size
    ROS_ERROR_STREAM("BYTES DATA SIZE LESS THAN 2");
    return std::make_tuple(false, msg, pd0, rng);
  }
  // fill in DVL type field so that subsequent processing can use it
  if (phased_array) {
    pd0.dvl_type = ds_sensor_msgs::RdiPD0::DVL_TYPE_PHASED_ARRAY;
  } else {
    pd0.dvl_type = ds_sensor_msgs::RdiPD0::DVL_TYPE_PISTON;
  }

  // Create parsed pd0 message. Only fails when bad header read,
  // checksum fails, or unknown binary data header.
  // INFO already fed out.
  success = parseHeaderID(buffer, 0, bytes.data.size(), &pd0);
  if (success)
  {
    msg_to_dvl(&msg, &pd0, beam_angle, phased_array);  // Create parsed dvl message
    msg_to_rng(&rng, &pd0, beam_angle, phased_array);
  }

  msg.header = bytes.header;
  msg.ds_header.io_time = bytes.ds_header.io_time;
  return std::make_tuple(success, msg, pd0, rng);
}

bool RdiDvl::parseHeaderID(const uint8_t* buffer, uint16_t offset, size_t recv_len, ds_sensor_msgs::RdiPD0* big_msg)
/// uses header ID to call specified parsing functions. Calls parseAddrID for sub-functions
{
  bool ok = true;
  uint32_t headerid = (buffer[offset] << 0) + (buffer[offset + 1] << 8);
  if (headerid == 0x7f7f && offset == 0)
  {
    // CASE HEADER
    auto hdr = ds_sensors::rdidvl_structs::header();
    std::tie(ok, hdr) = RdiDvl::parseHeader(buffer, recv_len);
    auto length = hdr.bytes;
    if (hdr.bytes > recv_len) {
      ROS_ERROR_STREAM("Header reports we should have received more bytes than we did, dropping...");
      return false;
    }
    if (!ok) {
      ROS_ERROR_STREAM("Unable to parse DVL header, returning...");
      return false;
    }
    if (hdr.numdata == 0)
    {
      ROS_INFO_STREAM("No data fields: " << hdr.numdata);
      ok = false;
    }
    for (int i = 0; i < hdr.numdata; i++)
    {
      if (hdr.addr[i] > length)
      {
        ROS_INFO_STREAM("Address: " << hdr.addr[i] << " outside of buffer length: " << length);
        ok = false;
      }
      if (i > 10)
      {
        ROS_INFO_STREAM("# " << hdr.numdata << " memory addresses in header > 10");  // 10 was chosen arbitrarily. 3 is
                                                                                     // typical for sentry
        ok = false;
      }
      if (!ok) {
        return false;
      } else {
        uint16_t end_addr;
        if (i < hdr.numdata-1) {
          end_addr = hdr.addr[i+1];
        } else {
          end_addr = hdr.bytes;
        }
        uint16_t fieldlen = end_addr - hdr.addr[i];
        ok = RdiDvl::parseAddrID(buffer, hdr.addr[i], fieldlen, big_msg);
      }
    }
  }
  else
  {
    ROS_INFO_STREAM("Header ID not recognized: " << headerid);
    return false;
  }
  return ok;
}

/// Parse a single PD0 subfield.  Note that the length of PD0 datagrams isn't very well-defined; individual
/// parsers may need to check length at various points before proceeding.
/// \param buffer The buffer containing the PD0 to parse from
/// \param offset The offset of the subfield
/// \param len The length of the subfield (until the start of the next one, or the checksum)
/// \param big_msg The big PD0 message we're parsing into
/// \return True if the parse succeeded
bool RdiDvl::parseAddrID(const uint8_t* buffer, uint16_t offset, size_t len, ds_sensor_msgs::RdiPD0* big_msg)
/// uses header ID to call specified parsing functions. Calls self for sub-functions
{
  bool ok = true;
  uint32_t headerid = (buffer[offset] << 0) + (buffer[offset + 1] << 8);
  //ROS_ERROR_STREAM("Parsing packet type " <<std::hex <<headerid <<std::dec <<" len: " <<len);
  //  try to memory parse based on header ID. If successful, then populate pd0 message
  if (headerid == RDI_PD0_ID_FIXED_LEADER && offset != 0)
  {
    // CASE FIXED LEADER
    auto fl = reinterpret_cast<const rdidvl_structs::fixedleader*>(&buffer[offset]);
    RdiDvl::fl_to_msg(fl, len, big_msg);
  }
  else if (headerid == RDI_PD0_ID_VARIABLE_LEADER && offset != 0)
  {
    // CASE VARIABLE LEADER
    auto vl = reinterpret_cast<const rdidvl_structs::variableleader*>(&buffer[offset]);
    RdiDvl::vl_to_msg(vl, len, big_msg);
  }
  else if (headerid == RDI_PD0_ID_BOTTOMTRACK && offset != 0)
  {
    // CASE BOTTOM TRACK
    auto bt = reinterpret_cast<const rdidvl_structs::bottomtrack*>(&buffer[offset]);
    RdiDvl::bt_to_msg(bt, len, big_msg);
  }
  else if (headerid == RDI_PD0_ID_VELOCITY_HIGHRES && offset != 0) {
    auto vel_highres = reinterpret_cast<const rdidvl_structs::velocity_highres*>(&buffer[offset]);
    RdiDvl::highres_to_msg(vel_highres, len, big_msg);
  }
  else if (headerid == RDI_PD0_ID_BOTTOMTRACK_RANGE && offset != 0) {
    auto btr = reinterpret_cast<const rdidvl_structs::bottomtrack_range*>(&buffer[offset]);
    RdiDvl::btr_to_msg(btr, len, big_msg);
  }
  else if (headerid == RDI_PD0_ID_NAV_PARAMS && offset != 0) {
    auto np = reinterpret_cast<const rdidvl_structs::nav_parameters*>(&buffer[offset]);
    RdiDvl::np_to_msg(np, len, big_msg);
  }
  else
  {
    ROS_INFO_STREAM("Address ID not recognized: " << headerid);
    return false;
  }
  return ok;
}

void RdiDvl::msg_to_rng(ds_sensor_msgs::Ranges3D* rngdata, ds_sensor_msgs::RdiPD0* big_msg, double beam_angle, bool phased_array)
{
  rngdata->ranges.resize(4);
  double range[4], xy_range[4];

  if (phased_array) {
    rngdata->soundspeed_correction_type = rngdata->SOUNDSPEED_CORRECTION_PHASEDARRAYDVL;
  } else {
    rngdata->soundspeed_correction_type = rngdata->SOUNDSPEED_CORRECTION_NORMAL;
  }

  // Z component for each beam is - the altitude
  for (int i = 0; i < 4; ++i)
  {
    rngdata->ranges[i].range_quality = big_msg->correlation[i];
    double beam_range = big_msg->range[i];
    if (big_msg->btrange_valid) {
      beam_range = big_msg->btrange_raw_range[i];
    }

    if (beam_range == 0)
      rngdata->ranges[i].range_validity = ds_sensor_msgs::Range3D::RANGE_INDETERMINANT;
    else
      rngdata->ranges[i].range_validity = ds_sensor_msgs::Range3D::RANGE_VALID;
    rngdata->ranges[i].range.point.z = -beam_range;
    range[i] = beam_range / cos(beam_angle);
    xy_range[i] = range[i] * sin(beam_angle);
  }

  rngdata->ranges[0].range.point.x = -xy_range[0];
  rngdata->ranges[0].range.point.y = 0.0;
  rngdata->ranges[1].range.point.x = +xy_range[1];
  rngdata->ranges[1].range.point.y = 0.0;
  rngdata->ranges[2].range.point.x = 0.0;
  rngdata->ranges[2].range.point.y = +xy_range[2];
  rngdata->ranges[3].range.point.x = 0.0;
  rngdata->ranges[3].range.point.y = -xy_range[3];
}

void RdiDvl::msg_to_dvl(ds_sensor_msgs::Dvl* dvldata, ds_sensor_msgs::RdiPD0* big_msg, double beam_angle, bool phased_array)
/// extracts/converts PD0 message into slimmer DVLData message
{
  dvldata->dvl_time = big_msg->dvl_time;
  //    dvldata->ds_header.io_time = ros::Time::now();
  //    dvldata->ds_header.source_uuid = "b6c54489-38a0-5f50-a60a-fd8d76219cae";

  if (phased_array) {
    dvldata->dvl_type = ds_sensor_msgs::Dvl::DVL_TYPE_PHASED_ARRAY;
  } else {
    dvldata->dvl_type = ds_sensor_msgs::Dvl::DVL_TYPE_PISTON;
  }

  dvldata->speed_sound = big_msg->sound_vel;
  dvldata->num_good_beams = big_msg->good_beams;

  for (int i = 0; i < 9; i++) {
    dvldata->velocity_covar[i] = -1;
  }

  dvldata->velocity_mode = dvldata->DVL_MODE_BOTTOM;  // Bottom tracking mode
  dvldata->coordinate_mode = big_msg->coord_mode;

  // 2018-02-23 SS - change order and signs to conform to the dvl convention we adopted
  // (native instrument frame, stationary bottom, moving dvl)
  for (int i = 0; i < 4; i++)
  {
    // This geometry will be needed for visualizing in RVIZ
    // the beams are a 30-60-90 triangle, with the sqrt(3)/2 size on z
    // (filled out here) and the 0.5 side in the reference frame of the
    // DVL (filled out further below)
    // In the mean time, we zero all the other components.
    dvldata->beam_unit_vec[i].x = 0;
    dvldata->beam_unit_vec[i].y = 0;
    dvldata->beam_unit_vec[i].z = -cos(beam_angle);

    // New versions of the firmware have better fields to grab some stuff from.
    // We'd like to make those available in the message and fall back on something
    // sensible if required
    if (big_msg->highres_valid) {
      // for some reason, the high-res field seems to be in
      // dvl-moving mode.
      dvldata->raw_velocity[i] = big_msg->highres_bt_velocity[i];
    } else {
      dvldata->raw_velocity[i] = -big_msg->velocity[i];
    }

    if (big_msg->navp_valid) {
      dvldata->raw_velocity_covar[i] = big_msg->navp_bottomtrack_stddev[i];
    } else {
      dvldata->raw_velocity_covar[i] = -1;                      // sigma = 0.3 m/s
    }

    dvldata->beam_quality[i] = big_msg->correlation[i];       // include beam correlation statistics
    if (big_msg->btrange_valid) {
      dvldata->range[i] = big_msg->btrange_raw_range[i];
    } else {
      dvldata->range[i] = big_msg->range[i] / cos(beam_angle);  // DVL reports ALTITUDE, not RANGE. so send RANGE
    }
    dvldata->range_covar[i] = -1;                             // sigma unknown
  }

  // fill in the horizontal component for each beam in
  // the INSTRUMENT frame
  dvldata->beam_unit_vec[0].x = -sin(beam_angle);
  dvldata->beam_unit_vec[1].x = sin(beam_angle);
  dvldata->beam_unit_vec[2].y = sin(beam_angle);
  dvldata->beam_unit_vec[3].y = -sin(beam_angle);

  if (dvldata->coordinate_mode == dvldata->DVL_COORD_INSTRUMENT)
  {
    dvldata->velocity.x = dvldata->raw_velocity[0];
    dvldata->velocity.y = dvldata->raw_velocity[1];
    dvldata->velocity.z = dvldata->raw_velocity[2];
  }

  dvldata->altitude = big_msg->altitude_sum / big_msg->good_beams;
  dvldata->speed_gnd = big_msg->speed_gnd;
  dvldata->course_gnd = big_msg->course_gnd;
}

///*-----------------------------------------------------------------------------*///
///*   MEMORY PARSERS: read from buffer into structs                             *///
///*-----------------------------------------------------------------------------*///
std::pair<bool, rdidvl_structs::header> RdiDvl::parseHeader(const uint8_t* raw, size_t recv_len)
/// memory-parses buffer into header
{
  bool ok = true;
  auto hdr = reinterpret_cast<const rdidvl_structs::header*>(raw);
  if (hdr->numdata > 10)
  {
    ROS_ERROR_STREAM("Header result failure: too many data: " << hdr->numdata);
    return { false, *hdr};
  }
  if (hdr->bytes > recv_len)
  {
    ROS_ERROR_STREAM(
        "Header result failure: Parsed length too long: " << hdr->bytes);  // Too long length chosen arbitrarily
    return { false, *hdr};
  }
  if (!RdiDvl::checksum(hdr->bytes, raw))
  {
    ROS_ERROR_STREAM("Header result failure: Checksum failed");
    ok = false;
  }
  return { ok, *hdr };
}

///*-----------------------------------------------------------------------------*///
///*   DATA CONVERSION PARSERS: generate PD0 from memory parsers                 *///
///*-----------------------------------------------------------------------------*///
void RdiDvl::fl_to_msg(const rdidvl_structs::fixedleader* fl, size_t len, ds_sensor_msgs::RdiPD0* big_msg)
/// extracts/converts data into PD0 message from memory-parsed fixedleader
{
  //        Distances [m]; Velocities [m/s]; Angles [deg]
  big_msg->fw_ver = fl->fw_ver;
  big_msg->fw_rev = fl->fw_rev;
  int frequencies[8] = {75, 150, 300, 600, 1200, 2400, -1, -1};
  big_msg->config_khz = frequencies[fl->config0 & 0x07];
  // page 41 of the Pioneer DVL Guide July 2016 gives more precise
  // carrier frequencies for only the 300 and 600 kHz Pioneer systems.
  // We'll need these later though when we do the precision timing stuff.
  int carriers[8] = {75000, 150000, 307200, 614400, 1200000, 2400000, -1, -1};
  big_msg->carrier_frequency_hz = carriers[fl->config0 & 0x07];
  big_msg->config_convex = (fl->config0 & 0x08) != 0;
  big_msg->config_sensornum = 1 + (fl->config0 >> 4 & 0xF);
  big_msg->config_xdcr = (fl->config0 & 0x40);
  big_msg->config_up = (fl->config0 & 0x80);

  int beam_angles[4] = {15, 20, 30, -1};
  big_msg->config_beamangle = beam_angles[fl->config1 & 0x03];

  big_msg->config_janus = fl->config1 >> 4;
  big_msg->real_sim = fl->real_sim;  // 0=real is default
  big_msg->lag = fl->lag;

  big_msg->beams = fl->beams;
  big_msg->cells = (fl->cells < 128 ? fl->cells : 128);  // must be less than 128

  big_msg->pings = fl->pings;
  big_msg->cell_depth = fl->cell_depth;
  big_msg->blank = fl->blank;

  big_msg->signal_proc = fl->signal_proc;  // Always =1 //Profiling mode
  big_msg->min_thresh = fl->min_thresh;
  big_msg->code_reps = fl->code_reps;
  big_msg->min_good_pings = fl->min_good_pings;
  big_msg->good_thresh = fl->good_thresh;
  big_msg->ping_interval =
      ros::Duration(fl->ping_int_min * 60 + fl->ping_int_sec + fl->ping_int_hun * 0.01);  // Time between ping intervals
  big_msg->coord_mode = static_cast<uint8_t>((fl->coord_trans >> 3) & 0x03);            // xxx00xxx = no transformation
  big_msg->coord_tilts = (fl->coord_trans & 0x04);
  big_msg->coord_3beam = (fl->coord_trans & 0x02) >> 1;
  big_msg->coord_binmapping = (fl->coord_trans & 0x01);
  big_msg->hdng_align = fl->hdng_align * 0.01;  // ea-command
  big_msg->hdng_bias = fl->hdng_bias * 0.01;    // eb-command
  big_msg->sensor_src = fl->sensor_src;         // ez-command
  big_msg->sensor_avail = fl->sensor_avail;     // same as sensor_src pattern
  big_msg->bin1_dist = fl->bin1_dist;           // cm
  big_msg->xmit_pulse_len = fl->xmit_pulse_len;
  big_msg->avg_start = fl->avg_start;
  big_msg->avg_end = fl->avg_end;
  big_msg->avg_false_thresh = fl->avg_false_thresh;
  big_msg->trans_lag_dist = fl->trans_lag_dist;
  for (int i = 0; i < 8; i++)
  {
    big_msg->serial_num_cpu[i] = fl->serial_num_cpu[i];
  }
  // These may not be valid according to Gene Terray's adcpupk.m
  big_msg->wb_cmd = fl->wb_cmd;
  big_msg->power = fl->power;
  big_msg->serial_num = fl->serial_num;
  big_msg->beam_angle = fl->beam_angle;
}
void RdiDvl::vl_to_msg(const rdidvl_structs::variableleader* vl, size_t len, ds_sensor_msgs::RdiPD0* big_msg)
/// extracts/converts data into PD0 message from memory-parsed variableleader
{
  //    Units: Hdg, Pitch, Roll [deg],  P [Pa],  dt [s],  Xdepth [m],  Cs [m/s],  S [ppt]
  big_msg->ensemble_num = (vl->ensemble_num | vl->ensemble_num_msb << 16);
  big_msg->BIT = vl->bit_result;
  big_msg->rtc_time =
      ros::Time(vl->rtc_day * 86400 + vl->rtc_hour * 3600 + vl->rtc_minute * 60 + vl->rtc_second + vl->rtc_hundredth * 0.01);
  big_msg->rtc_year = vl->rtc_year;
  big_msg->rtc_month = vl->rtc_month;
  big_msg->rtc_day = vl->rtc_day;
  big_msg->rtc_hour = vl->rtc_hour;
  big_msg->rtc_minute = vl->rtc_minute;
  big_msg->rtc_second = vl->rtc_second;
  big_msg->rtc_hundredth = vl->rtc_hundredth;
  big_msg->dvl_time = vl->rtc_hour * 3600 + vl->rtc_minute * 60 + vl->rtc_second + vl->rtc_hundredth * 0.01;
  // SS - using the full y2k-compliant datetime as dvl time
  // short unsigned int y2k_year = vl.rtc_century_y2k * 100 + vl.rtc_year_y2k;
  // boost::posix_time::ptime pt{boost::gregorian::date{y2k_year, vl.rtc_month_y2k, vl.rtc_day_y2k},
  // boost::posix_time::time_duration{vl.rtc_hour_y2k, vl.rtc_minute_y2k, vl.rtc_second_y2k}};
  //ROS_INFO_STREAM("RTC month:" << (int)(vl->rtc_month));
  //ROS_INFO_STREAM("RTC day:" << (int)(vl->rtc_day));
  //ROS_INFO_STREAM("RTC day:" << (int)(vl->rtc_hundredth_y2k));
  short unsigned int y2k_year = 2000 + vl->rtc_year;
  try
  {
    boost::posix_time::ptime pt{ date{ y2k_year, vl->rtc_month, vl->rtc_day },
                                 time_duration{ vl->rtc_hour, vl->rtc_minute, vl->rtc_second } };
    big_msg->dvl_time = seconds_from_epoch(pt) + vl->rtc_hundredth / 100.0;
  }
  catch (...)
  {
    big_msg->dvl_time = 0;
  }
  big_msg->error_demod1 = (vl->bit_result & 0x10) != 0;
  big_msg->error_demod0 = (vl->bit_result & 0x08) != 0;
  big_msg->error_timingcard = (vl->bit_result & 0x02) != 0;
  big_msg->sound_vel = vl->sound_vel;  // m/s
  big_msg->depth = vl->depth * 0.1;    // meters

  big_msg->heading = vl->heading * 0.01;          // positive degrees
  big_msg->pitch = vl->pitch * 0.01;              // pos/neg degrees
  big_msg->roll = vl->roll * 0.01;                // pos/neg degrees
  big_msg->salinity = vl->salinity;               // parts per thousand
  big_msg->temperature = vl->temperature * 0.01;  // pos/neg degrees C
  big_msg->mpt_wait = ros::Duration(vl->mpt_minute * 60 + vl->mpt_second + vl->mpt_hundredth * 0.01);
  big_msg->heading_std = vl->heading_std;
  big_msg->pitch_std = vl->pitch_std * 0.1;
  big_msg->roll_std = vl->roll_std * 0.1;

  for (int i = 0; i < 4; i++)
    big_msg->adc[i] = vl->adc[i];

  big_msg->error_busexception = (vl->error_status_word[0] & 0x01) != 0;  // error_status_word0
  big_msg->error_address = (vl->error_status_word[0] & 0x02) != 0;
  big_msg->error_illegalinstruction = (vl->error_status_word[0] & 0x04) != 0;
  big_msg->error_zerodivide = (vl->error_status_word[0] & 0x08) != 0;
  big_msg->error_emulator = (vl->error_status_word[0] & 0x10) != 0;
  big_msg->error_unassigned = (vl->error_status_word[0] & 0x20) != 0;
  big_msg->error_watchdogrestart = (vl->error_status_word[0] & 0x40) != 0;
  big_msg->error_batterysaver = (vl->error_status_word[0] & 0x80) != 0;
  big_msg->error_pinging = (vl->error_status_word[1] & 0x01) != 0;  // error_status_word1
  big_msg->error_coldwakeup = (vl->error_status_word[1] & 0x40) != 0;
  big_msg->error_unknwakeup = (vl->error_status_word[1] & 0x80) != 0;
  big_msg->error_clockread = (vl->error_status_word[2] & 0x01) != 0;  // error_status_word2
  big_msg->error_unexpectedalarm = (vl->error_status_word[2] & 0x02) != 0;
  big_msg->error_clockforward = (vl->error_status_word[2] & 0x04) != 0;
  big_msg->error_clockbackward = (vl->error_status_word[2] & 0x08) != 0;
  big_msg->error_powerfail = (vl->error_status_word[3] & 0x08) != 0;  // error_status_word3
  big_msg->error_interrupt4dsp = (vl->error_status_word[3] & 0x10) != 0;
  big_msg->error_interrupt5uart = (vl->error_status_word[3] & 0x20) != 0;
  big_msg->error_interrupt6clock = (vl->error_status_word[3] & 0x40) != 0;
  big_msg->error_interrupt7 = (vl->error_status_word[3] & 0x80) != 0;

  big_msg->pressure = vl->pressure * 10;                    // Pa
  big_msg->pressure_variance = vl->pressure_variance * 10;  // Pa
  // Y2K time.  Not present in Workhorse Navigators
  if (len < 65) {
    return;
  }
  big_msg->y2k_time = ros::Time(vl->rtc_day_y2k * 86400 + vl->rtc_hour_y2k * 3600 + vl->rtc_minute_y2k * 60 +
                                vl->rtc_second_y2k + vl->rtc_hundredth_y2k * 0.01);

  // pioner status fields
  if (len < 77) {
    return;
  }

  big_msg->leak_valid = true;
  big_msg->leak_status = vl->leak_status;
  big_msg->leakA_raw = vl->leak_a_count;
  big_msg->leakB_raw = vl->leak_b_count;
  big_msg->leakA_detected = (big_msg->leak_status & 0x01) ? 1 : 0;
  big_msg->leakA_open = (big_msg->leak_status & 0x02) ? 1 : 0;
  big_msg->leakB_detected = (big_msg->leak_status & 0x04) ? 1 : 0;
  big_msg->leakB_open = (big_msg->leak_status & 0x08) ? 1 : 0;

  if (vl->tx_voltage == 0xffff) {
    big_msg->tx_voltage = std::numeric_limits<float>::quiet_NaN();
  } else {
    big_msg->tx_voltage = static_cast<float>(vl->tx_voltage) * 0.001;
  }

  if (vl->tx_current == 0xffff) {
    big_msg->tx_current = std::numeric_limits<float>::quiet_NaN();
  } else {
    big_msg->tx_current = static_cast<float>(vl->tx_current) * 0.001;
  }

  big_msg->transducer_impedence = static_cast<float>(vl->transducer_impedence) * 0.001;
}
void RdiDvl::bt_to_msg(const rdidvl_structs::bottomtrack* bt, size_t len, ds_sensor_msgs::RdiPD0* big_msg)
/// extracts/converts data into PD0 message from memory-parsed bottomtrack
{
  //    All ranges are in m, other distances in cm, velocities in m/s, intensities in counts
  big_msg->pings_per_ensemble = bt->pings_per_ensemble;
  big_msg->delay = bt->delay;
  big_msg->corr_mag_min = bt->corr_mag_min;
  big_msg->eval_amp_min = bt->eval_amp_min;
  big_msg->percent_good_min = bt->percent_good_min;
  big_msg->mode = bt->mode;
  big_msg->err_vel_max = bt->err_vel_max;

  for (int i = 0; i < 4; i++)
  {
    big_msg->range[i] = (bt->range[i] + (bt->range_msb[i] << 16)) * 0.01;
    if (big_msg->range[i] > 0.25)
      big_msg->good_beams += 1;
    big_msg->altitude_sum += big_msg->range[i];
    if (abs(bt->velocity[i]) != 32768)
      big_msg->velocity[i] = bt->velocity[i] * 0.001;

    big_msg->correlation[i] = bt->correlation[i];
    big_msg->eval_amp[i] = bt->eval_amp[i];
    big_msg->percent_good[i] = bt->percent_good[i];
    big_msg->ref_velocity[i] = bt->ref_velocity[i];
    big_msg->ref_correlation[i] = bt->ref_correlation[i];
    big_msg->ref_intensity[i] = bt->ref_intensity[i];
    big_msg->ref_percent_good[i] = bt->ref_percent_good[i];
    big_msg->rssi_amp[i] = bt->rssi_amp[i];  // Receiver Signal Strength indicator
  }

  big_msg->ref_min =
      bt->ref_min * 10;  // minimum layer size, near boundary, and far boundary for BT water-reference layer
  big_msg->ref_near = bt->ref_near * 10;
  big_msg->ref_far = bt->ref_far * 10;
  big_msg->depth_max = bt->depth_max * 10;  // dm to cm
  big_msg->gain = bt->gain;

  big_msg->speed_gnd = sqrt(big_msg->velocity[0] * big_msg->velocity[0] + big_msg->velocity[1] * big_msg->velocity[1]);
  big_msg->course_gnd = atan2(big_msg->velocity[0], big_msg->velocity[1]) * 180.0 / 3.14159;
}

void RdiDvl::highres_to_msg(const rdidvl_structs::velocity_highres* highres, size_t len, ds_sensor_msgs::RdiPD0* big_msg) {
  big_msg->highres_valid = true;
  for (size_t i=0; i<4; i++) {
    if (highres->bt_velocity[i] != std::numeric_limits<int32_t>::min()) {
      big_msg->highres_bt_velocity[i] = static_cast<float>(highres->bt_velocity[i]) * 1.0e-5;
    }
    big_msg->highres_bt_dmg[i] = static_cast<float>(highres->bt_dmg[i]) * 1.0e-5;

    if (highres->bt_dmg[i] != std::numeric_limits<int32_t>::min()) {
      big_msg->highres_bt_dmg[i] = static_cast<float>(highres->bt_dmg[i]) * 1.0e-5;
    }

    if (highres->wm_velocity[i] != std::numeric_limits<int32_t>::min()) {
      big_msg->highres_wm_velocity[i] = static_cast<float>(highres->wm_velocity[i]) * 1.0e-5;
    }

    if (highres->wm_dmg[i] != std::numeric_limits<int32_t>::min()) {
      big_msg->highres_wm_dmg[i] = static_cast<float>(highres->wm_dmg[i]) * 1.0e-5;
    }
  }
  big_msg->highres_sound_vel = highres->sound_speed * 10e-7;
}

void RdiDvl::btr_to_msg(const rdidvl_structs::bottomtrack_range* btr, size_t len, ds_sensor_msgs::RdiPD0* big_msg) {
  big_msg->btrange_valid = true;
  big_msg->btrange_slant_range = static_cast<float>(btr->slant_range)*1.0e-4;
  big_msg->btrange_axis_delta_range = static_cast<float>(btr->axis_delta_range)*1.0e-4;
  big_msg->btrange_vertical_range = static_cast<float>(btr->vertical_range)*1.0e-4;
  big_msg->btrange_pct_good_4beam = btr->pct_good_4beam;
  big_msg->btrange_pct_good_beam12 = btr->pct_good_beam12;
  big_msg->btrange_pct_good_beam34 = btr->pct_good_beam34;

  for (size_t i=0; i<4; i++ ){
    big_msg->btrange_raw_range[i] = static_cast<float>(btr->raw_range[i]) * 1.0e-4;
    big_msg->btrange_max_filter[i] = btr->raw_max_bt_filter[i];
    big_msg->btrange_max_amp[i] = btr->raw_max_bt_amp[i];
  }

}

void RdiDvl::np_to_msg(const rdidvl_structs::nav_parameters* np, size_t len, ds_sensor_msgs::RdiPD0* big_msg) {

  double carrier_cycles = 1.0 / static_cast<double>(big_msg->carrier_frequency_hz);

  big_msg->navp_valid = true;
  for (size_t i=0; i<4; i++) {
    big_msg->navp_time_to_bottom[i] = static_cast<float>(np->time_to_bottom[i])*carrier_cycles*8;
    big_msg->navp_bottomtrack_stddev[i] = static_cast<float>(np->bottom_track_stddev[i])*1.0e-3;
    big_msg->navp_bottomtrack_valid_time[i] = static_cast<float>(np->bottomtrack_time_of_validity[i])*1.0e-6;

    big_msg->navp_time_to_watermass[i] = static_cast<float>(np->time_to_watermass[i])*carrier_cycles*8;
    big_msg->navp_watertrack_stddev[i] = static_cast<float>(np->watertrack_stddev[i])*1.0e-3;
    big_msg->navp_watertrack_valid_time[i] = static_cast<float>(np->watertrack_time_of_validity[i])*1.0e-6;
  }

  big_msg->navp_watertrack_range = static_cast<float>(np->range_to_watermass)*carrier_cycles;
  if (np->shallow_operation) {
    big_msg->navp_bottomtrack_shallow_mode = big_msg->NAVP_BT_MODE_SHALLOW;
  } else {
    big_msg->navp_bottomtrack_shallow_mode = big_msg->NAVP_BT_MODE_DEEP;
  }
}


///*-----------------------------------------------------------------------------*///
///*   CHECKSUM                                                                  *///
///*-----------------------------------------------------------------------------*///
bool RdiDvl::checksum(uint16_t length, const uint8_t* buffer)
//    Buffer up until checksum is length bytes long.
{
  uint16_t sum = 0x00;
  for (int i = 0; i < length; i++)
  {
    sum += buffer[i];
  }
  auto chksum = reinterpret_cast<const uint16_t*>(&buffer[length]);
  return (sum == *chksum);
}

void RdiDvl::setupConnections()
{
  SensorBase::setupConnections();

  // auto foo = connection("instrument");
  // auto connection = reinterpret_cast<ds_asio::DsSerial*>(foo);
  // if (connection != nullptr)
  //{
  //  connection->set_matcher(match_header_length_2());
  //}
  // else
  //  ROS_DEBUG("DVL CONNECTION IS NOT SERIAL");
}

void RdiDvl::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(RdiDvl);
  auto nh = nodeHandle();
  d->dvl_pub_ = nh.advertise<ds_sensor_msgs::Dvl>(ros::this_node::getName() + "/dvl", 10);
  d->pd0_pub_ = nh.advertise<ds_sensor_msgs::RdiPD0>(ros::this_node::getName() + "/pd0", 10);
  d->ranges_pub_ = nh.advertise<ds_sensor_msgs::Ranges3D>(ros::this_node::getName() + "/ranges", 1);
}

void RdiDvl::setupParameters()
{
  SensorBase::setupParameters();

  DS_D(RdiDvl);

  d->beam_angle = ros::param::param<double>("~/beam_angle_deg", 30) * M_PI / 180.0;
  d->phased_array = ros::param::param<bool>("~/phased_array", false);
}
///*-----------------------------------------------------------------------------*///
///*   TOP FUNCTIONS: receive incoming raw data, create messages, and publish    *///
///*-----------------------------------------------------------------------------*///
void RdiDvl::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
/// TOP LEVEL calls parsing functions. If successful, publishes and saves messages
{
  DS_D(RdiDvl);

  // Create our result variables.
  auto ok = false;
  auto msg = ds_sensor_msgs::Dvl{};
  auto pd0 = ds_sensor_msgs::RdiPD0{};
  auto rng = ds_sensor_msgs::Ranges3D{};

  std::tie(ok, msg, pd0, rng) = RdiDvl::parse_bytes(bytes, d->beam_angle, d->phased_array);

  if (!ok)
  {
    return;
  }

  // update all timestamps based on our rules
  FILL_SENSOR_HDR(msg, msg.header.stamp, msg.ds_header.io_time);
  FILL_SENSOR_HDR(pd0, msg.header.stamp, msg.ds_header.io_time);
  FILL_SENSOR_HDR(rng, msg.header.stamp, msg.ds_header.io_time);
  for (int i = 0; i < rng.ranges.size(); ++i)
  {
    rng.ranges[i].range.header.frame_id = frameId();
    rng.ranges[i].range.header.stamp = msg.header.stamp;
  }

  d->dvl_pub_.publish(msg);
  d->pd0_pub_.publish(pd0);
  d->ranges_pub_.publish(rng);
  updateTimestamp("dvl", msg.header.stamp);
  updateTimestamp("pd0", msg.header.stamp);
  updateTimestamp("rng", msg.header.stamp);
}

}  // end namespace ds_sensors
