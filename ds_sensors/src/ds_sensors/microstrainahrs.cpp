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
// Created by ivaughn on 11/15/19.
//

#include <ds_sensors/microstrainahrs.h>
#include "microstrainahrs_private.h"
#include "microstrain_structs.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>
#include <limits>
#include <endian.h>

namespace ds_sensors {

MicrostrainAhrs::MicrostrainAhrs() : SensorBase(), d_ptr_(new MicrostrainAhrsPrivate) {
}

MicrostrainAhrs::MicrostrainAhrs(int argc, char *argv[], const std::string &name)
    : SensorBase(argc, argv, name), d_ptr_(new MicrostrainAhrsPrivate) {
}

MicrostrainAhrs::~MicrostrainAhrs() = default;

void MicrostrainAhrs::setup() {
  SensorBase::setup();
  sendConfigCmd();
}

void MicrostrainAhrs::setupParameters() {
  SensorBase::setupParameters();

  DS_D(MicrostrainAhrs);
  d->samplerate = ros::param::param<double>("~/samplerate", 100);
  d->downsample = ros::param::param<int>("~/downsample", 10);
  d->skipped = 0;
  d->max_time_delta = ros::param::param<double>("~/max_time_error", 5.0);
}

void MicrostrainAhrs::setupPublishers() {
  DS_D(MicrostrainAhrs);
  SensorBase::setupPublishers();

  auto nh = nodeHandle();
  d->fullimu_pub = nh.advertise<ds_sensor_msgs::MemsImu>(ros::this_node::getName() + "/fullimu", 10);
  d->memsimu_pub = nh.advertise<ds_sensor_msgs::MemsImu>(ros::this_node::getName() + "/memsimu", 10);
  d->imu_pub = nh.advertise<sensor_msgs::Imu>(ros::this_node::getName() + "/imu", 10);
  d->gyro_pub = nh.advertise<ds_sensor_msgs::Gyro>(ros::this_node::getName() +"/gyro", 10);
}

void MicrostrainAhrs::setupTimers() {
  DS_D(MicrostrainAhrs);
  SensorBase::setupTimers();

  auto nh = nodeHandle();

  // ROS timers suck.  We need a timer that expires just after the top of the second.  Let's
  auto now = ros::WallTime::now();

  // wait until 2.050 seconds from the start of the current second,
  // so we send no earlier than ~50ms from the start of the next second
  d->time_generator = nh.createWallTimer(ros::WallDuration(2.05 - static_cast<double>(now.nsec)*1.0e-9),
      &MicrostrainAhrs::timeGenStartCallback, this, true);
  d->time_init = false;
}

void MicrostrainAhrs::timeGenStartCallback(const ros::WallTimerEvent& evt) {
  DS_D(MicrostrainAhrs);
  auto nh = nodeHandle();

  sendTimeMsg(evt.current_real);

  d->time_init = true;
  d->time_generator = nh.createWallTimer(ros::WallDuration(1.0), &MicrostrainAhrs::timeGenCallback, this);
}

void MicrostrainAhrs::timeGenCallback(const ros::WallTimerEvent& evt) {
  DS_D(MicrostrainAhrs);

  sendTimeMsg(evt.current_real);
}

const uint32_t SECS_PER_DAY = 24*3600;
const uint32_t SECS_PER_WEEK = 7*SECS_PER_DAY;

void MicrostrainAhrs::sendTimeMsg(const ros::WallTime &now) {
  DS_D(MicrostrainAhrs);

  // find the start of GPS time in the epoch
  static uint32_t day_offset = boost::gregorian::days(boost::gregorian::date(1980, 1, 6)
      - boost::gregorian::date(1970, 1, 1)).days();

  // subtract off the days corresponding to the start of the GPS epoch
  uint32_t gps_now = now.sec - day_offset * SECS_PER_DAY;
  uint32_t seconds = gps_now % SECS_PER_WEEK;
  uint32_t total_weeks = gps_now / SECS_PER_WEEK;
  uint32_t weeks = total_weeks % 1024; // limit to max week value
  //ROS_INFO_STREAM("Sending time message at time: " <<now <<", week=" <<weeks <<", sec=" <<seconds);

  uint8_t buf[255];
  // prepare some buffers
  uint8_t* ptr = buf;
  auto pkt_hdr = reinterpret_cast<_microstrain_mip::MipPacketHeader*>(ptr);
  ptr += sizeof(_microstrain_mip::MipPacketHeader);

  uint8_t* payload_start = ptr;

  auto field_hdr = reinterpret_cast<_microstrain_mip::MipFieldHeader*>(ptr);
  ptr += sizeof(_microstrain_mip::MipFieldHeader);

  auto time_msg = reinterpret_cast<_microstrain_mip::GpsTimeUpdate*>(ptr);
  ptr += sizeof(_microstrain_mip::GpsTimeUpdate);

  // fill in the Cmd header
  time_msg->function = _microstrain_mip::GpsTimeUpdateFunctions::USE_NEW_NOACK;
  time_msg->field_selector = _microstrain_mip::GpsTimeSelector::GPS_WEEK;
  time_msg->new_value = htobe32(weeks);

  // fill in field header
  field_hdr->descriptor = _microstrain_mip::BaseFields::GPS_TIME_UPDATE;
  field_hdr->length = ptr - payload_start;

  // fill in packet header
  pkt_hdr->sync1 = _microstrain_mip::SYNC1;
  pkt_hdr->sync2 = _microstrain_mip::SYNC2;
  pkt_hdr->descriptorSetByte = _microstrain_mip::DescriptorSets::BASE;
  pkt_hdr->payloadLen = ptr - payload_start;

  // compute the checksum
  size_t len = ptr - buf;
  *reinterpret_cast<uint16_t*>(ptr) = htobe16(_microstrain_mip::mip_checksum(buf, len));
  len += 2;

  // actually send
  this->connection("instrument")->send(std::string(reinterpret_cast<const char*>(buf), len));

  // we're going to have to send a second message with the timestamp.  Let's re-use this one as pretty much the same.
  time_msg->field_selector = _microstrain_mip::GpsTimeSelector::GPS_SECONDS;
  time_msg->new_value = htobe32(seconds);
  *reinterpret_cast<uint16_t*>(ptr) = htobe16(_microstrain_mip::mip_checksum(buf, len-2));

  // actually send seconds
  this->connection("instrument")->send(std::string(reinterpret_cast<const char*>(buf), len));
}

void MicrostrainAhrs::parseReceivedBytes(const ds_core_msgs::RawData &bytes) {
  DS_D(MicrostrainAhrs);

  // (hopefully) parse our packet
  auto fullimu = parseMipPacket(bytes);
  if (!fullimu) {
    return;
  }

  // finish the full message
  fullimu->header.frame_id = frameId();

  // check constraints ont he time
  ros::Duration diff = fullimu->header.stamp - fullimu->ds_header.io_time;
  if (fabs(diff.toSec()) > d->max_time_delta) {
    fullimu->header.stamp = fullimu->ds_header.io_time;
  }

  // publish the full message
  d->fullimu_pub.publish(*fullimu);

  if (d->skipped >= d->downsample-1) {
    d->skipped = 0;

    // convert our messages & publish
    d->memsimu_pub.publish(*fullimu);
    d->gyro_pub.publish(convertToGyro(*fullimu));
    d->imu_pub.publish(convertToImu(*fullimu));
  } else {
    d->skipped++;
  }
}
boost::optional<ds_sensor_msgs::MemsImu> MicrostrainAhrs::parseMipPacket(const ds_core_msgs::RawData &bytes) {
  // initial sanity checsk
  if (bytes.data.size() < _microstrain_mip::BYTES_NOT_IN_LEN) {
    ROS_ERROR_STREAM("Microstrain packet has too few bytes to decode header...");
    return boost::optional<ds_sensor_msgs::MemsImu>();
  }

  // ptr points to the start of the next set of bytes to be consumed
  const uint8_t *ptr = bytes.data.data();
  size_t pktlen = bytes.data.size();
  auto pkt_hdr = reinterpret_cast<const _microstrain_mip::MipPacketHeader *>(ptr);
  ptr += sizeof(_microstrain_mip::MipPacketHeader);
  pktlen -= sizeof(_microstrain_mip::MipPacketHeader);

  // sanity-check fields
  if (pkt_hdr->sync1 != _microstrain_mip::SYNC1 || pkt_hdr->sync2 != _microstrain_mip::SYNC2) {
    ROS_WARN_STREAM("Microstrain binary stream SYNC LOST!");
    return boost::optional<ds_sensor_msgs::MemsImu>();
  }

  // sanity-check header size
  if (bytes.data.size() < pkt_hdr->payloadLen + _microstrain_mip::BYTES_NOT_IN_LEN) {
    ROS_ERROR_STREAM("Microstrain packet buffer has fewer bytes than specified in header, dropping");
    return boost::optional<ds_sensor_msgs::MemsImu>();
  }

  // compute the checksum over all bytes except the checksum
  size_t checksum_offset = sizeof(_microstrain_mip::MipPacketHeader) + pkt_hdr->payloadLen;
  uint16_t chksum_calc = _microstrain_mip::mip_checksum(bytes.data.data(), checksum_offset);
  uint16_t chksum_recv = be16toh(*reinterpret_cast<const uint16_t *>(bytes.data.data() + checksum_offset));
  if (chksum_recv != chksum_calc) {
    ROS_ERROR_STREAM("Microstrain checksum mismatch!" << std::hex << chksum_calc
                                                      << " recv=" << std::hex << chksum_recv << std::dec);
    return boost::optional<ds_sensor_msgs::MemsImu>();
  }

  // packet is now (probably) valid.  Start parsing fields based on the descriptor.
  switch (pkt_hdr->descriptorSetByte) {
    case _microstrain_mip::DescriptorSets::IMU:
      return parseImuData(ptr, pkt_hdr->payloadLen, bytes.ds_header.io_time);
    default:
      ROS_WARN_STREAM(
          "Microstrain: Unknown descriptor byte " << static_cast<unsigned int>(pkt_hdr->descriptorSetByte));
      return boost::optional<ds_sensor_msgs::MemsImu>();
  }
}
ds_sensor_msgs::MemsImu MicrostrainAhrs::parseImuData(const uint8_t* buf, size_t buflen, const ros::Time& iot) {
  ds_sensor_msgs::MemsImu imu;
  imu.header.stamp = iot;
  imu.ds_header.io_time = iot;
  imu.linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
  imu.linear_acceleration.y = std::numeric_limits<double>::quiet_NaN();
  imu.linear_acceleration.z = std::numeric_limits<double>::quiet_NaN();
  imu.linear_delta.x = std::numeric_limits<double>::quiet_NaN();
  imu.linear_delta.y = std::numeric_limits<double>::quiet_NaN();
  imu.linear_delta.z = std::numeric_limits<double>::quiet_NaN();
  imu.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  imu.angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
  imu.angular_velocity.z = std::numeric_limits<double>::quiet_NaN();
  imu.angular_delta.x = std::numeric_limits<double>::quiet_NaN();
  imu.angular_delta.y = std::numeric_limits<double>::quiet_NaN();
  imu.angular_delta.z = std::numeric_limits<double>::quiet_NaN();
  imu.magnetometer.x = std::numeric_limits<double>::quiet_NaN();
  imu.magnetometer.y = std::numeric_limits<double>::quiet_NaN();
  imu.magnetometer.z = std::numeric_limits<double>::quiet_NaN();
  imu.orientation.x = std::numeric_limits<double>::quiet_NaN();
  imu.orientation.y = std::numeric_limits<double>::quiet_NaN();
  imu.orientation.z = std::numeric_limits<double>::quiet_NaN();
  imu.orientation.w = std::numeric_limits<double>::quiet_NaN();

  while (buflen > 0) {
    auto field_hdr = reinterpret_cast<const _microstrain_mip::MipFieldHeader*>(buf);
    const uint8_t* field_data = buf + sizeof(_microstrain_mip::MipFieldHeader);

    switch (field_hdr->descriptor) {
      case _microstrain_mip::ImuFields::ACCEL:
      {
        auto data = _microstrain_mip::decode_vector3(field_data);
        //ROS_INFO_STREAM("ACCEL      : " <<data.x * _microstrain_mip::MPS2_PER_G
        //                               <<", " <<data.y * _microstrain_mip::MPS2_PER_G
        //                               <<", " <<data.z * _microstrain_mip::MPS2_PER_G
        //                               <<" m/s^2");
        imu.linear_acceleration.x = data.x * _microstrain_mip::MPS2_PER_G;
        imu.linear_acceleration.y = data.y * _microstrain_mip::MPS2_PER_G;
        imu.linear_acceleration.z = data.z * _microstrain_mip::MPS2_PER_G;
      }
        break;
      case _microstrain_mip::ImuFields::GYRO:
      {
        auto data = _microstrain_mip::decode_vector3(field_data);
        //ROS_INFO_STREAM("GYRO       : " <<data.x * _microstrain_mip::RAD2DEG
        //                               <<", " <<data.y * _microstrain_mip::RAD2DEG
        //                               <<", " <<data.z * _microstrain_mip::RAD2DEG
        //                               <<" deg/s");
        imu.angular_velocity.x = data.x;
        imu.angular_velocity.y = data.y;
        imu.angular_velocity.z = data.z;
      }
        break;
      case _microstrain_mip::ImuFields::MAGNETOMETER:
      {
        auto data = _microstrain_mip::decode_vector3(field_data);
        //ROS_INFO_STREAM("MAGGIES    : " <<data.x
        //                                <<", " <<data.y
        //                                <<", " <<data.z
        //                                <<" Gauss");
        imu.magnetometer.x = data.x;
        imu.magnetometer.y = data.y;
        imu.magnetometer.z = data.z;
      }
        break;
      case _microstrain_mip::ImuFields::PRESSURE:
      {
        auto data = _microstrain_mip::decode_scalar(field_data);
        //ROS_INFO_STREAM("PRESSURE   : " <<data.value <<" millibar");
      }
        break;
      case _microstrain_mip::ImuFields::DELTATHETA:
      {
        auto data = _microstrain_mip::decode_vector3(field_data);
        //ROS_INFO_STREAM("DELTA_THETA: " <<data.x * _microstrain_mip::RAD2DEG
        //                               <<", " <<data.y * _microstrain_mip::RAD2DEG
        //                               <<", " <<data.z * _microstrain_mip::RAD2DEG
        //                               <<" deg");
        imu.angular_delta.x = data.x;
        imu.angular_delta.y = data.y;
        imu.angular_delta.z = data.z;
      }
        break;
      case _microstrain_mip::ImuFields::DELTAVELOCITY:
      {
        auto data = _microstrain_mip::decode_vector3(field_data);
        //ROS_INFO_STREAM("DELTA_VEL   : " <<data.x * _microstrain_mip::MPS2_PER_G
        //                                <<", " <<data.y * _microstrain_mip::MPS2_PER_G
        //                                <<", " <<data.z * _microstrain_mip::MPS2_PER_G
        //                                <<" m/s");
        imu.linear_delta.x = data.x * _microstrain_mip::MPS2_PER_G;
        imu.linear_delta.y = data.y * _microstrain_mip::MPS2_PER_G;
        imu.linear_delta.z = data.z * _microstrain_mip::MPS2_PER_G;
      }
        break;
      case _microstrain_mip::ImuFields::CF_QUATERNION: {
        auto data = _microstrain_mip::decode_quaternion(field_data);
        //ROS_INFO_STREAM("CF_QUAT    : " <<data.qw <<", " <<data.qx <<", " <<data.qy <<", " <<data.qz);
        imu.orientation.x = data.qx;
        imu.orientation.y = data.qy;
        imu.orientation.z = data.qz;
        imu.orientation.w = data.qw;
      }
        break;
      case _microstrain_mip::ImuFields::GPSTIME:
      {
        auto data = _microstrain_mip::decode_gpstime(field_data);
        //ROS_INFO_STREAM("GPS_TIME   : " <<"WK=" <<data.gps_week <<" TIME: " <<data.time_of_week);
        imu.gps_week = data.gps_week;
        imu.gps_time_of_week = data.time_of_week;

        // get our time.  GPS times roll over every 19ish years, so pick the time closest to now
        if (data.flags & _microstrain_mip::GpsTimeFlags::GPS_INITIALIZED && iot.sec != 0 && iot.nsec != 0) {
          // compute the time relative to the GPS start time
          boost::gregorian::date gps_start(1980, 1, 6); // start of all GPS time
          boost::posix_time::ptime base_time(gps_start + boost::gregorian::days(7*data.gps_week),
                                   boost::posix_time::microseconds(data.time_of_week*1.0e6));

          // base_time is now off by an integer number of GPS weeks.  Find that integer!
          boost::posix_time::ptime now = iot.toBoost(); // use IO time
          boost::posix_time::time_duration dt = now - base_time;
          double dt_weeks = static_cast<double>(dt.total_seconds()) / (3600.0*24.0*7.0);
          int epochs = round(dt_weeks / 1024.0);

          // add the number of epochs back in
          boost::posix_time::ptime ptime = base_time + boost::gregorian::days(1024*7*epochs);

          // stuff back into timestamp
          imu.header.stamp = ros::Time::fromBoost(ptime);
        }

      }
        break;
      default:
        ROS_ERROR_STREAM("Unknown IMUDATA field ID= "
                             <<std::hex <<static_cast<unsigned int>(field_hdr->descriptor));
    }

    buf += field_hdr->length;
    buflen -= field_hdr->length;
  }

  return imu;
}

void MicrostrainAhrs::sendConfigCmd() {
  DS_D(MicrostrainAhrs);
  uint8_t buf[255];

  // pick the sample rate closest to our desired.  The base rate of 250Hz should really be read
  // from the sensor, but for now we'll hard-code it.
  uint16_t rate = round(1000 / d->samplerate);
  ROS_INFO_STREAM("Setting sample rate to " <<rate);

  // prepare some buffers
  uint8_t* ptr = buf;
  auto pkt_hdr = reinterpret_cast<_microstrain_mip::MipPacketHeader*>(ptr);
  ptr += sizeof(_microstrain_mip::MipPacketHeader);

  uint8_t* payload_start = ptr;

  auto field_hdr = reinterpret_cast<_microstrain_mip::MipFieldHeader*>(ptr);
  ptr += sizeof(_microstrain_mip::MipFieldHeader);

  auto cmd_hdr = reinterpret_cast<_microstrain_mip::ImuFormatCmd*>(ptr);
  ptr += sizeof(_microstrain_mip::ImuFormatCmd);

  auto cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  int n_descriptors = 0;
  // fill out the packet backwards; start by adding commands
  cmd_data->descriptor = _microstrain_mip::ImuFields::ACCEL;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  cmd_data->descriptor = _microstrain_mip::ImuFields::GYRO;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  cmd_data->descriptor = _microstrain_mip::ImuFields::MAGNETOMETER;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  cmd_data->descriptor = _microstrain_mip::ImuFields::DELTATHETA;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  cmd_data->descriptor = _microstrain_mip::ImuFields::DELTAVELOCITY;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  cmd_data->descriptor = _microstrain_mip::ImuFields::GPSTIME;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  cmd_data->descriptor = _microstrain_mip::ImuFields::CF_QUATERNION;
  cmd_data->decimation = htobe16(rate);
  ptr += sizeof(_microstrain_mip::ImuFormatCmdData); n_descriptors++;
  cmd_data = reinterpret_cast<_microstrain_mip::ImuFormatCmdData*>(ptr);

  // fill in the Cmd header
  cmd_hdr->function = _microstrain_mip::ImuFormatCmdFunctions::USE_NEW;
  cmd_hdr->num_descriptors = n_descriptors;

  // fill in field header
  field_hdr->descriptor = _microstrain_mip::Cmd3dmFields::ImuMessageFormat;
  field_hdr->length = ptr - payload_start;

  // fill in packet header
  pkt_hdr->sync1 = _microstrain_mip::SYNC1;
  pkt_hdr->sync2 = _microstrain_mip::SYNC2;
  pkt_hdr->descriptorSetByte = _microstrain_mip::DescriptorSets::CMD_3DM;
  pkt_hdr->payloadLen = ptr - payload_start;

  // compute the checksum
  size_t len = ptr - buf;
  *reinterpret_cast<uint16_t*>(ptr) = htobe16(_microstrain_mip::mip_checksum(buf, len));
  len += 2;

  // actually send
  ROS_INFO_STREAM("Sending new microstrain config command!");
  std::string cmdstr = std::string(reinterpret_cast<const char*>(buf), len);
  std::stringstream sout;
  for (size_t i=0; i<cmdstr.size(); i++) {
    sout << std::hex <<std::setfill('0') <<std::setw(2) <<static_cast<unsigned int>(cmdstr[i]) <<" ";
  }
  ROS_INFO_STREAM("LEN: " <<cmdstr.size() <<" CMD: " <<sout.str());
  this->connection("instrument")->send(cmdstr);
}

ds_sensor_msgs::Gyro MicrostrainAhrs::convertToGyro(const ds_sensor_msgs::MemsImu& fullimu) {
  ds_sensor_msgs::Gyro ret;

  tf2::Quaternion quat_tf;
  tf2::fromMsg(fullimu.orientation, quat_tf);
  tf2::Matrix3x3 mat_tf(quat_tf);

  ret.header = fullimu.header;
  ret.ds_header = fullimu.ds_header;

  mat_tf.getEulerYPR(ret.heading, ret.pitch, ret.roll);

  ret.heading_covar = -1;
  ret.pitch_covar = -1;
  ret.roll_covar = -1;

  return ret;
}

sensor_msgs::Imu MicrostrainAhrs::convertToImu(const ds_sensor_msgs::MemsImu& fullimu) {
  sensor_msgs::Imu ret;

  ret.header = fullimu.header;

  ret.orientation = fullimu.orientation;
  ret.angular_velocity = fullimu.angular_velocity;
  ret.linear_acceleration = fullimu.linear_acceleration;

  return ret;
}

} // namespace ds_sensors
