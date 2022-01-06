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

#ifndef DS_SENSORS_MICROSTRAIN_STRUCTS_H
#define DS_SENSORS_MICROSTRAIN_STRUCTS_H

#include <stdint.h>
#include <sys/types.h>

namespace ds_sensors {
namespace _microstrain_mip {

const uint8_t SYNC1 = 0x75;
const uint8_t SYNC2 = 0x65;

/// \brief Compute a Fletcher checksum as described in the microstrain manual
/// See Section 6.4 on page 140 of 3dm-gx5-25_dcp_manual_8500-0065_0.pdf
/// \param buf The buffer to take the checksum over
/// \param buflen The length of the buffer to take the checksum over
/// \return The 16-bit Fletcher checksum
uint16_t mip_checksum(const uint8_t* const buf, size_t buflen) {
  uint8_t byte1=0;
  uint8_t byte2=0;
  for (size_t i=0; i<buflen; i++) {
    byte1 += buf[i];
    byte2 += byte1;
  }

  // checksum described is big endian; convert to host.
  return (static_cast<uint16_t>(byte1) << 8) + static_cast<uint16_t>(byte2);
}

struct MipPacketHeader {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t descriptorSetByte;
  uint8_t payloadLen; // does not include header and checksum
} __attribute__((packed));

/// \brief The number of bytes to add to the playload length field to
/// get the actual packet length
const size_t BYTES_NOT_IN_LEN = sizeof(MipPacketHeader) + 2;

struct MipFieldHeader {
  uint8_t length; // total length, including this header
  uint8_t descriptor;
} __attribute__((packed));

namespace DescriptorSets {
const uint8_t BASE = 0x01;
const uint8_t IMU=0x80;
const uint8_t CMD_3DM=0x0C;
}

namespace BaseFields {
const uint8_t GPS_TIME_UPDATE = 0x72;
}

namespace Cmd3dmFields {
const uint8_t GetImuRate = 0x0b;
const uint8_t ImuMessageFormat = 0x08;
const uint8_t EstimationFilterMessageFormat = 0x0a;
}

namespace ImuFormatCmdFunctions {
const uint8_t USE_NEW = 0x01;
const uint8_t READ_BACK_CURRENT = 0x02;
const uint8_t SAVE_CURRENT_AS_STARTUP = 0x03;
const uint8_t LOAD_SAVED_STARTUP = 0x04;
const uint8_t RESET_TO_DEFAULT=0x05;
}

struct ImuFormatCmd {
  uint8_t function;
  uint8_t num_descriptors;
} __attribute__((packed));

struct ImuFormatCmdData {
  uint8_t descriptor;
  uint16_t decimation;
} __attribute__((packed));

namespace GpsTimeUpdateFunctions {
const uint8_t USE_NEW = 0x01;
const uint8_t READ_BACK_CURRENT = 0x02;
const uint8_t USE_NEW_NOACK = 0x06;
}

namespace GpsTimeSelector {
const uint8_t GPS_WEEK = 0x01;
const uint8_t GPS_SECONDS = 0x02;
}

struct GpsTimeUpdate {
  uint8_t function;
  uint8_t field_selector;
  uint32_t new_value;
} __attribute__((packed));

// all of these are in DescriptorSets::IMU
namespace ImuFields {

/// \brief Vector3. Temperature-compensated linear accelerations.  Instrument-frame.
const uint8_t ACCEL=0x04;

/// \brief Vector3. Temperature-compensated gyro angular rates.  Radians/sec.  Instrument-frame.
const uint8_t GYRO=0x05;

/// \brief Vector3. Instantaneous magnetometer reading.  Temperature-compensated, Gauss.  Instrument-frame.
const uint8_t MAGNETOMETER=0x06;

/// \brief Scalar. Ambient pressure.  Temperature-compensated, Millibar.
const uint8_t PRESSURE=0x17;

/// \brief Vector3. Time integral of angular rate, in radians.  Instrument-cordinate system.
const uint8_t DELTATHETA=0x07;

/// \brief Vector3. Time integral of velocity, in g*seconds.  Instrument-coordinate frame.
const uint8_t DELTAVELOCITY=0x08;

const uint8_t CF_QUATERNION=0x0a;

// skipping a bunch of possible fields

/// \brief GpsTime.  Time of this sample, (hopefully) synced to GPS time.
const uint8_t GPSTIME=0x12;
}; // namespace ImuFields

struct Vector3 {
  float x;
  float y;
  float z;
} __attribute__((packed));

struct Scalar {
  float value;
} __attribute__((packed));

struct Quaternion {
  float qw;
  float qx;
  float qy;
  float qz;
} __attribute((packed));

/// \brief Unit conversion: m/s^2 per G
/// From page 110 of 3dm-gx5 manual
const float MPS2_PER_G = 9.80665f;
const float RAD2DEG = 180.0/M_PI;

struct GpsTime {
  double time_of_week;
  uint16_t gps_week;
  uint16_t flags;
};

struct _RawGpsTime {
  uint64_t time_of_week;
  uint16_t gps_week;
  uint16_t flags;
} __attribute((packed));

namespace GpsTimeFlags {
const uint16_t PPS_GOOD = 0x0001;
const uint16_t GPS_REFRESHED = 0x0002;
const uint16_t GPS_INITIALIZED = 0x0004;
}

Vector3 decode_vector3(const uint8_t* buf) {
  Vector3 ret;

  const uint32_t* ptr = reinterpret_cast<const uint32_t*>(buf);

  uint32_t tmp = be32toh(*ptr++);
  ret.x = *reinterpret_cast<float*>(&tmp);

  tmp = be32toh(*ptr++);
  ret.y = *reinterpret_cast<float*>(&tmp);

  tmp = be32toh(*ptr++);
  ret.z = *reinterpret_cast<float*>(&tmp);

  return ret;
}

Scalar decode_scalar(const uint8_t* buf) {
  Scalar ret;

  const uint32_t* ptr = reinterpret_cast<const uint32_t*>(buf);

  uint32_t tmp = be32toh(*ptr++);
  ret.value = *reinterpret_cast<float*>(&tmp);

  return ret;
}

Quaternion decode_quaternion(const uint8_t* buf) {
  Quaternion ret;

  const uint32_t* ptr = reinterpret_cast<const uint32_t*>(buf);

  uint32_t tmp = be32toh(*ptr++);
  ret.qw = *reinterpret_cast<float*>(&tmp);

  tmp = be32toh(*ptr++);
  ret.qx = *reinterpret_cast<float*>(&tmp);

  tmp = be32toh(*ptr++);
  ret.qy = *reinterpret_cast<float*>(&tmp);

  tmp = be32toh(*ptr++);
  ret.qz = *reinterpret_cast<float*>(&tmp);

  return ret;
}

GpsTime decode_gpstime(const uint8_t* buf) {
  GpsTime ret;
  auto raw = reinterpret_cast<const _RawGpsTime*>(buf);

  uint64_t dtmp = be64toh(raw->time_of_week);
  ret.time_of_week = *reinterpret_cast<double*>(&dtmp);

  ret.gps_week = be16toh(raw->gps_week);
  ret.flags = be16toh(raw->flags);

  return ret;
}

} // namespace _microstrain_mip
} // namespace ds_sensors

#endif //DS_SENSORS_MICROSTRAIN_STRUCTS_H
