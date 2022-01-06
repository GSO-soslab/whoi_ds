//
// Created by ivaughn on 8/23/19.
//

#include "phinsbin_private.h"
#include "phinstdv3.h"
#include "endian.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/date_time/posix_time/posix_time_types.hpp>

/// \brief Convert a big-endian 32-bit number to a float using the host's endianness
float be32to_hfloat(uint32_t in) {
  uint32_t tmp = be32toh(in);
  return *reinterpret_cast<float*>(&tmp);
}

/// \brief Convert a big-endian 64-bit number to a double using the host's endianness
double be64to_hdouble(uint64_t in) {
  uint64_t tmp = be64toh(in);
  return *reinterpret_cast<double*>(&tmp);
}

namespace ds_sensors {

PhinsBin::PhinsBin() : ds_base::SensorBase(), d_ptr_(new PhinsBinPrivate()) {
  // do nothing
}

PhinsBin::PhinsBin(int argc, char *argv[], const std::string &name)
    : ds_base::SensorBase(argc, argv, name), d_ptr_(new PhinsBinPrivate()) {
  // do nothing
}

PhinsBin::~PhinsBin() = default;

void PhinsBin::setupParameters() {
  ds_base::SensorBase::setupParameters();

  DS_D(PhinsBin);

  // If the phins timestamp doesn't match the CPU to within 0.5 seconds, assume the
  // phins clock is wrong and use the I/O timestamp instead
  ros::param::param<double>("~/max_clock_offset", d->max_clock_offset, 0.5);
  ros::param::param<int>("~/downsample", d->downsample, 1);
  d->skipped = 0;
}

void PhinsBin::setupPublishers() {
  ds_base::SensorBase::setupPublishers();

  DS_D(PhinsBin);

  auto nh = nodeHandle();

  ROS_WARN_STREAM("Setting up publishers!");
  d->phinsbin_pub_ = nh.advertise<ds_sensor_msgs::PhinsStdbin3>(ros::this_node::getName() + "/phinsbin", 10, false);
  d->fullimu_pub_ = nh.advertise<ds_sensor_msgs::PhinsStdbin3>(ros::this_node::getName() + "/fullimu", 25, false);
  d->phinsstatus_pub_ = nh.advertise<ds_sensor_msgs::PhinsStatus>(ros::this_node::getName() + "/phins_status", 10, false);
  d->gyro_pub_ = nh.advertise<ds_sensor_msgs::Gyro>(ros::this_node::getName() + "/gyro", 10, false);
  d->compass_pub_ = nh.advertise<ds_sensor_msgs::Compass>(ros::this_node::getName() + "/compass", 10, false);
  d->imu_pub_ = nh.advertise<sensor_msgs::Imu>(ros::this_node::getName() + "/imu", 10, false);
}

void PhinsBin::checkProcessStatus(const ros::TimerEvent& event) {
  DS_D(PhinsBin);

  auto status = statusMessage();
  status.status = ds_core_msgs::Status::STATUS_GOOD;

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::ALIGNMENT) {

    ROS_WARN("PHINS is still aligning");
    status.status = ds_core_msgs::Status::STATUS_WARN;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::FINE_ALIGNMENT) {
    // this seems to happen on the bench
    //ROS_INFO("PHINS is still in fine alignment; try moving it around");
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::HRP_INVALID) {
    ROS_ERROR("PHINS reports invalid Heading/Pitch/Roll");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::DEGRADED_MODE) {
    ROS_ERROR("PHINS operating in degraded mode!");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::FAILURE_MODE) {
    ROS_ERROR("PHINS reports it has failed!");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::FOG_ANOMALY) {
    ROS_ERROR("PHINS Reports a fiber-optic gyro anomaly");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::ACC_ANOMALY) {
    ROS_ERROR("PHINS Reports an accelerometer anomaly");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::TEMPERATURE_ERR) {
    ROS_ERROR("PHINS Reports an issue reading temperature values");
    status.status = ds_core_msgs::Status::STATUS_WARN;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::CPU_OVERLOAD) {
    ROS_ERROR("PHINS Reports CPU overload");
    status.status = ds_core_msgs::Status::STATUS_WARN;
  }

  if (d->user_status[0] & _phinsstdbin3::status::user::w1::DYNAMIC_EXCEEDED
      || d->user_status[0] & _phinsstdbin3::status::user::w1::SPEED_SATURATION
      || d->user_status[0] & _phinsstdbin3::status::user::w1::ALTITUDE_SATURATION) {
    ROS_ERROR("PHINS has saturated and is probably locked up.");
    status.status = ds_core_msgs::Status::STATUS_ERROR;
  }

  publishStatus(status);
}

void PhinsBin::parseReceivedBytes(const ds_core_msgs::RawData &bytes) {
  DS_D(PhinsBin);

  // parse the message
  auto res = parseStdbin3Msg(bytes);
  if (!res.first) {
    return;
  }

  ds_sensor_msgs::PhinsStdbin3 msg = res.second;

  // fill in the UUID
  auto sensor_uuid = uuid();
  std::copy(sensor_uuid.begin(), sensor_uuid.end(), msg.ds_header.source_uuid.begin());


  // If the timestamps are wildly different-- probably because the phins clock isn't set-- slam
  // it back into place using hte I/O time
  ros::Duration dt = msg.ds_header.io_time - msg.header.stamp;
  if (fabs(dt.toSec()) > d->max_clock_offset) {
    ROS_WARN_STREAM_ONCE("Phins clock differs from CPU clock by " <<dt.toSec() <<" seconds; using I/O times");
    msg.header.stamp = msg.ds_header.io_time;
  }

  // fill in the sensor's frame ID using the standard machinery
  msg.header.frame_id = frameId();

  // publish our various messages
  // only publish the raw phinsbin message at FULL rate
  d->fullimu_pub_.publish(msg);

  if (d->skipped >= d->downsample - 1) {
      d->skipped = 0;

      d->phinsbin_pub_.publish(msg);
      d->update_status(msg);

      auto phins_status = makePhinsStatusMsg(msg);
      d->phinsstatus_pub_.publish(phins_status);

      auto gyro = makeGyroMsg(msg);
      d->gyro_pub_.publish(gyro);

      auto compass = makeCompassMsg(msg);
      d->compass_pub_.publish(compass);

      auto imu = makeImuMsg(msg);
      d->imu_pub_.publish(imu);
  } else {
      d->skipped++;
  }
}

std::pair<bool, ds_sensor_msgs::PhinsStdbin3> PhinsBin::parseStdbin3Msg(const ds_core_msgs::RawData& bytes) {
  ds_sensor_msgs::PhinsStdbin3 msg;

  auto hdr = reinterpret_cast<const _phinsstdbin3::Header*>(bytes.data.data());
  const uint8_t* ptr = bytes.data.data();
  ptr += sizeof(_phinsstdbin3::Header);

  // let's do some basic validity checking
  if (bytes.data.size() < sizeof(_phinsstdbin3::Header)) {
    ROS_ERROR_STREAM("Received fewer bytes than binary header, skipping ("
    <<bytes.data.size() <<" vs " <<sizeof(_phinsstdbin3::Header) <<")");
    return std::make_pair(false, ds_sensor_msgs::PhinsStdbin3());
  }
  if (hdr->version != _phinsstdbin3::VERSION3) {
    ROS_FATAL_STREAM("This node only works with binary version 3.  Please upgrade the firmware on your phins");
    return std::make_pair(false, ds_sensor_msgs::PhinsStdbin3());
  }

  ssize_t total_size = be16toh(hdr->total_size);

  if (bytes.data.size() < total_size) {
    ROS_ERROR_STREAM("Received fewer bytes than specified in header, skipping..."
    <<"(" <<bytes.data.size() <<" vs. " <<total_size <<")");
    return std::make_pair(false, ds_sensor_msgs::PhinsStdbin3());
  }

  if (hdr->header1 != _phinsstdbin3::HEADER1 || hdr->header2 != _phinsstdbin3::HEADER2) {
    ROS_ERROR_STREAM("Phins binary datagram has wrong sync values!, skipping");
    return std::make_pair(false, ds_sensor_msgs::PhinsStdbin3());
  }

  // check the checksum
  uint32_t calc_checksum = 0;
  const uint8_t* chkptr = bytes.data.data();
  for (size_t i=0; i<total_size - sizeof(uint32_t); i++) {
    calc_checksum += *chkptr++;
  }
  const uint32_t recv_checksum = be32toh(*reinterpret_cast<const uint32_t*>(bytes.data.data() + total_size - sizeof(uint32_t)));
  //ROS_INFO_STREAM("Got Phins STDBIN3 message, len=" <<total_size <<" chksum: " <<recv_checksum <<"|" <<calc_checksum);
  if (recv_checksum != calc_checksum) {
    ROS_ERROR_STREAM("Phins STDBIN3 Checksum mismatch! len="
    <<total_size <<" chksum: " <<recv_checksum <<"|" <<calc_checksum);
    return std::make_pair(false, ds_sensor_msgs::PhinsStdbin3());
  }

  // fill in the ds_header first
  msg.ds_header.io_time = bytes.ds_header.io_time;


  // parse the header first
  msg.nav_fields = be32toh(hdr->nav_data_mask);
  msg.extended_nav_fields = be32toh(hdr->nav_extended_data_mask);
  msg.external_data_fields = be32toh(hdr->external_data_mask);

  msg.counter = be32toh(hdr->counter);
  uint32_t vtime = be32toh(hdr->validity_time);
  msg.nav_validity_time = static_cast<double>(vtime)*100e-6; // 100us steps
  // boost's time stuff wants all ints because it sucks.  So make that happen.
  int32_t navt_secs = static_cast<int32_t>(vtime / 10000);
  int32_t navt_usecs = static_cast<int32_t>((vtime % 10000) * 100);

  if (_phinsstdbin3::NAV_BLOCK_ATTITUDEHEADING & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AttitudeHeading*>(ptr);

    msg.heading = be32to_hfloat(field->heading);
    msg.roll = be32to_hfloat(field->roll);
    msg.pitch = be32to_hfloat(field->pitch);

    ptr += sizeof(_phinsstdbin3::AttitudeHeading);
  }
  if (_phinsstdbin3::NAV_BLOCK_ATTITUDEHEADING_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AttitudeHeadingStddev*>(ptr);

    msg.heading_stddev = be32to_hfloat(field->heading_stddev);
    msg.roll_stddev = be32to_hfloat(field->roll_stddev);
    msg.pitch_stddev = be32to_hfloat(field->pitch_stddev);

    ptr += sizeof(_phinsstdbin3::AttitudeHeadingStddev);
  }
  if (_phinsstdbin3::NAV_BLOCK_RT_HEAVESURGESWAY & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::rtHeaveSurgeSway*>(ptr);

    msg.rt_heave_XVnH[0] = be32to_hfloat(field->rtSurge_xv1h);
    msg.rt_heave_XVnH[1] = be32to_hfloat(field->rtSway_xv2h);
    msg.rt_heave_XVnH[2] = be32to_hfloat(field->rtHeave_xv3h);

    ptr += sizeof(_phinsstdbin3::rtHeaveSurgeSway);
  }
  if (_phinsstdbin3::NAV_BLOCK_SMARTHEAVE & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::SmartHeave*>(ptr);

    msg.smart_heave_validity_time = static_cast<double>(be32toh(field->validity_time))*100.0e-6;
    msg.smart_heave = be32to_hfloat(field->heave_xv3h);

    ptr += sizeof(_phinsstdbin3::SmartHeave);
  }
  if (_phinsstdbin3::NAV_BLOCK_HPR_RATE & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::HprRate*>(ptr);

    msg.heading_rate = be32to_hfloat(field->heading_rate);
    msg.roll_rate = be32to_hfloat(field->roll_rate);
    msg.pitch_rate = be32to_hfloat(field->pitch_rate);

    ptr += sizeof(_phinsstdbin3::HprRate);
  }
  if (_phinsstdbin3::NAV_BLOCK_VESSEL_ROTRATE & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::BodyRotRates*>(ptr);

    msg.body_rates_XVn[0] = be32to_hfloat(field->rotrate_xv1);
    msg.body_rates_XVn[1] = be32to_hfloat(field->rotrate_xv2);
    msg.body_rates_XVn[2] = be32to_hfloat(field->rotrate_xv3);

    ptr += sizeof(_phinsstdbin3::BodyRotRates);
  }
  if (_phinsstdbin3::NAV_BLOCK_VESSEL_ACCEL & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::BodyAccel*>(ptr);

    msg.body_accel_XVn[0] = be32to_hfloat(field->accel_xv1);
    msg.body_accel_XVn[1] = be32to_hfloat(field->accel_xv2);
    msg.body_accel_XVn[2] = be32to_hfloat(field->accel_xv3);

    ptr += sizeof(_phinsstdbin3::BodyAccel);
  }
  if (_phinsstdbin3::NAV_BLOCK_POSITION & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::Position*>(ptr);

    msg.latitude = be64to_hdouble(field->latitude);
    msg.longitude = be64to_hdouble(field->longitude);
    msg.altitude = be32to_hfloat(field->altitude);
    if (field->altitude_ref == 0) {
      msg.altitude_reference = msg.ALTREF_GEOID;
    } else if (field->altitude_ref == 1) {
      msg.altitude_reference = msg.ALTREF_ELLPS;
    }

    ptr += sizeof(_phinsstdbin3::Position);
  }
  if (_phinsstdbin3::NAV_BLOCK_POSITION_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::PositionStddev*>(ptr);

    float north_stddev = be32to_hfloat(field->north_stddev);
    float east_stddev = be32to_hfloat(field->east_stddev);
    float corr = be32to_hfloat(field->north_east_correlation);
    msg.position_cov[0] = north_stddev * north_stddev;
    msg.position_cov[1] = corr * north_stddev * east_stddev;
    msg.position_cov[2] = msg.position_cov[1];
    msg.position_cov[3] = east_stddev * east_stddev;

    ptr += sizeof(_phinsstdbin3::PositionStddev);
  }
  if (_phinsstdbin3::NAV_BLOCK_GEO_SPEED & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::GeoSpeed*>(ptr);

    msg.velocity_NEU[0] = be32to_hfloat(field->north_vel);
    msg.velocity_NEU[1] = be32to_hfloat(field->east_vel);
    msg.velocity_NEU[2] = be32to_hfloat(field->up_vel);

    ptr += sizeof(_phinsstdbin3::GeoSpeed);
  }
  if (_phinsstdbin3::NAV_BLOCK_GEO_SPEED_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::GeoSpeedStddev*>(ptr);

    msg.velocity_stddev_NEU[0] = be32to_hfloat(field->north_vel_stddev);
    msg.velocity_stddev_NEU[1] = be32to_hfloat(field->east_vel_stddev);
    msg.velocity_stddev_NEU[2] = be32to_hfloat(field->up_vel_stddev);

    ptr += sizeof(_phinsstdbin3::GeoSpeedStddev);
  }
  if (_phinsstdbin3::NAV_BLOCK_GEO_CURRENT & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::GeoCurrent*>(ptr);

    msg.current_NE[0] = be32to_hfloat(field->north_current);
    msg.current_NE[1] = be32to_hfloat(field->east_current);

    ptr += sizeof(_phinsstdbin3::GeoCurrent);
  }
  if (_phinsstdbin3::NAV_BLOCK_GEO_CURRENT_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::GeoCurrentStddev*>(ptr);

    msg.current_stddev_NE[0] = be32to_hfloat(field->north_current_stddev);
    msg.current_stddev_NE[1] = be32to_hfloat(field->north_current_stddev);

    ptr += sizeof(_phinsstdbin3::GeoCurrentStddev);
  }
  if (_phinsstdbin3::NAV_BLOCK_SYSTEM_DATE & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::SystemDate*>(ptr);

    msg.year = be16toh(field->year);
    msg.month = field->month;
    msg.day = field->day;

    ptr += sizeof(_phinsstdbin3::SystemDate);
  }
  if (_phinsstdbin3::NAV_BLOCK_SENSOR_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::SensorStatus*>(ptr);

    msg.sensor_status[0] = be32toh(field->sensor_status1);
    msg.sensor_status[1] = be32toh(field->sensor_status2);

    ptr += sizeof(_phinsstdbin3::SensorStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_INS_ALGO_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::InsAlgoStatus*>(ptr);

    msg.ins_algo_status[0] = be32toh(field->algo_status1);
    msg.ins_algo_status[1] = be32toh(field->algo_status2);
    msg.ins_algo_status[2] = be32toh(field->algo_status3);
    msg.ins_algo_status[3] = be32toh(field->algo_status4);

    ptr += sizeof(_phinsstdbin3::InsAlgoStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_INS_SYS_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::InsSystemStatus*>(ptr);

    msg.ins_system_status[0] = be32toh(field->system_status1);
    msg.ins_system_status[1] = be32toh(field->system_status2);
    msg.ins_system_status[2] = be32toh(field->system_status3);

    ptr += sizeof(_phinsstdbin3::InsSystemStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_INS_USER_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::InsUserStatus*>(ptr);

    msg.ins_user_status = be32toh(field->user_status1);

    ptr += sizeof(_phinsstdbin3::InsUserStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_AHRS_ALGO_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AhrsAlgoStatus*>(ptr);

    msg.ahrs_algo_status = be32toh(field->algo_status);

    ptr += sizeof(_phinsstdbin3::AhrsAlgoStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_AHRS_SYS_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AhrsSystemStatus*>(ptr);

    msg.ahrs_system_status[0] = be32toh(field->system_status1);
    msg.ahrs_system_status[1] = be32toh(field->system_status2);
    msg.ahrs_system_status[2] = be32toh(field->system_status3);

    ptr += sizeof(_phinsstdbin3::AhrsSystemStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_AHRS_USER_STATUS & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AhrsUserStatus*>(ptr);

    msg.ahrs_user_status = be32toh(field->user_status1);

    ptr += sizeof(_phinsstdbin3::AhrsUserStatus);
  }
  if (_phinsstdbin3::NAV_BLOCK_HEAVE_SURGE_SWAY_SPEED & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::HeaveSurgeSwaySpeed*>(ptr);

    msg.heaveSurgeSway_speed_XVnH[0] = be32to_hfloat(field->rt_surge_speed_xv1h);
    msg.heaveSurgeSway_speed_XVnH[1] = be32to_hfloat(field->rt_sway_speed_xv2h);
    msg.heaveSurgeSway_speed_XVnH[2] = be32to_hfloat(field->rt_heave_speed_xv3h);

    ptr += sizeof(_phinsstdbin3::HeaveSurgeSwaySpeed);
  }
  if (_phinsstdbin3::NAV_BLOCK_VESSEL_SPEED & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::VesselSpeed*>(ptr);

    msg.body_velocity_XVn[0] = be32to_hfloat(field->vel_xv1);
    msg.body_velocity_XVn[1] = be32to_hfloat(field->vel_xv2);
    msg.body_velocity_XVn[2] = be32to_hfloat(field->vel_xv3);

    ptr += sizeof(_phinsstdbin3::VesselSpeed);
  }
  if (_phinsstdbin3::NAV_BLOCK_GEO_ACCEL_RAW & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::GeoAccel*>(ptr);

    msg.geo_accel_NEU[0] = be32to_hfloat(field->north_acc);
    msg.geo_accel_NEU[1] = be32to_hfloat(field->east_acc);
    msg.geo_accel_NEU[2] = be32to_hfloat(field->up_acc);

    ptr += sizeof(_phinsstdbin3::GeoAccel);
  }
  if (_phinsstdbin3::NAV_BLOCK_COG_SOG & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::CourseSpeed*>(ptr);

    msg.course_over_ground = be32to_hfloat(field->courseOverGround);
    msg.speed_over_ground = be32to_hfloat(field->speedOverGround);

    ptr += sizeof(_phinsstdbin3::CourseSpeed);
  }
  if (_phinsstdbin3::NAV_BLOCK_TEMPERATURES & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::Temperatures*>(ptr);

    msg.temp_avg_fog = be32to_hfloat(field->mean_fog);
    msg.temp_avg_acc = be32to_hfloat(field->mean_acc);
    msg.temp_board = be32to_hfloat(field->mean_board);

    ptr += sizeof(_phinsstdbin3::Temperatures);
  }
  if (_phinsstdbin3::NAV_BLOCK_ATTITUDE_QUATERNION & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AttitudeQuaternion*>(ptr);

    msg.attitude_quaternion[0] = be32to_hfloat(field->quat[0]);
    msg.attitude_quaternion[1] = be32to_hfloat(field->quat[1]);
    msg.attitude_quaternion[2] = be32to_hfloat(field->quat[2]);
    msg.attitude_quaternion[3] = be32to_hfloat(field->quat[3]);

    ptr += sizeof(_phinsstdbin3::AttitudeQuaternion);
  }
  if (_phinsstdbin3::NAV_BLOCK_ATTITUDE_QUAT_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::AttitudeQuaternionStddev*>(ptr);

    msg.attitude_quaternion_stddev[0] = be32to_hfloat(field->quat_stddev[0]);
    msg.attitude_quaternion_stddev[1] = be32to_hfloat(field->quat_stddev[1]);
    msg.attitude_quaternion_stddev[2] = be32to_hfloat(field->quat_stddev[2]);

    ptr += sizeof(_phinsstdbin3::AttitudeQuaternionStddev);
  }
  if (_phinsstdbin3::NAV_BLOCK_VESSEL_ACCEL_RAW & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::VesselRawAccel*>(ptr);

    msg.raw_accel_XVn[0] = be32to_hfloat(field->acc_xv1);
    msg.raw_accel_XVn[1] = be32to_hfloat(field->acc_xv2);
    msg.raw_accel_XVn[2] = be32to_hfloat(field->acc_xv3);

    ptr += sizeof(_phinsstdbin3::VesselRawAccel);
  }
  if (_phinsstdbin3::NAV_BLOCK_VESSEL_ACCEL_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::VesselAccelStddev*>(ptr);

    msg.body_accel_stddev_XVn[0] = be32to_hfloat(field->acc_xv1_stddev);
    msg.body_accel_stddev_XVn[1] = be32to_hfloat(field->acc_xv2_stddev);
    msg.body_accel_stddev_XVn[2] = be32to_hfloat(field->acc_xv3_stddev);

    ptr += sizeof(_phinsstdbin3::VesselAccelStddev);
  }
  if (_phinsstdbin3::NAV_BLOCK_VESSEL_ROTRATES_STDDEV & msg.nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::VesselRotRateStddev*>(ptr);

    msg.body_rotrate_stddev_XVn[0] = be32to_hfloat(field->rotrate_xv1_stddev);
    msg.body_rotrate_stddev_XVn[1] = be32to_hfloat(field->rotrate_xv2_stddev);
    msg.body_rotrate_stddev_XVn[2] = be32to_hfloat(field->rotrate_xv3_stddev);

    ptr += sizeof(_phinsstdbin3::VesselRotRateStddev);
  }
  if (_phinsstdbin3::NAV_EXT_VESSEL_ROT_ACCEL & msg.extended_nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::RawRotAccel*>(ptr);

    msg.raw_rot_acc_XVn[0] = be32to_hfloat(field->rotaccel_xv1);
    msg.raw_rot_acc_XVn[1] = be32to_hfloat(field->rotaccel_xv2);
    msg.raw_rot_acc_XVn[2] = be32to_hfloat(field->rotaccel_xv3);

    ptr += sizeof(_phinsstdbin3::RawRotAccel);
  }
  if (_phinsstdbin3::NAV_EXT_VESSEL_ROT_ACCEL_STDDEV & msg.extended_nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::RawRotAccelStddev*>(ptr);

    msg.raw_rot_acc_stddev_XVn[0] = be32to_hfloat(field->rotaccel_xv1_stddev);
    msg.raw_rot_acc_stddev_XVn[1] = be32to_hfloat(field->rotaccel_xv2_stddev);
    msg.raw_rot_acc_stddev_XVn[2] = be32to_hfloat(field->rotaccel_xv3_stddev);

    ptr += sizeof(_phinsstdbin3::RawRotAccelStddev);
  }
  if (_phinsstdbin3::NAV_EXT_VESSEL_RAW_ROTRATE & msg.extended_nav_fields) {
    auto field = reinterpret_cast<const _phinsstdbin3::VesselRawRotrate*>(ptr);

    msg.raw_rot_rate_XVn[0] = be32to_hfloat(field->raw_rotrate_xv1);
    msg.raw_rot_rate_XVn[1] = be32to_hfloat(field->raw_rotrate_xv2);
    msg.raw_rot_rate_XVn[2] = be32to_hfloat(field->raw_rotrate_xv3);

    ptr += sizeof(_phinsstdbin3::VesselRawRotrate);
  }

  // assemble the time
  boost::posix_time::ptime msgtime(boost::gregorian::date(msg.year, msg.month, msg.day),
      boost::posix_time::seconds(navt_secs) + boost::posix_time::microseconds(navt_usecs));

  msg.header.stamp = ros::Time::fromBoost(msgtime);

  return std::make_pair(true, msg);
}

ds_sensor_msgs::Gyro PhinsBin::makeGyroMsg(const ds_sensor_msgs::PhinsStdbin3& msg) {
  ds_sensor_msgs::Gyro ret;

  ret.header = msg.header;
  ret.ds_header = msg.ds_header;

  // copied essentially verbatim from the ascii phins.cpp parser
  // Stefano wrote during the ross upgrade.  See phins.cpp, Phins::parseReceivedPhinsStandard
  // Probably about line 185 or so.

  ret.roll = msg.roll * M_PI / 180.0;
  ret.pitch = msg.pitch * M_PI / 180.0;
  ret.heading = msg.heading * M_PI/180.0;

  // This format even features uncertainties!
  ret.roll_covar = msg.roll_stddev * M_PI/180;
  ret.roll_covar *= ret.roll_covar;
  ret.pitch_covar = msg.pitch_stddev * M_PI/180;
  ret.pitch_covar *= ret.pitch_covar;
  ret.heading_covar = msg.heading_stddev * M_PI/180;
  ret.heading_covar *= ret.heading_covar;

  Eigen::Quaterniond q(Eigen::AngleAxisd(-ret.heading + M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ret.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ret.roll, Eigen::Vector3d::UnitX()));
  ret.orientation.x = q.x();
  ret.orientation.y = q.y();
  ret.orientation.z = q.z();
  ret.orientation.w = q.w();

  return ret;
}

ds_sensor_msgs::Compass PhinsBin::makeCompassMsg(const ds_sensor_msgs::PhinsStdbin3 &msg) {
  ds_sensor_msgs::Compass ret;


  ret.header = msg.header;
  ret.ds_header = msg.ds_header;

  ret.is_true_heading = true;
  ret.heading = msg.heading * M_PI/180.0;
  ret.heading_covar = msg.heading_stddev * M_PI/180.0;
  ret.heading_covar *= ret.heading_covar;

  return ret;
}

sensor_msgs::Imu PhinsBin::makeImuMsg(const ds_sensor_msgs::PhinsStdbin3& msg) {
  sensor_msgs::Imu ret;

  ret.header = msg.header;

  // compute the quaternion
  double rheading = msg.heading * M_PI/180;
  double rpitch = msg.pitch * M_PI/180;
  double rroll = msg.roll * M_PI/180;
  Eigen::Quaterniond q(Eigen::AngleAxisd(-rheading + M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rroll, Eigen::Vector3d::UnitX()));
  ret.orientation.x = q.x();
  ret.orientation.y = q.y();
  ret.orientation.z = q.z();
  ret.orientation.w = q.w();

  // orientation uncertainties -- assume diagonal
  ret.orientation_covariance[0] = (msg.roll_stddev*M_PI/180.0)*(msg.roll_stddev*M_PI/180.0);
  ret.orientation_covariance[4] = (msg.pitch_stddev*M_PI/180.0)*(msg.pitch_stddev*M_PI/180.0);
  ret.orientation_covariance[8] = (msg.heading_stddev*M_PI/180.0)*(msg.heading_stddev*M_PI/180.0);

  // angular velocities
  ret.angular_velocity.x = msg.body_rates_XVn[0] * M_PI/180;
  ret.angular_velocity.y = msg.body_rates_XVn[1] * M_PI/180;
  ret.angular_velocity.z = msg.body_rates_XVn[2] * M_PI/180;

  // angular velocity uncertainty-- assume diagonal
  ret.angular_velocity_covariance[0] = msg.body_rotrate_stddev_XVn[0] * M_PI/180;
  ret.angular_velocity_covariance[0] *= ret.angular_velocity_covariance[0];
  ret.angular_velocity_covariance[4] = msg.body_rotrate_stddev_XVn[1] * M_PI/180;
  ret.angular_velocity_covariance[4] *= ret.angular_velocity_covariance[4];
  ret.angular_velocity_covariance[8] = msg.body_rotrate_stddev_XVn[2] * M_PI/180;
  ret.angular_velocity_covariance[8] *= ret.angular_velocity_covariance[8];

  // linear accelerations
  ret.linear_acceleration.x = msg.body_accel_XVn[0];
  ret.linear_acceleration.y = msg.body_accel_XVn[1];
  ret.linear_acceleration.z = msg.body_accel_XVn[2];

  // linear acceleration uncertainty
  ret.linear_acceleration_covariance[0] = msg.body_accel_stddev_XVn[0];
  ret.linear_acceleration_covariance[0] *= ret.linear_acceleration_covariance[0];
  ret.linear_acceleration_covariance[4] = msg.body_accel_stddev_XVn[1];
  ret.linear_acceleration_covariance[4] *= ret.linear_acceleration_covariance[4];
  ret.linear_acceleration_covariance[8] = msg.body_accel_stddev_XVn[2];
  ret.linear_acceleration_covariance[8] *= ret.linear_acceleration_covariance[8];

  return ret;
}

ds_sensor_msgs::PhinsStatus PhinsBin::makePhinsStatusMsg(const ds_sensor_msgs::PhinsStdbin3& msg) {
  ds_sensor_msgs::PhinsStatus ret;

  ret.header = msg.header;
  ret.ds_header = msg.ds_header;

  ret.algorithm.status = static_cast<uint64_t>(msg.ins_algo_status[0])
      | (static_cast<uint64_t>(msg.ins_algo_status[1]) << 32);
  // msg.ins_algo_status[2] is not included in the PixseAlgsts.msg
  // msg.ins_algo_status[3] is not included in the PixseAlgsts.msg

  ret.system.status = static_cast<uint64_t>(msg.ins_system_status[0])
      | (static_cast<uint64_t>(msg.ins_system_status[1]) << 32);
  // msg.ins_system_status[2] is not included in the PixseStatus message

  ret.user.status = msg.ins_user_status;

  return ret;
}

} // namespace ds_sensors
