//
// Created by ivaughn on 8/23/19.
//

#ifndef DS_SENSORS_PHINSTDV3_H
#define DS_SENSORS_PHINSTDV3_H

#include <stdint.h>

namespace _phinsstdbin3 {

const uint8_t HEADER1 = 'I';
const uint8_t HEADER2 = 'X';
const uint8_t VERSION2 = 0x02;
const uint8_t VERSION3 = 0x03;

struct Header {
  uint8_t header1;
  uint8_t header2;
  uint8_t version;
  uint32_t nav_data_mask;
  uint32_t nav_extended_data_mask;
  uint32_t external_data_mask;
  uint16_t total_size;
  uint32_t validity_time;
  uint32_t counter;
} __attribute__((packed));

const uint32_t NAV_BLOCK_ATTITUDEHEADING        = (1 <<  0);
const uint32_t NAV_BLOCK_ATTITUDEHEADING_STDDEV = (1 <<  1);
const uint32_t NAV_BLOCK_RT_HEAVESURGESWAY      = (1 <<  2);
const uint32_t NAV_BLOCK_SMARTHEAVE             = (1 <<  3);
const uint32_t NAV_BLOCK_HPR_RATE               = (1 <<  4);
const uint32_t NAV_BLOCK_VESSEL_ROTRATE         = (1 <<  5);
const uint32_t NAV_BLOCK_VESSEL_ACCEL           = (1 <<  6);
const uint32_t NAV_BLOCK_POSITION               = (1 <<  7);
const uint32_t NAV_BLOCK_POSITION_STDDEV        = (1 <<  8);
const uint32_t NAV_BLOCK_GEO_SPEED              = (1 <<  9);
const uint32_t NAV_BLOCK_GEO_SPEED_STDDEV       = (1 << 10);
const uint32_t NAV_BLOCK_GEO_CURRENT            = (1 << 11);
const uint32_t NAV_BLOCK_GEO_CURRENT_STDDEV     = (1 << 12);
const uint32_t NAV_BLOCK_SYSTEM_DATE            = (1 << 13);
const uint32_t NAV_BLOCK_SENSOR_STATUS          = (1 << 14);
const uint32_t NAV_BLOCK_INS_ALGO_STATUS        = (1 << 15);
const uint32_t NAV_BLOCK_INS_SYS_STATUS         = (1 << 16);
const uint32_t NAV_BLOCK_INS_USER_STATUS        = (1 << 17);
const uint32_t NAV_BLOCK_AHRS_ALGO_STATUS       = (1 << 18);
const uint32_t NAV_BLOCK_AHRS_SYS_STATUS        = (1 << 19);
const uint32_t NAV_BLOCK_AHRS_USER_STATUS       = (1 << 20);
const uint32_t NAV_BLOCK_HEAVE_SURGE_SWAY_SPEED = (1 << 21);
const uint32_t NAV_BLOCK_VESSEL_SPEED           = (1 << 22);
const uint32_t NAV_BLOCK_GEO_ACCEL_RAW          = (1 << 23);
const uint32_t NAV_BLOCK_COG_SOG                = (1 << 24);
const uint32_t NAV_BLOCK_TEMPERATURES           = (1 << 25);
const uint32_t NAV_BLOCK_ATTITUDE_QUATERNION    = (1 << 26);
const uint32_t NAV_BLOCK_ATTITUDE_QUAT_STDDEV   = (1 << 27);
const uint32_t NAV_BLOCK_VESSEL_ACCEL_RAW       = (1 << 28);
const uint32_t NAV_BLOCK_VESSEL_ACCEL_STDDEV    = (1 << 29);
const uint32_t NAV_BLOCK_VESSEL_ROTRATES_STDDEV = (1 << 30);

const uint32_t NAV_EXT_VESSEL_ROT_ACCEL         = (1 <<  0);
const uint32_t NAV_EXT_VESSEL_ROT_ACCEL_STDDEV  = (1 <<  1);
const uint32_t NAV_EXT_VESSEL_RAW_ROTRATE       = (1 <<  2);

// everything is big-endian.  We have to encode the structures using
// 32-bit ints instead of float_bes to properly endian-swap on little-endian
// platforms.  However, to make it clear which fields are which
// we'll use a typedef
typedef uint32_t float_be;
typedef uint64_t double_be;

// we don't bother with external sensors
struct AttitudeHeading {
  float_be heading;
  float_be roll;
  float_be pitch;
} __attribute__((packed));

struct AttitudeHeadingStddev {
  float_be heading_stddev;
  float_be roll_stddev;
  float_be pitch_stddev;
} __attribute__((packed));

struct rtHeaveSurgeSway {
  float_be rtHeave_noarm;
  float_be rtHeave_xv3h;
  float_be rtSurge_xv1h;
  float_be rtSway_xv2h;
} __attribute__((packed));

struct SmartHeave {
  uint32_t validity_time; // 100us
  float_be heave_xv3h; // m
} __attribute__((packed));

struct HprRate {
  float_be heading_rate; // deg/s
  float_be roll_rate; // deg/s
  float_be pitch_rate; // deg/s
} __attribute__((packed));

struct BodyRotRates {
  float_be rotrate_xv1;
  float_be rotrate_xv2;
  float_be rotrate_xv3;
} __attribute__((packed));

struct BodyAccel {
  float_be accel_xv1;
  float_be accel_xv2;
  float_be accel_xv3;
} __attribute__((packed));

struct Position {
  double_be latitude;
  double_be longitude;
  uint8_t altitude_ref;
  float_be altitude;
} __attribute__((packed));

struct PositionStddev {
  float_be north_stddev;
  float_be east_stddev;
  float_be north_east_correlation;
  float_be altitude_stddev;
} __attribute__((packed));

struct GeoSpeed {
  float_be north_vel;
  float_be east_vel;
  float_be up_vel;
} __attribute__((packed));

struct GeoSpeedStddev {
  float_be north_vel_stddev;
  float_be east_vel_stddev;
  float_be up_vel_stddev;
} __attribute__((packed));

struct GeoCurrent {
  float_be north_current;
  float_be east_current;
} __attribute__((packed));

struct GeoCurrentStddev {
  float_be north_current_stddev;
  float_be east_current_stddev;
} __attribute__((packed));

struct SystemDate {
  uint8_t day;
  uint8_t month;
  uint16_t year;
} __attribute__((packed));

struct SensorStatus {
  uint32_t sensor_status1;
  uint32_t sensor_status2;
} __attribute__((packed));

struct InsAlgoStatus {
  uint32_t algo_status1;
  uint32_t algo_status2;
  uint32_t algo_status3;
  uint32_t algo_status4;
} __attribute__((packed));

struct InsSystemStatus {
  uint32_t system_status1;
  uint32_t system_status2;
  uint32_t system_status3;
} __attribute__((packed));

struct InsUserStatus {
  uint32_t user_status1;
} __attribute__((packed));

struct AhrsAlgoStatus {
  uint32_t algo_status;
} __attribute__((packed));

struct AhrsSystemStatus {
  uint32_t system_status1;
  uint32_t system_status2;
  uint32_t system_status3;
} __attribute__((packed));

struct AhrsUserStatus {
  uint32_t user_status1;
} __attribute__((packed));

struct HeaveSurgeSwaySpeed {
  float_be rt_heave_speed_xv3h;
  float_be rt_surge_speed_xv1h;
  float_be rt_sway_speed_xv2h;
} __attribute__((packed));

struct VesselSpeed {
  float_be vel_xv1;
  float_be vel_xv2;
  float_be vel_xv3;
} __attribute__((packed));

struct GeoAccel {
  float_be north_acc;
  float_be east_acc;
  float_be up_acc;
} __attribute__((packed));

struct CourseSpeed {
  float_be courseOverGround;
  float_be speedOverGround;
} __attribute__((packed));

struct Temperatures {
  float_be mean_fog;
  float_be mean_acc;
  float_be mean_board;
} __attribute__((packed));

struct AttitudeQuaternion {
  // tehse don't really get better labels; consult the manual
  float_be quat[4];
} __attribute__((packed));

struct AttitudeQuaternionStddev {
  float_be quat_stddev[3];
} __attribute__((packed));

struct VesselRawAccel {
  float_be acc_xv1;
  float_be acc_xv2;
  float_be acc_xv3;
} __attribute__((packed));

struct VesselAccelStddev {
  float_be acc_xv1_stddev;
  float_be acc_xv2_stddev;
  float_be acc_xv3_stddev;
} __attribute__((packed));

struct VesselRotRateStddev {
  float_be rotrate_xv1_stddev;
  float_be rotrate_xv2_stddev;
  float_be rotrate_xv3_stddev;
} __attribute__((packed));

struct RawRotAccel {
  float_be rotaccel_xv1;
  float_be rotaccel_xv2;
  float_be rotaccel_xv3;
} __attribute__((packed));

struct RawRotAccelStddev {
  float_be rotaccel_xv1_stddev;
  float_be rotaccel_xv2_stddev;
  float_be rotaccel_xv3_stddev;
} __attribute__((packed));

struct VesselRawRotrate {
  float_be raw_rotrate_xv1;
  float_be raw_rotrate_xv2;
  float_be raw_rotrate_xv3;
} __attribute__((packed));


// Static values for masks to interpret status fields for a C3
namespace status {

typedef uint32_t status32_t;

namespace sensor {

namespace w1 {
static const status32_t DATA_READY_ERR = 0x00000001;
static const status32_t SOURCE_POWER_CONTROL_ERR = 0x00000002;
static const status32_t SOURCE_DIODE_ERR = 0x00000004;
static const status32_t SOURCE_MODE_ERR = 0x00000008;
static const status32_t ACC_X_SATURATION_ERR = 0x00000010;
static const status32_t ACC_Y_SATURATION_ERR = 0x00000020;
static const status32_t ACC_Z_SATURATION_ERR = 0x00000040;
static const status32_t ACC_X_ACQ_ERR = 0x00000080;
static const status32_t ACC_Y_ACQ_ERR = 0x00000100;
static const status32_t ACC_Z_ACQ_ERR = 0x00000200;
static const status32_t FOG_X_SATURATION_ERR = 0x00000400;
static const status32_t FOG_Y_SATURATION_ERR = 0x00000800;
static const status32_t FOG_Z_SATURATION_ERR = 0x00001000;
static const status32_t FOG_X_VPI_ERR = 0x00002000;
static const status32_t FOG_Y_VPI_ERR = 0x00004000;
static const status32_t FOG_Z_VPI_ERR = 0x00008000;
static const status32_t FOG_X_LOW_POWER = 0x00010000;
static const status32_t FOG_Y_LOW_POWER = 0x00020000;
static const status32_t FOG_Z_LOW_POWER = 0x00040000;
static const status32_t FOG_X_ACQ_ERR = 0x00080000;
static const status32_t FOG_Y_ACQ_ERR = 0x00100000;
static const status32_t FOG_Z_ACQ_ERR = 0x00200000;
static const status32_t FOG_X_CRC_ERR = 0x00400000;
static const status32_t FOG_Y_CRC_ERR = 0x00800000;
static const status32_t FOG_Z_CRC_ERR = 0x01000000;
static const status32_t TEMP_ACQ_ERR = 0x02000000;
static const status32_t TEMP_THRESHOLD_ERR = 0x04000000;
static const status32_t DTEMP_THRESHOLD_ERR= 0x08000000;
static const status32_t SENSOR_DATA_FIFO_WARNING = 0x10000000;
static const status32_t SENSOR_DATA_FIFO_ERR = 0x20000000;
static const status32_t SOURCE_POWER_ERR = 0x40000000;
static const status32_t SOURCE_RECEPTION_ERR = 0x80000000;
} // namespace w1
static const status32_t w1_errors = ~(w1::SENSOR_DATA_FIFO_WARNING);

namespace w2 {
static const status32_t FOG_X_ERR = 0x00000001;
static const status32_t FOG_Y_ERR = 0x00000002;
static const status32_t FOG_Z_ERR = 0x00000004;
static const status32_t SOURCE_ERR = 0x00000008;
static const status32_t ACC_X_ERR = 0x00000010;
static const status32_t ACC_Y_ERR = 0x00000020;
static const status32_t ACC_Z_ERR = 0x00000040;
static const status32_t TEMP_ERR = 0x00000080;
static const status32_t DSP_OVERLOAD = 0x00000100;
static const status32_t ERR_INIT_CAN_ACC_X = 0x00000200;
static const status32_t ERR_INIT_CAN_ACC_Y = 0x00000400;
static const status32_t ERR_INIT_CAN_ACC_Z = 0x00000800;
static const status32_t MODELISATION_ERROR = 0x00001000;
// reserved -- 0x00002000
// reserved -- 0x00004000
// reserved -- 0x00008000
// reserved -- 0x00010000
// reserved -- 0x00020000
// reserved -- 0x00040000
// reserved -- 0x00080000
// reserved -- 0x00100000
// reserved -- 0x00200000
// reserved -- 0x00400000
// reserved -- 0x00800000
// reserved -- 0x01000000
// reserved -- 0x02000000
// reserved -- 0x04000000
// reserved -- 0x08000000
// reserved -- 0x10000000
// reserved -- 0x20000000
static const status32_t DEGRADED_MODE = 0x40000000;
static const status32_t FAILURE_MODE = 0x80000000;
} // namespace w2

} // namespace sensor

namespace system {
namespace w1 {
static const status32_t SERIAL_IN_R_ERR = 0x00000001;
static const status32_t INPUT_A_ERR = 0x00000002;
static const status32_t INPUT_B_ERR = 0x00000004;
static const status32_t INPUT_C_ERR = 0x00000008;
static const status32_t INPUT_D_ERR = 0x00000010;
static const status32_t INPUT_E_ERR = 0x00000020;
static const status32_t INPUT_F_ERR = 0x00000040;
static const status32_t INPUT_G_ERR = 0x00000080;
static const status32_t INPUT_R_ACTIVITY = 0x00000100;
static const status32_t INPUT_A_ACTIVITY = 0x00000200;
static const status32_t INPUT_B_ACTIVITY = 0x00000400;
static const status32_t INPUT_C_ACTIVITY = 0x00000800;
static const status32_t INPUT_D_ACTIVITY = 0x00001000;
static const status32_t INPUT_E_ACTIVITY = 0x00002000;
static const status32_t INPUT_F_ACTIVITY = 0x00004000;
static const status32_t INPUT_G_ACTIVITY = 0x00008000;
static const status32_t OUTPUT_R_FULL = 0x00010000;
static const status32_t OUTPUT_A_FULL = 0x00020000;
static const status32_t OUTPUT_B_FULL = 0x00040000;
static const status32_t OUTPUT_C_FULL = 0x00080000;
static const status32_t OUTPUT_D_FULL = 0x00100000;
static const status32_t OUTPUT_E_FULL = 0x00200000;
static const status32_t ETHERNET_FULL = 0x00400000;
// reserved -- 0x00800000
static const status32_t INTERNAL_TIME_USED = 0x01000000;
// reserved -- 0x02000000 eventmarker on LANDINS
static const status32_t ETHERNET_PORT_ACTIVITY = 0x04000000;
static const status32_t PULSE_IN_A_ACTIVITY = 0x08000000;
static const status32_t PULSE_IN_B_ACTIVITY = 0x10000000;
static const status32_t PULSE_IN_C_ACTIVITY = 0x20000000;
static const status32_t PULSE_IN_D_ACTIVITY = 0x40000000;
// reserved -- 0x80000000
} // namespace w1

namespace w2 {
static const status32_t DVL_BT_DETECTED = 0x00000001;
static const status32_t DVL_WT_DETECTED = 0x00000002;
static const status32_t GPS_DETECTED = 0x00000004;
static const status32_t GPS2_DETECTED = 0x00000008;
static const status32_t USBL_DETECTED = 0x00000010;
static const status32_t LBL_DETECTED = 0x00000020;
static const status32_t DEPTH_DETECTED = 0x00000040;
static const status32_t EMLOG_DETECTED = 0x00000080;
static const status32_t DMI_DETECTED = 0x00000100;
static const status32_t UTC_DETECTED = 0x00000200;
static const status32_t ALTITUDE_DETECTED = 0x00000400;
static const status32_t PPS_DETECTED = 0x00000800;
static const status32_t ZUP_MODE_ACTIVATED = 0x00001000;
// reserved -- 0x00002000
static const status32_t MANUAL_GPS_DETECTED = 0x00004000;
static const status32_t CTD_DETECTED = 0x00008000;
static const status32_t SIMULATION_MODE = 0x00010000;
// reserved -- 0x00020000
static const status32_t DSP_INCOMPATIBILITY = 0x00040000;
static const status32_t HEADING_ALERT = 0x00080000;
static const status32_t POSITION_ALERT = 0x00100000;
static const status32_t WAIT_FOR_POSITION = 0x00240000;
// reserved -- 0x00400000
static const status32_t POLAR_MODE = 0x00800000;
static const status32_t INTERNAL_LOG = 0x01000000;
// reserved -- 0x02000000
static const status32_t DOV_CORR_DETECTED = 0x04000000;
static const status32_t MPC_OVERLOAD = 0x08000000;
static const status32_t POWER_SUPPLY_FAILURE = 0x10000000;
static const status32_t RD_MODE = 0x20000000;
static const status32_t CONFIGURATION_SAVED = 0x40000000;
// reserved -- 0x80000000
} // namespace w2

namespace w3 {
static const status32_t UTC2_DETECTED = 0x00000001;
static const status32_t PPS2_DETECTED = 0x00000002;
static const status32_t ADVANCED_FILTERING = 0x00000004;
static const status32_t NTP_SYNC_IN_PROGRESS = 0x00000008;
static const status32_t NTP_RECEIVED = 0x00000010;
static const status32_t NTP_SYNC = 0x00000020;
static const status32_t NTP_FAILED = 0x00000040;
// reserved -- 0x00000080
static const status32_t DVL2_BT_DETECTED = 0x00000100;
static const status32_t DVL2_WT_DETECTED = 0x00000200;
static const status32_t EMLOG2_DETECTED = 0x00000400;
// reserved -- 0x00000800
// reserved -- 0x00001000
// reserved -- 0x00002000
// reserved -- 0x00004000
// reserved -- 0x00008000
// reserved -- 0x00010000
// reserved -- 0x00020000
// reserved -- 0x00040000
// reserved -- 0x00080000
// reserved -- 0x00100000
// reserved -- 0x00200000
// reserved -- 0x00400000
// reserved -- 0x00800000
// reserved -- 0x01000000
// reserved -- 0x02000000
// reserved -- 0x04000000
// reserved -- 0x08000000
// reserved -- 0x10000000
// reserved -- 0x20000000
// reserved -- 0x40000000
// reserved -- 0x80000000
} // namespace w3
} // namespace system

namespace algorithm {
namespace w1 {
static const status32_t NAVIGATION = 0x00000001;
static const status32_t ALIGNMENT = 0x00000002;
static const status32_t FINE_ALIGNMENT = 0x00000004;
static const status32_t DEAD_RECKONING = 0x00000008;
static const status32_t GPS_ALITITUDE = 0x00000010;
static const status32_t DEPTHSENSOR_ALTITUDE = 0x00000020;
static const status32_t ZERO_ALTITUDE = 0x00000040;
static const status32_t HYDRO_ALTITUDE = 0x00000080;
static const status32_t LOG_RECEIVED = 0x00000100;
static const status32_t LOG_VALID = 0x00000200;
static const status32_t LOG_WAITING = 0x00000400;
static const status32_t LOG_REJECTED = 0x00000800;
static const status32_t GPS_RECEIVED = 0x00001000;
static const status32_t GPS_VALID = 0x00002000;
static const status32_t GPS_WAITING = 0x00004000;
static const status32_t GPS_REJECTED = 0x00008000;
static const status32_t USBL_RECEIVED = 0x00010000;
static const status32_t USBL_VALID = 0x00020000;
static const status32_t USBL_WAITING = 0x00040000;
static const status32_t USBL_REJECTED = 0x00080000;
static const status32_t DEPTH_RECEIVED = 0x00100000;
static const status32_t DEPTH_VALID = 0x00200000;
static const status32_t DEPTH_WAITING = 0x00400000;
static const status32_t DEPTH_REJECTED = 0x00800000;
static const status32_t LBL_RECEIVED = 0x01000000;
static const status32_t LBL_VALID = 0x02000000;
static const status32_t LBL_WAITING = 0x04000000;
static const status32_t LBL_REJECTED = 0x08000000;
static const status32_t ALTITUDE_SATURATION = 0x10000000;
static const status32_t SPEED_SATURATION = 0x20000000;
static const status32_t INTERPOLATION_MISSED = 0x40000000;
static const status32_t HEAVE_INITIALISATION = 0x80000000;
} // namespace w1
namespace w2 {
static const status32_t WATERTRACK_RECEIVED = 0x00000001;
static const status32_t WATERTRACK_VALID = 0x00000002;
static const status32_t WATERTRACK_WAITING = 0x00000004;
static const status32_t WATERTRACK_REJECTED = 0x00000008;
static const status32_t GPS2_RECEIVED = 0x00000010;
static const status32_t GPS2_VALID = 0x00000020;
static const status32_t GPS2_WAITING = 0x00000040;
static const status32_t GPS2_REJECTED = 0x00000080;
// reserved -- 0x00000100
// reserved -- 0x00000200
// reserved -- 0x00000400
// reserved -- 0x00000800
static const status32_t ALTITUDE_RECEIVED = 0x00001000;
static const status32_t ALTITUDE_VALID = 0x00002000;
static const status32_t ALTITUDE_WAITING = 0x00004000;
static const status32_t ALTITUDE_REJECTED = 0x00008000;
static const status32_t ZUPT_MODE_ACTIVATED = 0x00010000;
static const status32_t ZUPT_MODE_VALID = 0x00020000;
static const status32_t AUTOSTATICBENCH_ZUPT_MODE_ACTIVATED = 0x00040000;
static const status32_t AUTOSTATICBENCH_ZUPT_MODE_VALID = 0x00080000;
static const status32_t STATIC_CONVERGENCE_ON = 0x00100000;
static const status32_t STATIC_CONV_GO_TO_NAV = 0x00200000;
static const status32_t FAST_ALIGNMENT = 0x00400000;
static const status32_t EMULATION_MODE = 0x00800000;
static const status32_t EMLOG_RECEIVED = 0x01000000;
static const status32_t EMLOG_VALID = 0x02000000;
static const status32_t EMLOG_WAITING = 0x04000000;
static const status32_t EMLOG_REJECTED = 0x08000000;
static const status32_t MANUALGPS_RECEIVED = 0x10000000;
static const status32_t MANUALGPS_VALID = 0x20000000;
static const status32_t MANUALGPS_WAITING = 0x40000000;
static const status32_t MANUALGPS_REJECTED = 0x80000000;
} // namespace w2
namespace w3 {
static const status32_t SVL_RECEIVED = 0x00000001;
static const status32_t SVL_VALID = 0x00000002;
static const status32_t SVL_WAITING = 0x00000004;
static const status32_t SVL_REJECTED = 0x00000008;
static const status32_t EMLOG2_RECEIVED = 0x00000010;
static const status32_t EMLOG2_VALID = 0x00000020;
static const status32_t EMLOG2_WAITING = 0x00000040;
static const status32_t EMLOG2_REJECTED = 0x00000080;
static const status32_t USBL2_RECEIVED = 0x00000100;
static const status32_t USBL2_VALID = 0x00000200;
static const status32_t USBL2_WAITING = 0x00000400;
static const status32_t USBL2_REJECTED = 0x00000800;
static const status32_t USBL3_RECEIVED = 0x00001000;
static const status32_t USBL3_VALID = 0x00002000;
static const status32_t USBL3_WAITING = 0x00004000;
static const status32_t USBL3_REJECTED = 0x00008000;
// reserved -- 0x00010000
static const status32_t CALCHK = 0x00020000;
static const status32_t RESTORE_ATTITUDE_FAILED = 0x00040000;
static const status32_t REL_SPD_ZUP_ACTIVATED = 0x00080000;
static const status32_t REL_SPD_ZUP_VALID = 0x00100000;
static const status32_t EXT_SENSOR_OUTDATED = 0x00200000;
static const status32_t SENSOR_USED_BEFORE_CALIB = 0x00400000;
static const status32_t RESTORE_ATTITUDE_REJECTED = 0x00800000;
// reserved -- 0x01000000
// reserved -- 0x02000000
// reserved -- 0x04000000
static const status32_t POLAR_VALIDITY = 0x08000000;
static const status32_t FIRM_INCOMPATIBLES = 0x10000000;
// reserved -- 0x20000000
// reserved -- 0x40000000
// reserved -- 0x80000000
} // namespace w3
namespace w4 {
static const status32_t LOG2_RECEIVED = 0x00000001;
static const status32_t LOG2_VALID = 0x00000002;
static const status32_t LOG2_WAITING = 0x00000004;
static const status32_t LOG2_REJECTED = 0x00000008;
static const status32_t WATERTRACK2_RECEIVED = 0x00000010;
static const status32_t WATERTRACK2_VALID = 0x00000020;
static const status32_t WATERTRACK2_WAITING = 0x00000040;
static const status32_t WATERTRACK2_REJECTED = 0x00000080;
static const status32_t DVL_DIST_TRAVELLED_VALID = 0x00000100;
static const status32_t DVL_CALIBRATION_NONE = 0x00000200;
static const status32_t DVL_ROUGH_CALIBRATION = 0x00000400;
static const status32_t DVL_FINE_CALIBRAITION = 0x00000800;
static const status32_t DVL_CHECK_CALIBRATION = 0x00001000;
// reserved -- 0x00002000
// reserved -- 0x00004000
// reserved -- 0x00008000
// reserved -- 0x00010000
// reserved -- 0x00020000
// reserved -- 0x00040000
// reserved -- 0x00080000
// reserved -- 0x00100000
// reserved -- 0x00200000
// reserved -- 0x00400000
// reserved -- 0x00800000
// reserved -- 0x01000000
// reserved -- 0x02000000
// reserved -- 0x04000000
// reserved -- 0x08000000
// reserved -- 0x10000000
// reserved -- 0x20000000
// reserved -- 0x40000000
// reserved -- 0x80000000
} // namespace w4
} // namespace algorithm

namespace user {
namespace w1 {
static const status32_t DVL_RECEIVED_VALID = 0x00000001;
static const status32_t GPS_RECEIVED_VALID = 0x00000002;
static const status32_t DEPTH_RECEIVED_VALID = 0x00000004;
static const status32_t USBL_RECEIVED_VALID = 0x00000008;
static const status32_t LBL_RECEIVED_VALID = 0x00000010;
static const status32_t GPS2_RECEIVED_VALID = 0x00000020;
static const status32_t EMLOG_RECEIVED_VALID = 0x00000040;
static const status32_t MANUALGPS_RECEIVED_VALID = 0x00000080;
static const status32_t TIME_RECEIVED_VALID = 0x00000100;
static const status32_t FOG_ANOMALY = 0x00000200;
static const status32_t ACC_ANOMALY = 0x00000400;
static const status32_t TEMPERATURE_ERR = 0x00000800;
static const status32_t CPU_OVERLOAD = 0x00001000;
static const status32_t DYNAMIC_EXCEEDED = 0x00002000;
static const status32_t SPEED_SATURATION = 0x00004000;
static const status32_t ALTITUDE_SATURATION = 0x00008000;
static const status32_t INPUT_A_ERR = 0x00010000;
static const status32_t INPUT_B_ERR = 0x00020000;
static const status32_t INPUT_C_ERR = 0x00040000;
static const status32_t INPUT_D_ERR = 0x00080000;
static const status32_t INPUT_E_ERR = 0x00100000;
static const status32_t OUTPUT_A_ERR = 0x00200000;
static const status32_t OUTPUT_B_ERR = 0x00400000;
static const status32_t OUTPUT_C_ERR = 0x00800000;
static const status32_t OUTPUT_D_ERR = 0x01000000;
static const status32_t OUTPUT_E_ERR = 0x02000000;
static const status32_t HRP_INVALID = 0x04000000;
static const status32_t ALIGNMENT = 0x08000000;
static const status32_t FINE_ALIGNMENT = 0x10000000;
static const status32_t NAVIGATION = 0x20000000;
static const status32_t DEGRADED_MODE = 0x40000000;
static const status32_t FAILURE_MODE = 0x80000000;
} // namespace w1
} // namespace user

} // namespace status

} //namespace _phinsstdbin3

#endif //DS_SENSORS_PHINSTDV3_H
