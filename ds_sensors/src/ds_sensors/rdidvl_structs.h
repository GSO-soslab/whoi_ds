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
#ifndef DS_SENSORS_RDIDVL_STRUCTS
#define DS_SENSORS_RDIDVL_STRUCTS

namespace ds_sensors
{
namespace rdidvl_structs
{

struct header
{
  uint16_t headerid;
  uint16_t bytes;
  uint8_t spare;
  uint8_t numdata;
  uint16_t addr[10];  // Typically =3, but lets leave some extra room. We can add a catch for new data types.
};

#define RDI_PD0_ID_FIXED_LEADER 0x0000
struct fixedleader
{
  uint16_t headerid;  // defined 0000h
  uint8_t fw_ver;
  uint8_t fw_rev;
  uint8_t config0;   // frequency, concave vs. convex, beam # lsb
  uint8_t config1;   // msb
  uint8_t real_sim;  // 0=real is default
  uint8_t lag;
  uint8_t beams;
  uint8_t cells;
  uint16_t pings;
  uint16_t cell_depth;
  uint16_t blank;
  uint8_t signal_proc;  // Always =1
  uint8_t min_thresh;
  uint8_t code_reps;
  uint8_t min_good_pings;
  uint16_t good_thresh;
  uint8_t ping_int_min;  // Time between ping intervals
  uint8_t ping_int_sec;
  uint8_t ping_int_hun;
  uint8_t coord_trans;  // xxx00xxx = no transformation
  uint16_t hdng_align;  // ea-command
  uint16_t hdng_bias;   // eb-command

  uint8_t sensor_src;    // ez-command
  uint8_t sensor_avail;  // same as sensor_src pattern

  uint16_t bin1_dist;
  uint16_t xmit_pulse_len;
  uint8_t avg_start;
  uint8_t avg_end;
  uint8_t avg_false_thresh;
  uint8_t spare0;
  uint16_t trans_lag_dist;
  uint8_t serial_num_cpu[8];
  uint16_t wb_cmd;      // MAY BE INVALID DATA
  uint8_t power;        // MAY BE INVALID DATA
  uint8_t spare1;       // MAY BE INVALID DATA
  uint32_t serial_num;  // MAY BE INVALID DATA
  uint8_t beam_angle;   // MAY BE INVALID DATA
} __attribute__((packed));

#define RDI_PD0_ID_VARIABLE_LEADER 0x0080
struct variableleader
{
  uint16_t headerid;
  uint16_t ensemble_num;
  uint8_t rtc_year;  // Real-time clock for current data ensemble
  uint8_t rtc_month;
  uint8_t rtc_day;
  uint8_t rtc_hour;
  uint8_t rtc_minute;
  uint8_t rtc_second;
  uint8_t rtc_hundredth;
  uint8_t ensemble_num_msb;
  uint8_t bit_result;
  uint8_t reserved0;
  uint16_t sound_vel;    // ec
  uint16_t depth;        // ed
  uint16_t heading;      // eh
  int16_t pitch;         // ep
  int16_t roll;          // er
  uint16_t salinity;     // es
  uint16_t temperature;  // et
  uint8_t mpt_minute;    // Minimum prep-ping waiting time
  uint8_t mpt_second;
  uint8_t mpt_hundredth;
  uint8_t heading_std;  // standard deviations
  uint8_t pitch_std;
  uint8_t roll_std;
  uint8_t adc[8];                // analog-digital converter from dsp board 0-8
  uint8_t error_status_word[4];  // easier to process with array
  uint16_t reserved1;
  uint32_t pressure;
  uint32_t pressure_variance;
  uint8_t spare;
  uint8_t rtc_century_y2k;  // Y2K-compliant real-time clock for current data ensemble
  uint8_t rtc_year_y2k;
  uint8_t rtc_month_y2k;
  uint8_t rtc_day_y2k;
  uint8_t rtc_hour_y2k;
  uint8_t rtc_minute_y2k;
  uint8_t rtc_second_y2k;
  uint8_t rtc_hundredth_y2k;

  // only available on pioneer
  uint8_t spare2;
  uint8_t leak_status;
  uint16_t leak_a_count;
  uint16_t leak_b_count;
  uint16_t tx_voltage;
  uint16_t tx_current;
  uint16_t transducer_impedence;
} __attribute__((packed));

#define RDI_PD0_ID_BOTTOMTRACK 0x0600
struct bottomtrack
{
  uint16_t headerid;
  uint16_t pings_per_ensemble;
  uint16_t delay;
  uint8_t corr_mag_min;
  uint8_t eval_amp_min;
  uint8_t percent_good_min;
  uint8_t mode;
  uint16_t err_vel_max;
  uint32_t reserved0;
  uint16_t range[4];
  int16_t velocity[4];
  uint8_t correlation[4];
  uint8_t eval_amp[4];
  uint8_t percent_good[4];
  uint16_t ref_min;  // minimum layer size, near boundary, and far boundary for BT water-reference layer
  uint16_t ref_near;
  uint16_t ref_far;
  uint16_t ref_velocity[4];
  uint8_t ref_correlation[4];
  uint8_t ref_intensity[4];
  uint8_t ref_percent_good[4];
  uint16_t depth_max;
  uint8_t rssi_amp[4];  // Receiver Signal Strength indicator
  uint8_t gain;
  uint8_t range_msb[4];
  uint32_t reserved1;
} __attribute__((packed));

#define RDI_PD0_ID_VELOCITY_HIGHRES 0x5803
struct velocity_highres
{
  uint16_t headerid;
  int32_t bt_velocity[4];
  int32_t bt_dmg[4];
  int32_t wm_velocity[4];
  int32_t wm_dmg[4];
  uint32_t sound_speed;
} __attribute__((packed));

#define RDI_PD0_ID_BOTTOMTRACK_RANGE 0x5804
struct bottomtrack_range {
  uint16_t headerid;
  int32_t slant_range;
  int32_t axis_delta_range;
  int32_t vertical_range;
  uint8_t pct_good_4beam;
  uint8_t pct_good_beam12;
  uint8_t pct_good_beam34;
  uint32_t raw_range[4];
  uint8_t raw_max_bt_filter[4];
  uint8_t raw_max_bt_amp[4];
} __attribute__((packed));

#define RDI_PD0_ID_NAV_PARAMS 0x2013
struct nav_parameters {
  uint16_t header_id;
  uint32_t time_to_bottom[4];
  uint16_t bottom_track_stddev[4];
  uint8_t shallow_operation;
  uint32_t time_to_watermass[4];
  uint16_t range_to_watermass;
  uint16_t watertrack_stddev[4];
  uint32_t bottomtrack_time_of_validity[4];
  uint32_t watertrack_time_of_validity[4];
} __attribute__((packed));

}  // rdidvl_structs

}  // ds_sensors

#endif  // DS_SENSORS_RDIDVL_STRUCTS
