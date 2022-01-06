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
// Created by jvaccaro on 9/28/18.
// Based on data descriptions from (sentry-dav)
// Devices/Science/Velocimeter/Nortek_Vector/Device_Manuals/system-integrator-manual-1.pdf
//

#ifndef PROJECT_NORTEKVECTOR_STRUCTS_H
#define PROJECT_NORTEKVECTOR_STRUCTS_H

#include <string>

namespace ds_sensors
{
namespace nortekvector
{
struct inquiry
{
  std::string cmd = "II";
};

struct aquisition_to_command
{
  std::string interrupt = "@@@@@@";
  float wait_1 = 0.5;
  std::string cmd_1 = "K1W%!Q";
  float wait_2 = 3.0;
  std::string cmd_3 = "MC";
};

struct command_to_aquisition
{
  std::string cmd = "SR";
};

struct command_to_powerdown
{
  std::string cmd = "PD";
};

struct powerdown_to_command
{
  std::string interrupt = "@@@@@@";
  float wait_1 = 0.5;
  std::string cmd_1 = "K1W%!Q";
};
}

namespace nortekvector_structs
{
struct header
// Total Size 2 Bytes
{
  uint8_t sync;
  uint8_t id;
} __attribute__((packed));

struct clock
// Total Size 6 Bytes
{
  uint8_t minute;  // (BCD format)
  uint8_t second;  // (BCD format)
  uint8_t day;     // (BCD format)
  uint8_t hour;    // (BCD format)
  uint8_t year;    // (BCD format)
  uint8_t month;   // (BCD format)
} __attribute__((packed));

struct hardware_config
// Total Size 48 Bytes
{
  header hdr;  // 0x05
  uint8_t serialNo[14];
  uint16_t config;
  uint16_t frequency;  // kHz
  uint16_t PIC_version;
  uint16_t HW_revision;
  uint16_t rec_size;
  uint16_t status;
  uint8_t spare[12];
  uint32_t FW_version;
  uint16_t checksum;  // b58c + sum of all words in structure
} __attribute__((packed));

struct head_config
// Total Size 224 Bytes
{
  header hdr;          // 0x04
  uint16_t size;       // size of structure in words
  uint16_t config;     // bit 0: Pressure sensor (0=no, 1=yes)
                       // bit 1: Magnetometer sensor (0=no, 1=yes)
                       // bit 2: Tilt sensor (0=no, 1=yes)
                       // bit 3: Tilt sensor mounting (0=up, 1=down)
  uint16_t frequency;  // kHz
  uint16_t type;       // head type
  uint8_t serial_number[12];
  uint8_t system_data[176];
  uint8_t spare[22];
  uint16_t n_beams;
  uint16_t checksum;  // b58c + sum
} __attribute__((packed));

struct user_config
// Total Size 512 Bytes
{
  header hdr;             // 0x00
  uint16_t size;          // size of structure in words
  uint16_t t1;            // transmit pulse length
  uint16_t t2;            // blanking distance
  uint16_t t3;            // receive length
  uint16_t t4;            // time between pings
  uint16_t t5;            // time between burst sequences
  uint16_t n_pings;       // number of beam sequences per burst
  uint16_t avg_interval;  // average interval in seconds
  uint16_t n_beams;       // number of beams
  uint16_t tim_ctrl_reg;  // timing controller profile
  // bit 1: profile (0=single, 1=continuous)
  // bit 2: mode (0=burst, 1=continuous)
  // bit 5: power level (0=1, 1=2, 0=3, 1=4)
  // bit 6: power level (0 0 1 1 )
  // bit 7: synchout position (0=middle of
  // sample, 1=end of sample (Vector))
  // bit 8: sample on synch
  //(0=disabled,1=enabled, rising edge)
  // bit 9: start on synch (0=disabled,1=enabled,
  // rising edge)
  uint16_t a1;  // Not used
  uint16_t b0;  // Not used
  uint16_t b1;  // Not used
  uint16_t compass_udp_rate;
  uint16_t coord_sys;  // 0=ENU, 1=XYZ, 2=BEAM
  uint16_t n_bins;
  uint16_t bin_length;
  uint16_t meas_interval;
  uint8_t deploy_name[6];
  uint16_t wrap_mode;
  clock clock_deploy;
  uint32_t diag_interval;
  uint16_t mode;  // bit 0: use user specified sound speed (0=no,1=yes)
  // bit 1: diagnostics/wave mode 0=disable,  1=enable)
  // bit 2: analog output mode (0=disable,  1=enable)
  // bit 3: output format (0=Vector, 1=ADV)
  // bit 4: scaling (0=1 mm, 1=0.1 mm)
  // bit 5: serial output (0=disable, 1=enable)
  // bit 6: reserved EasyQ
  // bit 7: stage (0=disable, 1=enable)
  // bit 8: output power for analog input
  //    (0=disable, 1=enable)
  uint16_t adj_soundspeed;  // soundspeed adjustment factor
  uint16_t n_samp_diag;
  uint16_t n_beams_cell_diag;
  uint16_t n_pings_diag;
  uint16_t mode_test;  // bit 0: correct using DSP filter (0=no filter,  1=filter)
  // bit 1: filter data output (0=total corrected velocity,1=only correction part)
  uint16_t analog_input_address;
  uint16_t SW_version;
  uint16_t spare;
  uint8_t val_adjust_table[180];
  uint8_t comments[180];
  uint16_t mode_wave_measurement;  // bit 0: data rate (0=1 Hz, 1=2 Hz)
  // bit 1: wave cell position (0=fixed,1=dynamic)
  // bit 2: type of dynamic position (0=pct of  mean pressure, 1=pct of min re)
  uint16_t dynamic_positioning_percent;  // 32767 * ##% / 100
  uint16_t wave_transmit_pulse;
  uint16_t wave_blanking_distance;
  uint16_t wave_cell_size;
  uint16_t wave_n_samples;
  uint16_t a1_0;               // Not used
  uint16_t b0_0;               // Not used
  uint16_t samples_per_burst;  // samples per burst
  uint16_t spare_0;
  uint16_t analog_out_scale;
  uint16_t corr_threshold;
  uint16_t spare_1;
  uint16_t transmit_pulse_length;
  uint8_t spare_2[30];
  uint8_t qual_constant[16];
  uint16_t checksum;  // b58c + sum
} __attribute__((packed));

// struct aquadopp_data
//// Total size 42 Bytes
//{
//  header hdr; //0x01 for aquadopp_velocity_data, 0x80 for aquadopp_diagnostics_data
//  uint16_t size; //size of structure in words
//  clock clk;
//  uint16_t error;
//  uint16_t analog_in1;
//  uint16_t battery;
//  uint16_t analog_in2;
//  uint16_t heading;
//  uint16_t pitch;
//  uint16_t roll;
//  uint8_t pressure_MSB; //0.001dBar
//  uint8_t status;
//  uint16_t pressure_LSW; //0.001dBar
//  uint16_t temperature;
//  uint16_t vel1; //beam1 or X or east
//  uint16_t vel2; //beam2 or Y or north
//  uint16_t vel3; //beam3 or Z or up
//  uint8_t amp1; //beam1
//  uint8_t amp2; //beam2
//  uint8_t amp3; //beam3
//  uint8_t fill;
//  uint16_t checksum; //b58c + sum
//};
//
// struct aquadopp_diagnostics_data_header
//// Total size 36 Bytes
//{
//  header hdr; //06
//  uint16_t size; //size of structure in words
//  uint16_t records;
//  uint16_t cell;
//  uint8_t noise1; //beam 1
//  uint8_t noise2;
//  uint8_t noise3;
//  uint8_t noise4;
//  uint16_t proc_mag1;
//  uint16_t proc_mag2;
//  uint16_t proc_mag3;
//  uint16_t proc_mag4;
//  uint16_t distance1;
//  uint16_t distance2;
//  uint16_t distance3;
//  uint16_t distance;
//  uint8_t spare[6];
//  uint16_t checksum; //b58c + sum
//}__attribute__((packed));

struct vector_velocity_data_header
// Total Size 42 Bytes
{
  header hdr;     // 0x12
  uint16_t size;  // size of structure in words
  clock clk;
  uint16_t n_records;
  uint8_t noise1;  // beam 1
  uint8_t noise2;
  uint8_t noise3;
  uint8_t spare;
  uint8_t corr1;  // beam 1
  uint8_t corr2;
  uint8_t corr3;
  uint8_t spare_0;
  uint8_t spare_1[20];
  uint16_t checksum;  // b58c + sum
} __attribute__((packed));

struct vector_velocity_data
// Total Size 24 Bytes
{
  header hdr;  // ID = 10 //NO SIZE FIELD
  uint8_t analog_input2_LSB;
  uint8_t count;
  uint8_t pressure_MSB;
  uint8_t analog_input2_MSB;
  uint16_t pressure_LSW;
  uint16_t analog_input1;
  int16_t vel1;
  int16_t vel2;
  int16_t vel3;
  int8_t amp1;
  int8_t amp2;
  int8_t amp3;
  int8_t corr1;
  int8_t corr2;
  int8_t corr3;
  uint16_t checksum;  // b58c + sum
} __attribute__((packed));

struct vector_system_data
// Total Size 28 Bytes
{
  header hdr;     // 0x11
  uint16_t size;  // size of structure in words
  clock clk;
  int16_t battery;
  int16_t sound_speed;
  int16_t heading;
  int16_t pitch;
  int16_t roll;
  int16_t temperature;
  uint8_t error;
  uint8_t status;
  uint16_t analog_input;
  uint16_t checksum;  // b58c + sum
} __attribute__((packed));

struct vector_probe_check_data
// Variable
{
  header hdr;     // 0x07
  uint16_t size;  // size of structure in words
  uint16_t samples;
  uint16_t first_sample;
  uint8_t amp_B1[300];
  uint8_t amp_B2[300];
  uint8_t amp_B3[300];
  uint16_t checksum;  // b58c + sum
} __attribute__((packed));

}  // namespace
}  // namespace
#endif  // PROJECT_NORTEKVECTOR_STRUCTS_H