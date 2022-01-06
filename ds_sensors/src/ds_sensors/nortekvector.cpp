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
//

#include "ds_sensors/nortekvector.h"
#include "nortekvector_private.h"
#include "nortekvector_structs.h"

namespace ds_sensors
{
NortekVector::NortekVector() : SensorBase(), d_ptr_(std::unique_ptr<NortekVectorPrivate>(new NortekVectorPrivate))
{
}

NortekVector::NortekVector(int argc, char* argv[], const std::string& name)
  : SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<NortekVectorPrivate>(new NortekVectorPrivate))
{
}

NortekVector::~NortekVector() = default;

std::tuple<bool, bool, ds_sensor_msgs::Velocity3D, ds_sensor_msgs::NortekVectorSystem>
NortekVector::parse_bytes(const ds_core_msgs::RawData& bytes)
{
  bool ok_velocity = false;
  bool ok_system = false;
  auto velocity_msg = ds_sensor_msgs::Velocity3D{};
  auto system_msg = ds_sensor_msgs::NortekVectorSystem{};

  if (bytes.data.size() < 2)
  {
    ROS_ERROR_STREAM("Expected data size > 2, got: " << bytes.data.size());
    return std::tuple<bool, bool, ds_sensor_msgs::Velocity3D, ds_sensor_msgs::NortekVectorSystem>(
        false, false, velocity_msg, system_msg);
  }

  auto hdr = nortekvector_structs::header{};
  hdr.sync = bytes.data.data()[0];
  hdr.id = bytes.data.data()[1];

  if (hdr.sync != 0xa5)
  {
    ROS_ERROR("Expected sync value A5, got %x", hdr.sync);
    return std::tuple<bool, bool, ds_sensor_msgs::Velocity3D, ds_sensor_msgs::NortekVectorSystem>(
        false, false, velocity_msg, system_msg);
  }
  switch (static_cast<DATA_ID>(hdr.id))
  {
    case DATA_ID::VECTOR_VELO_DATA:
      std::tie(ok_velocity, velocity_msg) = parseVelocityData(bytes.data.data());
      break;
    case DATA_ID::VECTOR_SYSTEM_DATA:
      std::tie(ok_system, system_msg) = parseSystemData(bytes.data.data());
      break;
    default:
      ROS_ERROR("Expected ID value match, got %x", hdr.id);
      break;
  }
  if (ok_velocity)
  {
    velocity_msg.ds_header = bytes.ds_header;
    velocity_msg.header.stamp = velocity_msg.ds_header.io_time;
  }
  else if (ok_system)
  {
    system_msg.ds_header = bytes.ds_header;
    system_msg.header.stamp = system_msg.clk;
  }
  return std::tuple<bool, bool, ds_sensor_msgs::Velocity3D, ds_sensor_msgs::NortekVectorSystem>(
      ok_velocity, ok_system, velocity_msg, system_msg);
}

void NortekVector::setupPublishers()
{
  SensorBase::setupPublishers();
  DS_D(NortekVector);
  d->velocity_3D_pub_ =
      nodeHandle().advertise<ds_sensor_msgs::Velocity3D>(ros::this_node::getName() + "/velocity_3D", 10);
  d->system_pub_ =
      nodeHandle().advertise<ds_sensor_msgs::NortekVectorSystem>(ros::this_node::getName() + "/system", 10);
}

void NortekVector::setupServices()
{
  SensorBase::setupServices();
  DS_D(NortekVector);

  d->set_mode_acquisition_ =
      nodeHandle().advertiseService<ds_core_msgs::VoidCmd::Request, ds_core_msgs::VoidCmd::Response>(
          ros::this_node::getName() + "/set_mode_acquisition",
          boost::bind(&NortekVector::set_mode_acquisition, this, _1, _2));
  d->set_mode_command_ = nodeHandle().advertiseService<ds_core_msgs::VoidCmd::Request, ds_core_msgs::VoidCmd::Response>(
      ros::this_node::getName() + "/set_mode_command", boost::bind(&NortekVector::set_mode_command, this, _1, _2));
  d->set_mode_shutdown_ =
      nodeHandle().advertiseService<ds_core_msgs::VoidCmd::Request, ds_core_msgs::VoidCmd::Response>(
          ros::this_node::getName() + "/set_mode_shutdown",
          boost::bind(&NortekVector::set_mode_shutdown, this, _1, _2));
  d->send_command_ = nodeHandle().advertiseService<ds_core_msgs::StringCmd::Request, ds_core_msgs::StringCmd::Response>(
      ros::this_node::getName() + "/send_command", boost::bind(&NortekVector::send_command, this, _1, _2));
}

void NortekVector::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
{
  auto ok_velocity = false;
  auto ok_system = false;
  auto msg_velocity = ds_sensor_msgs::Velocity3D{};
  auto msg_system = ds_sensor_msgs::NortekVectorSystem{};

  std::tie(ok_velocity, ok_system, msg_velocity, msg_system) = NortekVector::parse_bytes(bytes);

  if (!ok_velocity && !ok_system)
  {
    return;
  }

  DS_D(NortekVector);
  if (ok_velocity)
  {
    FILL_SENSOR_HDR_IOTIME(msg_velocity, msg_velocity.ds_header.io_time);
    d->velocity_3D_pub_.publish(msg_velocity);
  }
  if (ok_system)
  {
    FILL_SENSOR_HDR_IOTIME(msg_system, msg_system.ds_header.io_time);
    d->system_pub_.publish(msg_system);
  }
}

std::pair<bool, ds_sensor_msgs::Velocity3D> NortekVector::parseVelocityData(const uint8_t* raw)
{
  auto msg = ds_sensor_msgs::Velocity3D{};
  auto data = reinterpret_cast<const nortekvector_structs::vector_velocity_data*>(raw);

  if (!data)
  {
    ROS_ERROR_STREAM("Cast to Vector Velocity Data failed");
    return { false, msg };
  }
  if (!checksumPassed(22, raw, data->checksum))
  {
    return { false, msg };
  }

  msg.count = data->count;
  msg.pressure = data->pressure_LSW * 0.001 + data->pressure_MSB * 65.536;
  msg.analog_input1 = data->analog_input1;
  msg.analog_input2 = data->analog_input2_LSB + (data->analog_input2_MSB) << 8;
  msg.vel = { static_cast<double>(data->vel1) * 0.001, static_cast<double>(data->vel2) * 0.001,
              static_cast<double>(data->vel3) * 0.001 };
  msg.amp = { data->amp1, data->amp2, data->amp3 };
  msg.corr = { data->corr1, data->corr2, data->corr3 };

  return { true, msg };
}

std::pair<bool, ds_sensor_msgs::NortekVectorSystem> NortekVector::parseSystemData(const uint8_t* raw)
{
  auto msg = ds_sensor_msgs::NortekVectorSystem{};
  auto data = reinterpret_cast<const nortekvector_structs::vector_system_data*>(raw);

  if (!data)
  {
    ROS_ERROR_STREAM("Cast to Vector System Data failed");
    return { false, msg };
  }
  if (!checksumPassed(26, raw, data->checksum))
  {
    return { false, msg };
  }

  // Convert time from BCD to ros::Time
  uint16_t second = ds_util::bcd_to_int(data->clk.second);
  uint16_t minute = ds_util::bcd_to_int(data->clk.minute);
  uint16_t hour = ds_util::bcd_to_int(data->clk.hour);
  uint16_t day = ds_util::bcd_to_int(data->clk.day);
  uint16_t month = ds_util::bcd_to_int(data->clk.month);

  uint16_t year = 2000 + ds_util::bcd_to_int(data->clk.year);
  if (second > 60 || minute > 60 || hour > 24 || day > 31 || month < 1 || month > 12 || month < 1)
  {
    ROS_ERROR_STREAM("failure--invalid timestamp");
    return { false, msg };
  }
  boost::posix_time::ptime t1(boost::gregorian::date(year, month, day),
                              boost::posix_time::time_duration(hour, minute, second));
  msg.clk = ros::Time::fromBoost(t1);

  // Populate the data fields
  msg.battery = data->battery * 0.1;
  msg.sound_speed = data->sound_speed * 0.1;
  msg.heading = data->heading * 0.1;
  msg.pitch = data->pitch * 0.1;
  msg.roll = data->roll * 0.1;
  msg.temperature = data->temperature * 0.01;
  msg.error = data->error;
  msg.status = data->status;

  return { true, msg };
}

bool NortekVector::checksumPassed(uint16_t length, const uint8_t* raw, uint16_t chksum)
{
  uint16_t sum = 0xb58c;
  for (int i = 0; i < length; i += 2)
  {
    sum += raw[i + 1] * 256 + raw[i];
  }

  if (sum != chksum)
  {
    ROS_ERROR_STREAM("Checksum failed: sum " << sum << " chksum " << chksum);
  }
  return (sum == chksum);
}

bool NortekVector::set_mode_acquisition(const ds_core_msgs::VoidCmd::Request& req,
                                        ds_core_msgs::VoidCmd::Response& resp)
{
  DS_D(NortekVector);
  std::string old_mode = d->mode_;

  // If current mode is unknown, then assume shutdown
  if (d->mode_ == "shutdown" || d->mode_ == "")
  {
    sendCommand("@@@@@@", "instrument");
    ros::Duration(0.5).sleep();
    sendCommand("K1W%!Q", "instrument");
    ros::Duration(0.5).sleep();
    d->mode_ = "command";
  }
  if (d->mode_ == "command")
  {
    sendCommand("SR", "instrument");
    ros::Duration(0.5).sleep();
    d->mode_ = "acquisition";
  }
  resp.msg = "Changed mode from " + old_mode + " to " + d->mode_;
  resp.success = true;
  return true;
}

bool NortekVector::set_mode_command(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp)
{
  DS_D(NortekVector);
  std::string old_mode = d->mode_;
  if (d->mode_ == "shutdown")
  {
    sendCommand("@@@@@@", "instrument");
    ros::Duration(0.5).sleep();
    sendCommand("K1W%!Q", "instrument");
    ros::Duration(0.5).sleep();
    d->mode_ = "command";
  }
  // If current mode is unknown, then assume acquisition
  if (d->mode_ == "acquisition" || d->mode_ == "")
  {
    sendCommand("@@@@@@", "instrument");
    ros::Duration(0.5).sleep();
    sendCommand("K1W%!Q", "instrument");
    ros::Duration(3.0).sleep();
    sendCommand("MC", "instrument");
    ros::Duration(0.5).sleep();
    d->mode_ = "command";
  }
  resp.msg = "Changed mode from " + old_mode + " to " + d->mode_;
  resp.success = true;
  return true;
}

bool NortekVector::set_mode_shutdown(const ds_core_msgs::VoidCmd::Request& req, ds_core_msgs::VoidCmd::Response& resp)
{
  DS_D(NortekVector);
  std::string old_mode = d->mode_;

  // If current mode is unknown, then assume acquisition
  if (d->mode_ == "acquisition" || d->mode_ == "")
  {
    sendCommand("@@@@@@", "instrument");
    ros::Duration(0.5).sleep();
    sendCommand("K1W%!Q", "instrument");
    ros::Duration(3.0).sleep();
    sendCommand("MC", "instrument");
    ros::Duration(0.5).sleep();
    d->mode_ = "command";
  }
  if (d->mode_ == "command")
  {
    sendCommand("PD", "instrument");
    ros::Duration(0.5).sleep();
    d->mode_ = "shutdown";
  }
  resp.msg = "Changed mode from " + old_mode + " to " + d->mode_;
  resp.success = true;
  return true;
}

bool NortekVector::send_command(const ds_core_msgs::StringCmd::Request& req, ds_core_msgs::StringCmd::Response& resp)
{
  // First send the command
  sendCommand(req.cmd, "instrument");

  // Then evaluate whether it was probably sent based on current mode
  DS_D(NortekVector);
  if (d->mode_ == "")
  {
    resp.msg = "Current mode unknown--set to command mode";
  }
  else if (d->mode_ != "command")
  {
    resp.msg = "Current mode " + d->mode_ + "--set to command mode";
  }
  else if (d->mode_ == "command")
  {
    resp.msg = "Current mode is command";
    resp.success = true;
  }
  return true;
}

}  // namespace