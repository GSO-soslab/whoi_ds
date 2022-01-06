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
///
/// # ROS node for running sensors defined in ds_sensor
///
/// ## Adding/Removing sensors:
///
/// To add or remove a sensor to the `sensor_node` executable you must do two things:
///    - Add/Remove an entry to the SENSORS list with the sensor name and description
///    - Add/Remove an entry in the 'if' case in main for your new sensor.
///
#include "ds_sensors/ds_sensors.h"
#include "sensor_node_utils.h"
#include <boost/program_options.hpp>
#include <memory>
namespace po = boost::program_options;

static const auto SENSORS =
    std::list<std::pair<std::string, std::string>>{ { "anderaa_optode4831", "Anderaa Oxygen Optode 4330/4835/4831" },
                                                    { "aps1540", "Applied Physics 1540 vector magnetometer" },
                                                    { "paro", "Paro Scientific DigiQuartz pressure sensor" },
                                                    { "isd4000", "Impact Subsea ISD4000 Depth Sensor"},
                                                    { "sbe49", "SeaBird SBE49 CTD" },
                                                    { "rdidvl", "Teledyne RD Instruments Doppler velocity log" },
                                                    { "nortekdvl", "Nortek Doppler Velocity Log 1000"},
                                                    { "reson_svp70", "Reson SVP 70/71 Sound Velocity Probe" },
                                                    { "phins", "IxBlue Phins Inertial Navigation System" },
                                                    { "phinsbin", "IxBlue Phins Inertial Navigation System Binary Interface" },
                                                    { "3dmgx5", "Lord Microstrain 3DM-GX25 AHRS/IMU" },
                                                    { "remus_htpgl", "Remus HTPGL Environmental Monitor Board" },
                                                    { "xeos_gps", "Xeos GPS" },
                                                    { "nortekvector", "Nortek Vector Velicimeter" },
                                                    { "sbe03", "SeaBird SBE03 temperature sensor"},
						    { "sonardyne_usbl", "USBL positioning system"}
                                                    };

void print_usage(const std::string& name, const po::options_description& options)
{
  std::cout << "Usage:  " << name << " sensor --node [ROS_OPTIONS]" << std::endl
            << std::endl
            << "Starts a node named 'sensor' for the desired sensor" << std::endl
            << std::endl
            << options << std::endl;

  std::cout << "ROS_OPTIONS:  These can be used to override ROS-specific parameters" << std::endl
            << "  __name:=$NAME         Override the default node name" << std::endl;

  std::cout << "EXAMPLE:" << std::endl
            << "  start a node named 'primary_depth' under the namespace 'devices' using the 'paro' sensor:"
            << std::endl
            << "    env ROS_NAMESPACE=devices rosrun ds_sensors sensor paro __name:=primary_depth" << std::endl
            << std::endl;
}

int main(int argc, char* argv[])
{
  //
  //  Setting up our command line parsing
  //

  po::options_description core("Core options");
  core.add_options()("sensor,s", po::value<std::string>(), "sensor type");

  po::options_description info("Info options");
  info.add_options()("help,h", "print this help message")("list-sensors", po::bool_switch()->default_value(false),
                                                          "Show list of available sensors");

  po::options_description connections("Connection options");
  connections.add_options()("serial", "Add a serial connection.  Format is $PORT:$BYTE$PARITY$STOP (e.g. "
                                      "/dev/ttyS0:8N1")("udp", "Add a udp connection.  Format is "
                                                               "$LISTEN_IFACE:LISTEN_PORT,$DEST_IP:$DEST_PORT (e.g. "
                                                               "0.0.0.0:5000,192.168.100.100:5000");

  po::options_description all("Options");
  all.add(core)
      //  .add(connections)
      .add(info);

  // Positional arguments
  po::positional_options_description p;
  p.add("sensor", 1);

  // Call ros::init now.  It'll handle any remappings from roslaunch or the like and remove all ros-specific
  // command line arguments.
  ros::init(argc, argv, "sensor");

  // Now try parsing our command line after ros has stripped out it's own parameters
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(all).positional(p).run(), vm);

  po::notify(vm);

  //
  // Start checking command line arguments
  //
  if (vm.count("help"))
  {
    print_usage(argv[0], info);
    return 0;
  }

  //
  // Print a list of available sensors
  //
  if (vm["list-sensors"].as<bool>())
  {
    std::cout << std::left << "Available sensors:" << std::endl << std::endl;

    std::cout << std::left << std::setw(20) << "sensor name:" << std::left << std::setw(50)
              << "description:" << std::endl;

    for (auto& sensor_pair : SENSORS)
    {
      std::cout << std::left << std::setw(20) << sensor_pair.first << std::left << std::setw(50) << sensor_pair.second
                << std::endl;
    }

    return 0;
  }

  //
  // Handle missing required arguments
  //
  if (vm["sensor"].empty())
  {
    std::cerr << "ERROR:  No sensor specified.\n"
              << "ERROR:  Use '--list-sensors' and choose a valid sensor." << std::endl
              << std::endl;
    print_usage(argv[0], info);
    return 1;
  }

  //
  // Everything seems good, start initializing the node.
  //

  // Create an empty unique pointer to a ds_sensors::SensorBase instance.  We'll use this to hold our
  // actual sensor class
  auto node = std::unique_ptr<ds_base::SensorBase>{};

  // Figure out our sensor type and the name of the node.
  const auto sensor = vm["sensor"].as<std::string>();

  // Look up our sensor.  There's better ways of doing this, but this is the easiest.
  if (sensor == "sbe49")
  {
    node.reset(new ds_sensors::Sbe49());
  }
  else if (sensor == "aps1540")
  {
    node.reset(new ds_sensors::Aps1540());
  }
  else if (sensor == "paro")
  {
    node.reset(new ds_sensors::ParoDigiquartz());
  }
  else if (sensor == "isd4000")
  {
    node.reset(new ds_sensors::isd4000());
  }
  else if (sensor == "reson_svp70")
  {
    node.reset(new ds_sensors::ResonSvp70());
  }
  else if (sensor == "rdidvl")
  {
    node.reset(new ds_sensors::RdiDvl());
  }
  else if (sensor == "nortekdvl")
  {
    node.reset(new ds_sensors::NortekDvl());
  }
  else if (sensor == "anderaa_optode4831")
  {
    node.reset(new ds_sensors::AnderaaOxyOptode());
  }
  else if (sensor == "phins")
  {
    node.reset(new ds_sensors::Phins());
  }
  else if (sensor == "phinsbin")
  {
    node.reset(new ds_sensors::PhinsBin());
  }
  else if (sensor == "3dmgx5")
  {
    node.reset(new ds_sensors::MicrostrainAhrs());
  }
  else if (sensor == "xeos_gps")
  {
    node.reset(new ds_sensors::XeosGps());
  }
  else if (sensor == "remus_htpgl")
  {
    node.reset(new ds_sensors::RemusHtpgl());
  }
  else if (sensor == "sbe03")
  {
    node.reset(new ds_sensors::Sbe03());
  }
  else if (sensor == "nortekvector")
  {
    node.reset(new ds_sensors::NortekVector());
  }
  else if (sensor == "sonardyne_usbl")
  {
    node.reset(new ds_sensors::SonardyneUsbl());
  }
  else
  {
    std::cerr << "ERROR: Unknown sensor specified: " << sensor << std::endl;
    std::cerr << "ERROR: Use '--list-sensors' and choose a valid sensor." << std::endl;
    return 1;
  }

  // Start running.
  node->run();

  // Return success.
  return 0;
}
