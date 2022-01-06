//
// Created by ivandor on 8/7/19.
//
#include "ds_sensors/isd4000.h"
#include "ds_util/fofonoff_depth.h"

#include <boost/algorithm/string.hpp>
#include <sstream>
#include <ds_core_msgs/VoidCmd.h>
#include "ds_util/ds_util.h"

namespace ds_sensors
{
    isd4000::isd4000()
            : ds_sensors::DepthSensor()
    {
    }

    isd4000::isd4000(int argc, char* argv[], const std::string& name)
            : ds_sensors::DepthSensor(argc, argv, name)
    {
    }

    isd4000::~isd4000() = default;

    std::pair<bool, ds_sensor_msgs::DepthPressure> isd4000::parse_pressure(const ds_core_msgs::RawData& bytes)
    {
        /* ISD4000 series looks like $ISDPT, ddd.dddd,M,,ppp.pppp,B,tt.tt
                * where $ indicates the start of the message
                *       ISDPT is a five character mode ID
                *       ddd.dddd is some number of characters representing the depth value
                *       M is one character representing the units of depth
                *       ppp.pppp is some number of characters representing the pressure value
                *       B is one character representing the units of pressure value
                *       tt.tt is some number of characters representing the temperature value
                *       C is one character representing the units of temperature
                */

        if (bytes.data.size() < 6)
        {
            return { false, {} };
        }

        auto msg = ds_sensor_msgs::DepthPressure{};
        auto host = 0;
        auto id = 0;
        const auto n = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "%*18c%lf[^,]", &msg.pressure_raw);
        /*
               if (n != 1)
               {
                   // Try alternate parsing method for ISD4000 : THIS PARSES DEPTH DIRECTLY
                   const auto n = sscanf(reinterpret_cast<const char*>(bytes.data.data()), "%*c%*c%*c%*c%*c%*c%*c %lf [^,]", &msg.pressure_raw);
                   if (m != 1)
                       return { false, {} };
               }
               */
        // Set some sensible defaults
        msg.depth = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
        msg.latitude = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;
        msg.tare = ds_sensor_msgs::DepthPressure::DEPTH_PRESSURE_NO_DATA;

        msg.ds_header = bytes.ds_header;
        msg.header.stamp = msg.ds_header.io_time;
        return { true, msg };
    }

    std::pair<bool, ds_sensor_msgs::DepthPressure> isd4000::parse_message(const ds_core_msgs::RawData &bytes) {
        while (ros::ok()) {
            return parse_pressure(bytes);
        }
}


}  // namespace ds_sensors
