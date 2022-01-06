//
// Created by ivandor on 8/7/19.
//

#ifndef DS_SENSORS_ISD4000_H
#define DS_SENSORS_ISD4000_H
#include "depthsensor.h"

namespace ds_sensors
{
    struct isd4000Private;

/// @brief Sensor class for the Impact Subsea ISD4000 depth sensor
///
/// Reads `OutputType=3` messages with 3 or 5 fields.
///
/// Topic       Message Type                     Description
/// ----------- ----------------------------     ----------------------------------
/// $name/raw   ds_core_msgs::RawData            all raw bytes received from sensor
/// $name/depth ds_sensor_msgs::DepthPressure    successfully parsed messages
    class isd4000 : public ds_sensors::DepthSensor
    {
    public:
        explicit isd4000();
        isd4000(int argc, char* argv[], const std::string& name);
        ~isd4000() override;

        /// @brief Parse a pressure reading message.
        ///
        /// This does *not* populate the entire DepthPressure message.  Quantities such as latitude, pressure unit,
        /// and tare value are required and are not part of the pressure record.
        ///
        /// \param bytes
        /// \return
        static std::pair<bool, ds_sensor_msgs::DepthPressure> parse_pressure(const ds_core_msgs::RawData& bytes);

    protected:
        // simply a virtually-bound wrapper around parse_pressure
        std::pair<bool, ds_sensor_msgs::DepthPressure> parse_message(const ds_core_msgs::RawData& bytes) override;
    };

}  // namespace ds_sensors
#endif //DS_SENSORS_ISD4000_H
