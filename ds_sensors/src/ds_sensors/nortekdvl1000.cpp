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

#include "ds_sensors/nortekdvl1000.h"
#include "nortekdvl1000_private.h"
#include <sstream>

using namespace boost::posix_time;
using namespace boost::gregorian;

/// ds_sensors/src/ds_sensors/nortekdvl1000.cpp
/// =============================================
/// DATE ------ WHO -------- WHAT ---------------
/// 7-23-19    I Vandor    Created and written

namespace ds_sensors
{
    NortekDvl::NortekDvl() : ds_base::SensorBase(), d_ptr_(std::unique_ptr<NortekDvlPrivate>(new NortekDvlPrivate))
    {
    }

    NortekDvl::NortekDvl(int argc, char* argv[], const std::string& name)
            : ds_base::SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<NortekDvlPrivate>(new NortekDvlPrivate))
    {
    }

    NortekDvl::~NortekDvl() = default;

    double NortekDvl::seconds_from_epoch(ptime const& t) {
        boost::posix_time::ptime const EPOCH(date(1970, 1, 1));
        boost::posix_time::time_duration delta(t-EPOCH);
        return (delta.total_microseconds() / 1000000.0);
    }

    std::tuple<bool, ds_sensor_msgs::NortekDF21>
    NortekDvl::parse_bytes(const ds_core_msgs::RawData& bytes, double beam_angle, bool phased_array, double max_clock_offset)
    /// creates parsed dvldata and Nortek df21 messages. Do not use returned messages when success=false
    {
        bool success = false;
        auto df21 = ds_sensor_msgs::NortekDF21{};

        if (bytes.data.size() < 2)
        {
            // Check for zero or small size
            ROS_ERROR_STREAM("BYTES DATA SIZE LESS THAN 2");
            return std::make_tuple(false, df21);
        }

        // fill in DVL type field so that subsequent processing can use it
        if (phased_array) {
            df21.dvl_type = ds_sensor_msgs::NortekDF21::DVL_TYPE_PHASED_ARRAY;
        } else {
            df21.dvl_type = ds_sensor_msgs::NortekDF21::DVL_TYPE_PISTON;
        }
        // Create parsed df21 message. Only fails when bad header read,
        // checksum fails, or unknown binary data header.
        // INFO already fed out.
        success = parseHeaderID(bytes, &df21);
        if (!success) {
            return std::make_tuple(false, df21);
        }

        // Determine the authoratative timestamp for this message
        ros::Duration dt = df21.ds_header.io_time - df21.header.stamp;
        //ROS_INFO_STREAM("   DF21 Time: " <<std::fixed <<df21.header.stamp.toSec());
        //ROS_INFO_STREAM("     IO Time: " <<std::fixed <<df21.ds_header.io_time.toSec());
        //ROS_INFO_STREAM("       Delta: " <<std::fixed <<dt.toSec());
        if (fabs(dt.toSec()) > max_clock_offset) {
          // If the timestamps are wildly different, slam it back into place using I/O time
          // by lin: not print for right now
            // ROS_WARN_STREAM("DVL clock differs from CPU clock by " << dt.toSec() << " seconds (threshold: "
                        //    <<max_clock_offset <<"); using I/O times");

            df21.header.stamp = df21.ds_header.io_time;
        }

        return std::make_tuple(success, df21);
    }

    bool NortekDvl::parseHeaderID(const ds_core_msgs::RawData& bytes, ds_sensor_msgs::NortekDF21* big_msg)
    /// uses header ID to call specified parsing functions.
    {
        size_t buf_len = bytes.data.size();
        const uint8_t* buffer = bytes.data.data();
        bool ok = true;
        uint32_t sync = (buffer[0]);
        if (sync == 0xa5) {
            // Match on HEADER
            auto *hdr = reinterpret_cast<const ds_sensors::nortekdvl_structs::header *> (buffer);
            auto length = hdr->header_size;
            auto payload = buffer + length;
            size_t payload_len = hdr->data_size;

            if (buf_len < sizeof(hdr)) {
                ROS_ERROR_STREAM("Header is too short. Header is: " << hdr->header_size);
                return false;
            }
            if (hdr->header_size > 10) {
                ROS_ERROR_STREAM("Header result failure: too many data: " << hdr->header_size);
                return false;
            }
            if (!NortekDvl::checksum(hdr->data_size, buffer)) {
                ROS_ERROR_STREAM("Header result failure: Checksum failed");
                return false;
            }
            if (hdr->headerid == 0x1b) {
                auto *bt = reinterpret_cast<const nortekdvl_structs::bottomtrack*>(payload);
                if (payload_len > buf_len) {
                    ROS_ERROR_STREAM("Payload length longer that data received");
                }
                NortekDvl::bt_to_msg(bytes.ds_header.io_time, *bt, big_msg);
                return true;
            } else {
                ROS_ERROR_STREAM("Header ID Not Recognized: " << hdr->headerid);
                return false;
            }
        }
        ROS_ERROR_STREAM("Nortek DVL Sync ID not recognized: " << sync);
        return false;
    }

    void NortekDvl::msg_to_rng(ds_sensor_msgs::Ranges3D* rngdata, ds_sensor_msgs::NortekDF21* big_msg, double beam_angle, bool phased_array)
    {
        // copy headers from big_msg
        rngdata->header = big_msg->header; 
        rngdata->ds_header = big_msg->ds_header; 

        rngdata->ranges.resize(4);
        double range[4], xy_range[4];

        if (phased_array) {
            rngdata->soundspeed_correction_type = rngdata->SOUNDSPEED_CORRECTION_PHASEDARRAYDVL;
        } else {
            rngdata->soundspeed_correction_type = rngdata->SOUNDSPEED_CORRECTION_NORMAL;
        }
        // Z component for each beam is - the altitude
        for (int i = 0; i < 4; ++i)
        {
            rngdata->ranges[i].range_quality = big_msg->fomBeam[i];
            double beam_range = big_msg->distBeam[i];

            if (beam_range == 0)
                rngdata->ranges[i].range_validity = ds_sensor_msgs::Range3D::RANGE_INDETERMINANT;
            else
                rngdata->ranges[i].range_validity = ds_sensor_msgs::Range3D::RANGE_VALID;
            rngdata->ranges[i].range.point.z = -beam_range;
            range[i] = beam_range / cos(beam_angle);
            xy_range[i] = range[i] * sin(beam_angle);
        }

        rngdata->ranges[0].range.point.x = -xy_range[0];
        rngdata->ranges[0].range.point.y = 0.0;
        rngdata->ranges[1].range.point.x = +xy_range[1];
        rngdata->ranges[1].range.point.y = 0.0;
        rngdata->ranges[2].range.point.x = 0.0;
        rngdata->ranges[2].range.point.y = +xy_range[2];
        rngdata->ranges[3].range.point.x = 0.0;
        rngdata->ranges[3].range.point.y = -xy_range[3];
    }

    void NortekDvl::msg_to_dvl(ds_sensor_msgs::Dvl* dvldata, ds_sensor_msgs::NortekDF21* big_msg, double beam_angle, bool phased_array)
/// extracts/converts DF21 message into slimmer DVLData message
    {

        // copy headers from big_msg
        dvldata->header = big_msg->header; 
        dvldata->ds_header = big_msg->ds_header; 

        dvldata->dvl_time = big_msg->dvl_time;
        //    dvldata->ds_header.io_time = ros::Time::now();
        //    dvldata->ds_header.source_uuid = "b6c54489-38a0-5f50-a60a-fd8d76219cae";

        if (phased_array) {
            dvldata->dvl_type = ds_sensor_msgs::Dvl::DVL_TYPE_PHASED_ARRAY;
        } else {
            dvldata->dvl_type = ds_sensor_msgs::Dvl::DVL_TYPE_PISTON;
        }

        dvldata->speed_sound = big_msg->speed_sound;
        dvldata->num_good_beams = big_msg->good_beams;

        for (int i = 0; i < 9; i++) {
            dvldata->velocity_covar[i] = -1;
        }
        dvldata->velocity_mode = dvldata->DVL_MODE_BOTTOM;  // Bottom tracking mode
        //dvldata->coordinate_mode = big_msg->coord_mode;

        // beam azimuth is defined as math-like (start at X, rotate around positive Z out of the page)
        // as it matches the drawing on page 18 of the DVL Operations Guide from Nortek
        double beam_azimuth[] = {M_PI/4.0, -M_PI/4.0, -3.0*M_PI/4.0, 3.0*M_PI/4.0};
        for (int i = 0; i < 4; i++) {
            // This geometry will be needed for visualizing in RVIZ
            // the beams are a 30-60-90 triangle, with the sqrt(3)/2 size on z
            // (filled out here) and the 0.5 side in the reference frame of the
            // DVL (filled out further below)
            // In the mean time, we zero all the other components.
            dvldata->beam_unit_vec[i].x = sin(beam_angle)*cos(beam_azimuth[i]);
            dvldata->beam_unit_vec[i].y = sin(beam_angle)*sin(beam_azimuth[i]);
            dvldata->beam_unit_vec[i].z = cos(beam_angle);

            dvldata->raw_velocity[i] = big_msg->velBeam[i];
            dvldata->raw_velocity_covar[i] = -1;

            dvldata->beam_quality[i]= big_msg->fomBeam[i];
            dvldata->range[i] = big_msg->distBeam[i]  / cos(beam_angle);
            dvldata->range_covar[i] = -1;
            
            
        }
	//fill in velocity
	dvldata->velocity.x = big_msg->velX;
	dvldata->velocity.y = big_msg->velY;
	if (big_msg->velZ1 == -32.768f) {
	  dvldata->velocity.z = big_msg->velZ2;
	} else if (big_msg->velZ2 == -32.768f) {
      dvldata->velocity.z = big_msg->velZ1;
	} else {
      dvldata->velocity.z = ((big_msg->velZ1 + big_msg->velZ2)/2.0);
	}

        dvldata->altitude = big_msg->altitude_sum / big_msg->good_beams;
        dvldata->speed_gnd = big_msg->speed_gnd;
        dvldata->course_gnd = big_msg->course_gnd;


    }

///*-----------------------------------------------------------------------------*///
///*   DATA CONVERSION PARSERS: generate DF21 from memory parsers                 *///
///*-----------------------------------------------------------------------------*///
    void NortekDvl::bt_to_msg(const ros::Time& io_t, const nortekdvl_structs::bottomtrack& bt, ds_sensor_msgs::NortekDF21* big_msg)
/// extracts/converts data into DF21 message from memory-parsed bottomtrack
    {
        for (int i = 0; i < 4; i++)
        {
            big_msg->distBeam[i] = bt.distBeam[i];
            if (big_msg->distBeam[i] != 0 && bt.velBeam[i] != -32.768f && bt.fomBeam[i] != 10)
                big_msg->good_beams += 1;
            big_msg->altitude_sum += big_msg->distBeam[i];
            if (bt.velBeam[i] != -32.768f)
                big_msg->velBeam[i] = bt.velBeam[i];

            big_msg->fomBeam[i] = bt.fomBeam[i];
            big_msg->timeDiff1Beam[i] = bt.timeDiff1Beam[i];
            big_msg->timeDiff2Beam[i] = bt.timeDiff2Beam[i];
            big_msg->timeVelEstBeam[i] = bt.timeVelEstBeam[i];
        }
        big_msg->version = bt.version;
        big_msg->offsetOfData = bt.data_offset;
        big_msg->serialNumber = bt.serial_num;
        big_msg->year = bt.year;
        //ROS_WARN_STREAM("Not-Adjusted year is: " << static_cast<int>(big_msg->year));
        big_msg->month = bt.month;
        //ROS_WARN_STREAM("Month is: " << static_cast<int>(big_msg->month));
        big_msg->day = bt.day;
        big_msg->hour = bt.hour;
        big_msg->minute = bt.minute;
        big_msg->seconds = bt.seconds;
        big_msg->microSeconds = bt.microseconds; // actually sent as 100-microsecond counts
        //big_msg->nBeams = bt.nbeams;
        big_msg->nBeams = big_msg->good_beams;

        int year = static_cast<int>(big_msg->year) + 1900;
        int month = static_cast<int>(big_msg->month) + 1;

        // Assemble the time as reported by the DVL
        boost::posix_time::ptime msgtime(boost::gregorian::date(year, month, big_msg->day),
                                         boost::posix_time::hours(big_msg->hour) + boost::posix_time::minutes(big_msg->minute) +
                                         boost::posix_time::seconds(big_msg->seconds) +
                                         boost::posix_time::microseconds(static_cast<int>(big_msg->microSeconds)*100));
        ros::Time msg_rostime = ros::Time::fromBoost(msgtime);
        big_msg->dvl_time = static_cast<double>(msg_rostime.sec) + static_cast<double>(msg_rostime.nsec)*1.0e-9;

        // we have time (finally), so let's fill in our headers
        // let's stash our time for now, we'll fill it in later
        big_msg->header.stamp = msg_rostime;
        big_msg->ds_header.io_time = io_t; // also fill in the IO time, to handle possible timestamp drift

        //Basic Error Handling
        big_msg->error = bt.error;
        if (big_msg->error != 0) {
            ROS_WARN_STREAM("Error Message is: "<<big_msg->error);
        }

        // Basic Status Message Handler
        //TODO: Update this with status struct
        big_msg->status = bt.status;
        if (big_msg->status & 0x0e000000) {
            ROS_WARN_STREAM("DVL processing capacity alarm");
        }

        big_msg->speed_sound = bt.speed_sound;
        big_msg->temperature = bt.temperature;
        big_msg->pressure = bt.pressure;
        big_msg->velX = bt.velX;
        big_msg->velY = bt.velY;
        big_msg->velZ1 = bt.velZ1;
        big_msg->velZ2 = bt.velZ2;
        big_msg->fomX = bt.fomX;
        big_msg->fomY = bt.fomY;
        big_msg->fomZ1 = bt.fomZ1;
        big_msg->fomZ2 = bt.fomZ2;
        big_msg->timeDiff1X = bt.timeDiff1X;
        big_msg->timeDiff1Y = bt.timeDiff1Y;
        big_msg->timeDiff1Z1 = bt.timeDiff1Z1;
        big_msg->timeDiff1Z2 = bt.timeDiff1Z2;
        big_msg->timeDiff2X = bt.timeDiff2X;
        big_msg->timeDiff2Y = bt.timeDiff2Y;
        big_msg->timeDiff2Z1 = bt.timeDiff2Z1;
        big_msg->timeDiff2Z2 = bt.timeDiff2Z2;
        big_msg->timeVelEstX = bt.timeVelEstX;
        big_msg->timeVelEstY = bt.timeVelEstY;
        big_msg->timeVelEstZ1 = bt.timeVelEstZ1;
        big_msg->timeVelEstZ2 =  bt.timeVelEstZ2;

        big_msg->speed_gnd = sqrt(big_msg->velX * big_msg->velX + big_msg->velY * big_msg->velY);
        big_msg->course_gnd = atan2(big_msg->velX, big_msg->velY) * 180.0 / M_PI;
    }

///*-----------------------------------------------------------------------------*///
///*   CHECKSUM                                                                  *///
///*-----------------------------------------------------------------------------*///
    bool NortekDvl::checksum(uint16_t length, const uint8_t* buffer)
//    Buffer up until checksum is length bytes long.
    {
        uint16_t chksum = 0xB58C;
        uint16_t nbshorts = (length >> 1);
        for (int i = 0; i < nbshorts; i++)
        {
            chksum += *buffer;
            length -= 2;
            buffer++;
        }
        if (length > 0) {
            chksum += ((uint16_t)(*buffer)) << 8;
        }
        return chksum;
    }

    void NortekDvl::setupPublishers()
    {
        SensorBase::setupPublishers();
        DS_D(NortekDvl);
        auto nh = nodeHandle();
        d->dvl_pub_ = nh.advertise<ds_sensor_msgs::Dvl>(ros::this_node::getName() + "/dvl", 10);
        d->df21_pub_ = nh.advertise<ds_sensor_msgs::NortekDF21>(ros::this_node::getName() + "/df21", 10);
        d->ranges_pub_ = nh.advertise<ds_sensor_msgs::Ranges3D>(ros::this_node::getName() + "/ranges", 1);
    }

    void NortekDvl::setupParameters()
    {
        SensorBase::setupParameters();

        DS_D(NortekDvl);

        d->beam_angle = ros::param::param<double>("~/beam_angle_deg", 30) * M_PI / 180.0;
        d->phased_array = ros::param::param<bool>("~phased_array", false);
        d->max_clock_offset = ros::param::param<double>("max_clock_offset", 0.5);
    }
///*-----------------------------------------------------------------------------*///
///*   TOP FUNCTIONS: receive incoming raw data, create messages, and publish    *///
///*-----------------------------------------------------------------------------*///
    void NortekDvl::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
/// TOP LEVEL calls parsing functions. If successful, publishes and saves messages
    {
        DS_D(NortekDvl);

        // Create our result variables.
        auto ok = false;
        auto msg = ds_sensor_msgs::Dvl{};
        auto df21 = ds_sensor_msgs::NortekDF21{};
        auto rng = ds_sensor_msgs::Ranges3D{};

        std::tie(ok, df21) = NortekDvl::parse_bytes(bytes, d->beam_angle, d->phased_array, d->max_clock_offset);

        if (!ok)
        {
            ROS_ERROR_STREAM("CANNOT PARSE MSGS");
            return;
        }
 
        // fill in sensor metadata
        FILL_SENSOR_HDR(df21, df21.header.stamp, bytes.ds_header.io_time);

        // build derived message
        msg_to_dvl(&msg, &df21, d->beam_angle, d->phased_array);  // Create parsed dvl message
        msg_to_rng(&rng, &df21, d->beam_angle, d->phased_array);


        // publish messages
        d->dvl_pub_.publish(msg);
        d->df21_pub_.publish(df21);
        d->ranges_pub_.publish(rng);
    }
} // ds_sensors
