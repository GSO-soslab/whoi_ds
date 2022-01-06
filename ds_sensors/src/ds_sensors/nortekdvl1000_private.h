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
// file:  src/ds_sensors/nortekdvl1000_private.h
#ifndef DS_SENSORS_NORTEKDVL1000_PRIVATE_H
#define DS_SENSORS_NORTEKDVL1000_PRIVATE_H

#include "ds_sensors/nortekdvl1000.h"

namespace ds_sensors {
// Our private impl struct subclasses SensorBase::Impl
    struct NortekDvlPrivate
    {
        ros::Publisher dvl_pub_;
        ros::Publisher df21_pub_;
        ros::Publisher ranges_pub_;

        // beam angle in radians.  Probably 30 deg.
        double beam_angle;

        //whether or not this is a phased array
        bool phased_array;

        // max clock offset before we use I/O time instead of the dvl time
        double max_clock_offset;

    };
}  // ds_sensors
#endif //DS_SENSORS_NORTEKDVL1000_PRIVATE_H
