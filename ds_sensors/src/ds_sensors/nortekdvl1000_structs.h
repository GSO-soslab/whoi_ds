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
// file:  include/ds_sensors/nortekdvl_structs.h

#ifndef DS_SENSORS_NORTEKDVL_STRUCTS_H
#define DS_SENSORS_NORTEKDVL_STRUCTS_H

namespace ds_sensors {
    namespace nortekdvl_structs
    {
        struct header
        {
            uint8_t sync;
            uint8_t header_size;
            uint8_t headerid;
            uint8_t family;
            uint8_t data_size;
            uint8_t data_checksum;
            uint8_t header_checksum;
        } __attribute__((packed));

        struct bottomtrack
        {
            uint8_t version;
            uint8_t data_offset;
            uint32_t serial_num;
            uint8_t year;
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t minute;
            uint8_t seconds;
            uint16_t microseconds;
            uint16_t nbeams;
            uint32_t error;
            uint32_t status;
            float speed_sound;
            float temperature;
            float pressure;

            float velBeam[4];    //Velocities for each beam
            float distBeam[4];   //Distances for each beam
            float fomBeam[4];    //Figure of merit for each beam
            float timeDiff1Beam[4];  //DT1 for each beam
            float timeDiff2Beam[4];  //DT2 for each beam
            float timeVelEstBeam[4]; //Duration of velocity estimate for each beam

            float velX;  //Velocity X
            float velY;  //Velocity Y
            float velZ1; //Velocity Z1
            float velZ2; //Velocity Z2
            float fomX;  //Figure of Merit X
            float fomY;  //Figure of Merit Y
            float fomZ1; //Figure of Merit Z1
            float fomZ2; //Figure of Merit Z2
            float timeDiff1X;    //Time from trigger to center of bottom echo
            float timeDiff1Y;    //Same as above
            float timeDiff1Z1;
            float timeDiff1Z2;   //Same as above
            float timeDiff2X;    //Time from start of NMEA output msg to center of bottom echo
            float timeDiff2Y;    //Same as above
            float timeDiff2Z1;   //Same as above
            float timeDiff2Z2;
            float timeVelEstX;   //Duration of velocity estimate for each component
            float timeVelEstY;   //Same as above
            float timeVelEstZ1;
            float timeVelEstZ2;
        } __attribute__((packed));


    }  // nortekdvl_structs

} //ds_sensors
#endif //DS_SENSORS_NORTEKDVL_STRUCTS_H
