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
// file:  include/ds_sensors/rdidvl_header_match.h

#ifndef DS_SENSORS_RDIDVL_MATCH_H
#define DS_SENSORS_RDIDVL_MATCH_H

#include "boost/circular_buffer.hpp"

class match_header_length_2
{
public:
  explicit match_header_length_2() : length_(833), len_(0), sync_(false), found_(2, false), cb_(2)
  {
    ROS_INFO_STREAM("Matcher set " << cb_.capacity());
  }

  typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

  template <typename Iterator>
  std::pair<Iterator, bool> operator()(Iterator begin, Iterator end)
  {
    Iterator i = begin;
    std::string hexAscii = "7F7F";
    std::vector<unsigned char> myHeader;
    for (int j = 0; j < hexAscii.length(); j += 2)
    {
      std::string byteString = hexAscii.substr(j, 2);
      unsigned int myByte;
      sscanf(byteString.c_str(), "%X", &myByte);
      myHeader.push_back((unsigned char)myByte);
    }
    header_ = myHeader;
    while (i != end)
    {
      cb_.push_back(*i++);
      // If the stream is not synchronized, access cb_ only if it's full i.e. size is equal to capacity
      if ((!sync_) && (cb_.full()))
      {
        // Update the found_ vector that stores matching header bytes status
        for (int j = 0; j < header_.size(); ++j)
        {
          found_[j] = (cb_[j] == header_[j] ? true : false);
        }
        // If all the found_ vector is true, then we are synchronized to the binary frame
        if (std::all_of(found_.begin(), found_.end(), [](bool v) { return v; }))
        {
          sync_ = true;
          // Increment the binary frame len_ that we already read by the size of the header
          len_ += header_.size();
        }
      }
      else if (sync_)
      {
        len_++;
        // We reached the expected length of the binary frame, tell async_read_until that we're done reading this frame
        if (len_ == 4)
        {
          length_ = cb_[0] + 2;
        }
        if (len_ == length_)
        {
          ROS_INFO_STREAM("Buffering ended, length: " << len_);
          return std::make_pair(i, true);
        }
      }
    }
    return std::make_pair(i, false);
  }

private:
  std::vector<unsigned char> header_;
  boost::circular_buffer<unsigned char> cb_;
  std::vector<bool> found_;
  int length_;
  int len_;
  bool sync_;
};

namespace boost
{
namespace asio
{
template <>
struct is_match_condition<match_header_length_2> : public boost::true_type
{
};
}  // namespace asio
}  // namespace boost

#endif  // DS_SENSORS_RDIDVL_MATCH_H