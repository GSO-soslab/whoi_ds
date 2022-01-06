# Extending the `ds_sensors` package

This document explains how to add a new sensor to the `ds_sensors` package by using the existing
`ds_sensors::Aps1540` sensor class as a model.  

## Quick Rules for New Sensors

1. The new sensor must subclass `ds_sensors::SensorBase`
2. The new sensor should hide all implementation details within a subclass of
  `ds_sensors::SensorBase::Impl`
3. String parsing functions should be added to the new sensor class as static methods
4. Unit tests for any new parsing functions are required.
5. The new sensor should be added to the `sensor_node` executable, and to `include/ds_sensors/ds_sensors.h`.


## Rule 1:  Subclassing ds_sensors::SensorBase

Create two new files:

- `include/ds_sensors/aps1540.h`
- `src/ds_sensors/aps1540.cpp`

Here's the header file.

```C++
// file:  include/ds_sensors/aps1540.h
#ifndef DS_SENSORS_APS1540_H
#define DS_SENSORS_APS1540_H

#include "ds_sensors/sensor_base.h"

// Our sensors live in the ds_sensors namespace
namespace ds_sensors {
class Aps1540 : public SensorBase {
 protected:
  // This will hold our implementation details.
  struct Impl;

 public:
  // Constructor overrides.  Match the same signatures as SensorBase
  explicit Aps1540();
  Aps1540(int argc, char* argv[], const std::string& name);

  // This gets called every time the underlying I/O layer reports bytes are ready
  // to be parsed.  We'll add hooks to our message parsing functions insdie.
  void parseReceivedBytes(const ds_core_msgs::RawData &bytes) override;

  ///@brief Parse a message from the Aps1540
  ///
  ///
  /// Returns a std::pair<bool, ds_sensor_msgs::VectorMagneticField>.  The boolean indicates whether parsing was
  /// successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \return
  static std::pair<bool, ds_sensor_msgs::VectorMagneticField> parse_bytes(const ds_core_msgs::RawData &bytes);

 protected:
  // Protected constructors so we can create subclasses of THIS class if desired.
  explicit Aps1540(std::unique_ptr<Impl> impl);
  Aps1540(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name);

 private:
  // Functions for accessing our implementation structure.  It is very important
  // to use these instead of direct access.
  auto d_func() noexcept -> Impl *;
  auto d_func() const noexcept -> Impl const *;
};

}
```

This covers the basic `SensorBase` methods that we need to override.  You can add additional
methods specific to this sensor as needed, but we won't for this example.

**NOTE:** To follow the PIMPL pattern you should refrain from adding non-public functions and
data members to your subclass here.  Anything you're tempted to add as a protected or private
member should be left to your `Impl` subclass.  We'll get to that in the next section.

Here's the source file:

```C++

#include "ds_sensors/aps1540.h"

namespace ds_sensors
{

//
// Our constructors use the protected constructor from `SensorBase`, providing our
// own version of the private implementation class.
//
// This newly constructed Aps1540::Impl gets implicitly upcast to SensorBase::Impl 
// when passed to SensorBase's constructor.
//
Aps1540::Aps1540() : Aps1540(std::unique_ptr<Impl>(new Impl()))
{
}

Aps1540::Aps1540(int argc, char* argv[], const std::string& name)
    :Aps1540(std::unique_ptr<Impl>(new Impl()), argc, argv, name)
{
}

Aps1540::Aps1540(std::unique_ptr<Impl> impl) : SensorBase(std::move(impl))
{
}

Aps1540::Aps1540(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name)
    : SensorBase(std::move(impl), argc, argv, name)
{
}

//
// This is how we get access to our new private Aps1540::Impl.
// See, in the constructors above, we upcast Aps1540::Impl into SensorBase::Impl, where
// it's stored in the SensorBase::impl_ member.
//
// To get the Impl class back *in the propper type* we need to downcast it again before
// working on it, which is why we have the static_cast<>'s here. To perhaps make it look
// more explicit, think of `static_cast<Impl *>` as 
// static_cast<Aps1540::Impl *>(SensorBase::Impl*)
//
inline auto Aps1540::d_func() noexcept -> Impl * {
  return static_cast<Impl *>(ds_base::DsProcess::d_func());
}

inline auto Aps1540::d_func() const noexcept -> Impl const * {
  return static_cast<Impl const *>(ds_base::DsProcess::d_func());
}

void Aps1540::parseReceivedBytes(const ds_core_msgs::RawData &bytes) {
}
```

Not too interesting yet.  We'll get there.  At the moment this won't even compile because 
`Aps1540::Impl` has not been defined.  Let's do that now..

## Rule 2:  Subclassing SensorBase::Impl

You'll notice one header file in `include/ds_sensors` looks different from the others.
`include/ds_sensors/sensor_base_private.h`  provides the basic building block for the private
implementation details for all sensors.  Take a look at it, we'll be using it for ourselves next.

Create a new file:  `src/ds_sensors/aps1540_private.h`.  **NOTE:**  this file is in the `src`
directory, not `include`, even though it's a header.  That's ON PURPOSE.  The whole point of
hiding implementation details is to *hide* them, we're not making this structure visible to the
outside world.

So what goes in here?  Anything that's not part of the public interface of the class.  Want to
keep some private variables associated with the class?  Put them here.  Have some complex helper
function that isn't called by the user?  Add it here.  You're free to do pretty much anything you
want to this structure and you won't break ABI compatibility.

**NOTE:** If you need access to the public class (`Aps1540` in this case) from a method in your
private structure (`Aps1540::Impl` here), pass it along as a parameter:
    
    Aps1540::Impl::someMethodThatRequiresPublic(Aps1540* public, ....) {
      //
      // do some expensive calculations or such..
      //
      public->callPublicMethod(....);
    }
    
then your call from the public side of things would be:

    Aps1540::somePublicMethod(....) {
        auto d = d_func();
        d->someMethodThatRequiresPublic(this, ....);
    }
    
Yes, it's a bit of extra complication. But remember, by doing things this way you are free to muck
around with the internals of how the class works without worrying about breaking the ABI.

With our Aps1540 we'd like to keep the last parsed data message around.  That's it.  A bit boring,
but the machinery allows for easy growth down the line.

Here's our file:

```C++
// file:  src/ds_sensors/aps1540_private.h

#ifndef DS_SENSORS_APS1540_PRIVATE_H
#define DS_SENSORS_APS1540_PRIVATE_H

#include "ds_sensors/sensor_base_private.h"

// We've already created a message type for our sensor in the ds_sensor_msgs package
#include "ds_sensor_msgs/VectorMagneticField.h"

namespace ds_sensors {

// Our private impl struct subclasses SensorBase::Impl
struct Aps1540::Impl : public SensorBase::Impl {

  Impl(): SensorBase::Impl() {}
  ds_sensor_msgs::VectorMagneticField vector_magnetic_field_;  //!< Last valid maggie message
  
};

}

```

### Adding Publishers to ds_sensors::Aps1540

Now that we've fully defined our impl structure we can use it (and it's base class) to easily
add new publishers using `SensorBase::Impl::addPublisher`:

```C++
// file: src/ds_sensors/aps1540.cpp

#include "ds_sensors/aps1540.h"
#include "aps1540_private.h"  //<  new! Include our new private structure

#include <sstream>

namespace ds_sensors
{

// Punt actual setup into our protected function so things stay in sync.
Aps1540::Aps1540() : Aps1540(std::unique_ptr<Impl>(new Impl()))
{
}

Aps1540::Aps1540(int argc, char* argv[], const std::string& name)
    :Aps1540(std::unique_ptr<Impl>(new Impl()), argc, argv, name)
{
}

Aps1540::Aps1540(std::unique_ptr<Impl> impl) : SensorBase(std::move(impl))
{
  // This is how you access Aps1540::Impl, through the d_func() function!  
  auto d = d_func();
  
  // Use the templated addPublisher method on SensorBase::Impl to add a new publisher
  // using our message type on the desired topic name.  READ THE DOCUMENTATION on
  // SensorBase::Impl::addPublisher for some nuances with how topic names are treated!
  auto d = d_func();
  d->addPublisher<ds_sensor_msgs::VectorMagneticField>(this, "vector_magnetic_field", 10);
}

Aps1540::Aps1540(std::unique_ptr<Impl> impl, int argc, char* argv[], const std::string& name)
    : SensorBase(std::move(impl), argc, argv, name)
{
  auto d = d_func();
  d->addPublisher<ds_sensor_msgs::VectorMagneticField>(this, "vector_magnetic_field", 10);
}

```

## Rule 3:  Add a Parsing Function

Data ready to parse is provided packed within the `ds_sensor_msgs/ByteSequence.msg`, which
provides a raw sequence of bytes as well as a header with a timestamp.  `ds_sensors::SensorBase`
takes care of sending this raw byte string out for logging, so you don't need to concern yourself
with that.

What we need is a function to convert `ds_sensor_msgs/ByteSequence.msg` into 
`ds_sensor_msgs/VectorMagneticField.msg`.  Here's one such function:

```C++
  // file:  include/ds_sensors/aps1540.h
  
  ///@brief Parse a message from the Aps1540
  ///
  ///
  /// Returns a std::pair<bool, ds_sensor_msgs::VectorMagneticField>.  The boolean indicates whether parsing was
  /// successful.
  ///
  /// \param bytes    Sequence of bytes received
  /// \return
  static std::pair<bool, ds_sensor_msgs::VectorMagneticField> parse_bytes(const ds_core_msgs::RawData &bytes);
```

```C++
// file:  src/ds_sensors/aps1540.cpp
namespace ds_sensors {
std::pair<bool, ds_sensor_msgs::VectorMagneticField> Aps1540::parse_bytes(const ds_core_msgs::RawData &bytes) {

  auto msg = ds_sensor_msgs::VectorMagneticField{};

  // From 'parse_aps_maggie' in rov::maggie_functions.cpp.  Does not guard against some types of malformed strings
  // (like having 'extra' numbers because we missed a line terminator)
  //
  // Message units for fields are nT, temperature is C
  // APS provides measurements in nT and C

  const auto n_parsed = sscanf(
      reinterpret_cast<const char *>(bytes.data.data()),
      "%f %f %f %f",
      &msg.x, &msg.y, &msg.z, &msg.temperature);

  if (n_parsed < 4) {
    ROS_DEBUG("Expected at least 4 values, found %d", n_parsed);
    return {false, msg};
  }

  msg.ds_header = bytes.ds_header;
  
  // Our instrument doesn't have a separate "measured" timestamp in the data,
  // so we'll just use the io_time
  msg.header.stamp = msg.ds_header.io_time;
  
  return {true, msg};
}
}
```

Key things:

- We return a pair with an element indicating success.
- The method is static:  we can use it to parse data without instantiating a class.  This
  presents a compromise between making the parser available for reuse and poluting a namespace
  with a bunch of `parse_XXX_message`-themed free functions.
  
### Adding hooks to parse new messages

We need to get our method called when new data is available.  We do this by finally adding some
meat to `Aps1540::parseReceivedBytes`

```C++
// file: src/ds_sensors/aps1540.cpp

void Aps1540::parseReceivedBytes(const ds_core_msgs::RawData &bytes) {

  // Create our result variables.
  auto ok = false;
  auto msg = ds_sensor_msgs::VectorMagneticField{};

  // 'modern' c++!  (Well, modern would be C17 and you could std::tie(auto ok, auto msg)... )
  // This 'splits' the pair returned by Aps1540::parse_bytes into the variables defined
  // above
  std::tie(ok, msg) = parse_bytes(bytes);

  // Parsing failed?  Nothing more to do.  The base class has already sent out the raw bytes.
  if (!ok) {
    return;
  }

  // Publish the new message.
  // Again, check out SensorBase::Impl::publishMessage.  This method does three things:
  // - publishes the message on the provided topic name (using the publisher you made earlier)
  // - saves the message in the last_message_  unordered_map as a boost::any
  // - saves the timestamp in the last_message_timestamp unordered_map
  //
  // item 1 is important for obvious reasons.  item 2 is potentiatlly interesting.  item 3 is
  // used for the sensor health check.
  
  auto d = d_func();
  d->publishMessage("vector_magnetic_field", msg);
}
```

That's it!  At this point we have a class that will respond to data, attempt to parse it, and
send out new data with each successful parse!

## Step 4:  Testing

No, we're not done, it's time for testing!

For this I'll refer you to [test_aps1540_parsing.cpp](src/test/test_aps1540_parsing.cpp) as it is
pretty descriptive on it's own.


## Step 5:  Add the new sensor to sensor_node and include/ds_sensors/ds_sensors.h

First, add the new sensor's public header to the all-inclusive `include/ds_sensors/ds_sensors.h`

```C++
// file: include/ds_sensors/ds_sensors.h


#ifndef DS_SENSORS_DS_SENSORS_H
#define DS_SENSORS_DS_SENSORS_H

#include "ds_sensors/aps1540.h"
#include "ds_sensors/sbe49.h"
```

Two changes to [`sensor_node.cpp`](src/ds_sensors/sensor_node.cpp) are required to enable the new
sensor for use with the ROS node:

```C++
// file: src/ds_sensors/sensor_node.cpp

static const auto SENSORS = std::list<std::pair<std::string, std::string>> {
  {"sbe49", "SeaBird SBE49 CTD"},
  {"aps1540", "Applied Physics 1540 vector magnetometer"} //< NEW!  Added this line here
};
```

The above adds a new sensor by the name "aps1540" to the list of available sensors.

```C++
  // The node handle is created somewhere above with these lines
  auto service = std::unique_ptr<boost::asio::io_service>(new boost::asio::io_service());
  auto handle = std::unique_ptr<ros::DsNodeHandle>(new ros::DsNodeHandle(service.get()));
  // .
  // .
  // and sometime later we enter our sensor name checking block...
  // .
  if (sensor == "sbe49") {
    node.reset(new ds_sensors::Sbe49(std::move(handle)));
  }
  // .
  // . (Hopefully) lots of other sensors
  // . 
  // This is our new entry!
  else if (sensor == "aps1540") {
    node.reset(new ds_sensors::Aps1540());
  }
  // And this is the default catch error.
  else{
    std::cerr << "ERROR: Unknown sensor specified: " << sensor << std::endl;
    std::cerr << "ERROR: Use '--list-sensors' and choose a valid sensor." << std::endl;
    return 1;
  }
```

And the above creates a new instance of our `Aps1540` class to be run as the sensor in the node.
That's all you have to do!
