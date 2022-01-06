# ds_sensors:  a ROS package for sensors


## Provided Sensors

- ds_sensors::AnderaaOxyOptode: Anderaa Oxygen Optode 4330/4835/4831
- ds_sensors::Aps1540:  Applied Physics 1540 vector magnetometer
- ds_sensors::ParoDigiquartz:  Paro DigiQuartz pressure sensor
- ds_sensors::ResonSvp70:  Reson SVP 70 sound velocity probe
- ds_sensors::Sbe49:  SeaBird SBE49 CTD sensor

## libds_sensors: The ds_sensors library

The primary goal of this package is to provide a library of data publishers following a common design pattern with
a stable ABI.  To achieve thih, the ds_sensor library makes use of the PIMPL design pattern.

## Topic Conventions

The topic convention for `ds_sensors` nodes is to place them beneath the ros node name.  So if you start a `ds_sensors`
node with the name `maggie` then topics will be named "under" the node (e.g. `maggie/$TOPIC`).  
It is still possible to create absolute topic names, see `SensorBase::Impl::addPublisher`.

## sensor_node: A ROS node for all libds_sensors-based sensors

This package provides runtime access to the sensors through `sensor_node`.  `sensor_node` has a few runtime arguments:

```
rosrun ds_sensors sensor --help
Usage:  /home/zac/ros/sensors_ws/devel/lib/ds_sensors/sensor sensor --node [ROS_OPTIONS]

Starts a node named 'sensor' for the desired sensor

Info options:
  -h [ --help ]         print this help message
  --list-sensors        Show list of available sensors

ROS_OPTIONS:  These can be used when using roslaunch
  __name:=$NAME         Override the default node name

EXAMPLE:
  start a node named 'primary_depth' under the namespace '/devices' using the 'paro' sensor:
    env ROS_NAMESPACE=/devices rosrun ds_sensors sensor paro __name:=primary_depth
```
- `sensor`:  name of the sensor.  For a list of available sensors, use the `--list-sensors` argument.

## Launch files

This package provides two launch files:

- `sensor.launch`:  A launch file for starting up a sensor node
- `sensor_asio.test`:  A launch file for starting a IO test

These launch files are prameterized and can be included in other files to launch multiple sensors.  Examples for each
case are provided:

- `example.launch`:  An example launch file that includes `sensor.launch`
- `example_asio.test`: An example launch file that includes `sensor_asio.test`

## Extending ds_sensors with new sensors

A detailed example of adding a new sensor to the `ds_sensors` package is provided in [EXTENDING.md](EXTENDING.md)
