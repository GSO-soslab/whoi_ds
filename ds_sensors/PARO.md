# Paro Depth Computations and Units
The driver for Paroscientific Digiquartz pressure-based depth sensors is capable of operating with 
many different input units and can correct for absolute pressure errors and the latitude-dependent
effects captured in the Foffonoff equation.

### Parameters

`depth_unit`: The depth unit emitted by the paroscientific depth sensor.  This is a setting made
 on the sensor.  Possible values:
 * `psi`: Pounds per square inch
 * `hpa`: Hectopascal, approximately 1 millibar
 * `kpa`: Kilopascal
 * `mpa`:
 * `inhg`: Inches of mercury
 * `mmhg`: Millimeters of mercury
 * `mh2o`: Meters of water.  Assumes only a basic density conversion
 
Standard DSL vehicles operate their paros to report absolute pressure in `psi`.
 
`frame_id`: The TF link for the paro's reference frame

`message_timeout`: The maximum time, in seconds, between paro messages before the node's status
 changes to "error".

`health_check_period`: How often this node checks for message timeout.

`descriptive_name`: A descriptive name for this node.  Used in error reporting, status monitoring, etc.

`serial_number`: This paro's serial number.  Used for UUID generation and metadata tracking.

`uuid`: The sensor's UUID.  Allows a user to optionally specify a UUID for this sensor.

`latitude_deg`: The initial latitude used for the fofonoff correction, in degrees.  Not necessary when using the 
latitude topic.
`latitude_topic`: The topic to listen on for a NavSatFix that provides real-time latitude for the 
Fofonoff correction.

TODO: Go over how UUID stuff works.
  
## How the fields in the pressure message are computed
 * `raw_pressure`: The raw pressure as parsed.  In whatever units this paro happens to be reporting (indicated via the
  `pressure_raw_unit` field)
 * `pressure`: The "best-guess" pressure at depth in decibars.  0dbar pressure occurs at the surface.  1 decibar is 
 approximately 1m in depth.
 * `depth`: The "best-guess" real-time depth estimate in meters.  Computed applying the Fofonoff function to the 
 `pressure` field. 
 * `tare`: The static pressure correction applied when converting `raw_pressure` to `pressure`.  Commonly measured at 
 the surface (see "zeroing the depth sensor" below) 
 * `latitude`: The latitude used by the Fofonoff correction for this sample.  
 Probably wildly inaccurate; not for navigation.  In degrees.
 
 Thus, the depth computation pseudocode is:
 
```$python
raw_pressure = parse_paro_string_as_float(serial_data)
raw_unit_to_dbar = get_unit_conversion(pressure_raw_unit)
pressure = raw_pressure * raw_unit_to_dbar - tare
depth = fofonoff(pressure, latitude)
```

Note that the tare is _never_ applied to the depth sensor via the Paro's TARE command.  This ensures the depth 
sensor always uses a consistent and known sensor offset.  It is recommended to report absolute pressure and use
this driver to correct for a static sensor offset.

## Zeroing the depth sensor

Variation in barometric pressure at the surface introduces a small static offset in the depth estimate.  This must be 
eliminated before a dive begins.

This can be eliminated by calling the `zero_depth` service.  That service will use the most recent raw pressure value
to set the tare and zero the depth to the most recent value.  If no data is available, or the only data is older than 
the `message_timeout` allows, the service call will fail and the tare value will not be updated.

Again, note that at no point is the tare value used to reprogram the paro's output itself.  All offset management is 
handled by the driver itself.

## Latitude sources for the Fofonoff Equation

Three sources may be used to set the latitude used by the Fofonoff correction.

1. A NavSatFix topic will update the latitude in real-time may be specified via the `latitude_topic` parameter.  
Latitude received via this topic will overwrite any of the other two options.  If the latitude data stops arriving, 
the most recent latitude will be used until a new one is received.
1. A static latitude may be set using the `latitude_deg` parameter.  Practical implementations may chose to remap this 
to their origin parameter.  This parameter is read at startup and does not update.
1. If no other latitude is provided, the DEFAULT_LATITUDE (35 degrees) is used.

For most applications, the latitude does not need to be especially accurate; half a degree is typically more than 
enough.  Accuracy requirements for almost all real-time applications simply are not that significant.  For deep-ocean
AUV/ROV/HOV dives, errors in the Fofonoff equation resulting from latitude inaccuracy are dwarfed by errors from 
Fofonoff's assumptions about the water column.  You need a full CTD cast, barometric pressure, and so on before 
worrying about giving the paro node 100m-accurate latitude.  That option is provided for convenience.