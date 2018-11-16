# contact_sensor

This package provides a Gazebo plugin: `libcontact_sensor_mavs.so` that can be
used in .sdf files to detect collision of the associated link.

For developers that work on top of Gazebo, and wish to update plugins, do:
`sudo apt-get install libgazebo8-dev`

## Input
Name | Description
--- | ---
`rosParamName` | Ros param that should be updated with truth values for collision detection

## Output
When a collision is detected, the ros parameter is set to `true`, it is `false` at the start.
Once a collsion is detected, the ros parameter will always be `true`.

### Note
- When `rosParamName` is not specified, the plugin defaults to `/vehicle_collided`

## Example use case in a .sdf file:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

     <include>
      <uri>model://sun</uri>
    </include>

    <model name="box">
      <link name="link">
        <pose>...</pose>

        <collision name="box_collision">
          ...
        </collision>

        <visual name="visual">
          ...
        </visual>

        <sensor name="my_contact" type="contact">
          <plugin name="contact_sensor" filename="libcontact_sensor_mavs.so">
            <rosParamName>/vehicle_collided</rosParamName>
          </plugin>
          <contact>
            <collision>box_collision</collision>
          </contact>
          <update_rate>5</update_rate>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
```
