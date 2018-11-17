# system_shutdown

## System shutdown handler

This is a node in system package that handles system shutdown based on ros parameters

## Flags and Settings

Name | Description
--- | ---
`/system/shutdown/flags/running` | Shutdown node has been started. Example file: demoD.yaml
`/system/shutdown/flags/initialized` | Shutdown node has been initialized. Example file: demoD.yaml

## Inputs
Following are the argument required by the node

Name | Description
--- | ---
`/system/shutdown/params/shutdown_initiation_flags` | List of parameters that are constantly monitored for their truth values. If any of these becomes `true`, system shutdown would be initiated. Example file: demoD.yaml
`/system/shutdown/params/shutdown_completion_flags` | List of parameters which inidicate pre-processing required for a safe shutdown. Only when all the parameters become `true`, ros system would be shutdown. Example file: demoD.yaml

## Output
If any of the `shutdown_initiation_flags` becomes `true`, then a shutdown routine will be initiated. The node would then wait for all `shutdown_completion_flags` to become `true`. Once it is achieved, this node would shutdown. Since `system_shutdown` is always added as a required node in a launch file, the ros system would kill all remaining nodes which were spawned through this launch file.

## Example use case
Put following code in launch file:

```xml
<?xml version="1.0"?>
<launch>
  <arg name="system_params_path" default="$(find system)/config/system/demos/demoD.yaml"/>

  <!-- Add your nodes -->

  <node name="system_shutdown" pkg="system" type="system_shutdown" output="screen" required="true">
     <rosparam file="$(arg system_params_path)" command="load"/>
  </node>

  <node name="bootstrap" pkg="system" type="bootstrap.jl" output="screen"/>

</launch>
```

Put following code in your system.yaml. Note: Do not use this directly, this is just an example and shows only relevant components for shutdon node.

```
system:
 flags:
  override_shutdown_hook: false
  data_logging_completed: true

 shutdown:
  flags:
   running: true
   initialized: false
  params:
    shutdown_initiation_flags: ["system/flags/goal_attained", "/vehicle_collided"]
    shutdown_completion_flags: [system/flags/data_logging_completed]
```
