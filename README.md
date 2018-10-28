# Michigan Autonomous Vehicles
This software simulates autonomous vehicles within a ROS environment.

## Documentation

[**STABLE**](https://juliampc.github.io/AVExamples.jl/stable/) &mdash; **most recently tagged version of the documentation.**

[**LATEST**](https://juliampc.github.io/AVExamples.jl/latest/) &mdash; **in-development version of the documentation.**

## Installation and Getting Started

To run MAVs, first clone the repo as:
```
git clone https://github.com/JuliaMPC/MAVs
```
Or for the develop branch
```
git clone -b develop https://github.com/JuliaMPC/MAVs
```
Then follow the [Docker instructions](https://github.com/JuliaMPC/MAVs/tree/master/docker#mavs-docker).

* Note: currently, the entire repo is cloned twice (once by the above command and once by the Docker file).
* In the future this software may be configured to save space and only cloned once

After this, to test the software:
### Mitigate this issue
Something like this may appear:
```
ERROR: LoadError: Module RobotOS declares __precompile__(true) but require failed to create a usable precompiled cache file.
Stacktrace:Waiting for '/gazebo/unpause_physics' service...
 [1]
_requirelidar simulation in Gazebo has been initialized
(::Symbol) at ./loading.jl:502
 [2] require(::Symbol) at ./loading.jl:405
 [3] include_from_node1(::String) at ./loading.jl:576
 [4] include(::String) at ./sysimg.jl:14
 [5] process_options(::Base.JLOptions) at ./client.jl:305
 [6] _start() at ./client.jl:371
while loading /home/mavs/MAVs/ros/src/models/gazebo/vehicle_description/link_positions.jl, in expression starting on line 2
````

While [this issue](https://github.com/jdlangs/RobotOS.jl/issues/45) needs to be resolved, a workaround is available:
```
sudo rm -r /home/mavs/.julia/.cache
```
This issue happens frequently, so just run the above command is there is a cache error from julia. Then rerun.

### Run some of the system demos
The demos are included in the documentation, for instance try [demoA](https://juliampc.github.io/AVExamples.jl/latest/demos/system/demoA.html).


## Requirements
The following is currently a non-exhaustive list of requirements:
```
sudo apt-get install ros-kinetic-hector-nav-msgs ros-kinetic-hector-map-tools ros-kinetic-joint-state-publisher
```

## Tests
Unfortunately this software stack exceeds the time limit fo Docker as well as Travis services (~45 min). So, while these services are configured, they cannot be utilized.   
[Docker Hub repo](https://hub.docker.com/r/avpg/mavs/) ![mavs build status](https://img.shields.io/docker/build/avpg/mavs.svg)

[![Build Status](https://travis-ci.org/JuliaMPC/MAVs.svg?branch=master)](https://travis-ci.org/JuliaMPC/MAVs)

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://juliampc.github.io/AVExamples.jl/stable/)
[![Latest](https://img.shields.io/badge/docs-latest-blue.svg)](https://juliampc.github.io/AVExamples.jl/latest/)
