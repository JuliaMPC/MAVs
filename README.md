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
Then follow the [Docker instructions](https://github.com/JuliaMPC/MAVs/tree/master/docker#mavs-docker).

After this, to test the software, try running some of the system demos. The demos are included in the documetnation, for instance try [demoA](https://juliampc.github.io/AVExamples.jl/latest/demos/system/demoA.html).

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
