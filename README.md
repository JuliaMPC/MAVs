# Michigan Autonomous Vehicles
This software simulates autonomous vehicles within a ROS environment.

## Documentation
[**STABLE**](https://juliampc.github.io/MAVs/stable/) &mdash; **most recently tagged version of the documentation.**

[**LATEST**](https://juliampc.github.io/MAVs/latest/) &mdash; **in-development version of the documentation.**

## Installation and Getting Started
To run MAVs, first clone the repo as:
```
git clone https://github.com/JuliaMPC/MAVs
```
Or for the develop branch
```
git clone -b develop https://github.com/JuliaMPC/MAVs
```
Then follow the [Docker instructions](hhttps://github.com/JuliaMPC/MAVs/tree/develop/image/cain).

* Note: currently, the entire repo is cloned twice (once by the above command and once by the Docker file).
* In the future this software may be configured to save space and only cloned once


## Tests
Unfortunately this software stack exceeds the time limit on Docker as well as Travis services (~45 min). So, while these services are configured, they cannot be utilized.   
[Docker Hub repo](https://hub.docker.com/r/avpg/mavs/) ![mavs build status](https://img.shields.io/docker/build/avpg/mavs.svg)

[![Build Status](https://travis-ci.org/JuliaMPC/MAVs.svg?branch=master)](https://travis-ci.org/JuliaMPC/MAVs)

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://juliampc.github.io/AVExamples.jl/stable/)
[![Latest](https://img.shields.io/badge/docs-latest-blue.svg)](https://juliampc.github.io/AVExamples.jl/latest/)
