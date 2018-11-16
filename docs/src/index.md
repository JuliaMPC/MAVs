# MAVs Documentation
This software simulates an autonomous vehicle within a ROS environment.

## Features
* nonlinear model predictive control through [NLOptControl.jl](https://github.com/JuliaMPC/NLOptControl.jl)
* lidar simulation through Gazebo
* vehicle model through Chrono

## Requirements
Tested on Ubuntu Xenial (16.04)
* An X server
* [Docker](https://www.docker.com/get-docker)
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

## Installation
1. Clone the `develop` branch of the repository
```
git clone -b develop https://github.com/JuliaMPC/MAVs
```

2. Build image
```
sh build.sh
```

## Citation
If you find [NLOptControl.jl](https://github.com/JuliaMPC/NLOptControl.jl) useful, please cite it:
```
@software{nlopt,
  author = {{Huckleberry Febbo}},
  title = {NLOptControl.jl},
  url = {https://github.com/JuliaMPC/NLOptControl.jl},
  version = {0.0.1},
  date = {2017-06-17},
}
```

If you find [VehicleModels.jl](https://github.com/JuliaMPC/VehicleModels.jl) useful, please cite this paper:
```
@Conference{Febbo2017,
  author    = {Huckleberry Febbo, Jiechao Liu, Paramsothy Jayakumar, Jeffrey L. Stein, Tulga Ersal},
  title     = {Moving Obstacle Avoidance for Large, High-Speed Autonomous Ground Vehicles},
  year      = {2017},
  publisher = {IEEE}
}
```
