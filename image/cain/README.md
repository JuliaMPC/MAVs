# MAVs Docker (cain)

## Requirements
Tested on Ubuntu Xenial (16.04)
* An X server
* [Docker](https://www.docker.com/get-docker)
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)


## How to update base image on docker hub
1) Run
```
$sh build.sh
```
2) login to docker
```
$docker login
```
3) ensure the name of the image
```
$docker image ls
```
Currently this should be avpg_base, where the image name is specified in the build.sh script

4) tag the image
```
$docker tag avpg_base avpg/cain:base_cudagl
```
assuming that the image name is avpg_base

5) publish the image
```
$docker push avpg/cain:base_cudagl
```

see the [docker tutorial on containers](https://docs.docker.com/get-started/part2/#build-the-app) for more detailed instructions

## Pull and run the image from the remote repository
It would be interesting to try this with the higher-level image, but this is *not yet tested*.
```
docker run -p 4000:80 avpg/cain:base_cudagl
```
