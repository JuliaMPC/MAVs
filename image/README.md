# MAVs Docker (virgil)

## Requirements

Tested on Ubuntu Xenial (16.04)

* An X server
* [Docker](https://www.docker.com/get-docker)
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

## How to build

```
$ cd MAVs/image/virgil/test
$ sh build.sh
```

## How to run
```
# Default shared directory path is /home/$USER/shared_dir
$ sh run.sh

# If you select your shared directory path
$ sh run.sh kinetic {SHARED_DIR_PATH}
```
