# MAVs Dockerfile workflow:
The workflow is consisted of two part, build a image and run a container:  
## Build a Image:
1. Pull the image from avpg/cain:middle  
2. Build Dockerfile: this Dockerfile mainly copy the source files in ros/ into the container and run “catkin_make” to setup ros workspace.  

### Dockerfile of avpg/cain:middle:
1. Pull the image from avpg/cain:base_cudagl:
2. Update Julia packages
3. Clone the source code from JuliaMPC/MAVs and build workspace

### Inside the Dockerfile of avpg/cain:base_cudagl:  
1. Build the base image from nvidia/cudagl:9.0-devel-ubuntu16.04
2. Install required packages  
3. Install ROS kinetic  
4. Install Julia
5. Install Chrono  

## Run a Container:
1. Setup PATH variable for later use of volumes when execute docker run.
2. docker run command to execute mavs image.


 

![volume_diagram](docker_volume.jpg)
