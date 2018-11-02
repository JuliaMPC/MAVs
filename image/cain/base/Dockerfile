FROM nvidia/cudagl:9.0-devel-ubuntu16.04
MAINTAINER Huckleberry Febbo <febbo@umich.edu>

# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

RUN apt-get update \
 && apt-get install -y \
    wget \
    unzip \
    lsb-release \
    sudo \
    git-core \
    mesa-utils \
    software-properties-common \
    # for Ipopt
    build-essential \
    gfortran \
    pkg-config \
    liblapack-dev \
    libblas-dev \
    # for obstacle_detector
    libarmadillo-dev \
    # for RobotOS.jl
    python-yaml \
    # to deal with rviz issue https://github.com/ros-visualization/rviz/issues/1291
 && add-apt-repository ppa:ubuntu-x-swat/updates \
 && apt-get update -y \
 && apt-get dist-upgrade -y  \
 && rm -rf /var/lib/apt/lists/*

# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo8 \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-navigation \
    ros-kinetic-hector-slam \
    ros-kinetic-joy \
    ros-kinetic-perception-pcl \
    ros-kinetic-velodyne-description \
 && apt-get clean

# Install julia
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/0.6/julia-0.6.2-linux-x86_64.tar.gz \
 && tar -xvf julia-0.6.2-linux-x86_64.tar.gz -C /opt \
 && rm -rf /var/lib/apt/lists/* \
 && rm julia-0.6.2-linux-x86_64.tar.gz*

# Add basic user to connect the container to host's X server for display.
ENV USERNAME mavs
ENV PULSE_SERVER /run/pulse/native
RUN useradd -m $USERNAME && \
       echo "$USERNAME:$USERNAME" | chpasswd && \
      usermod --shell /bin/bash $USERNAME && \
      usermod -aG sudo $USERNAME && \
      echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
      chmod 0440 /etc/sudoers.d/$USERNAME && \
      # Replace 1000 with your user/group id
      usermod  --uid 1000 $USERNAME && \
      groupmod --gid 1000 $USERNAME

# Setup .bashrc for ROS
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/$USERNAME/.bashrc \
#Fix for qt and X server errors
&& echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc \
# cd to home on login
&& echo "cd" >> /home/$USERNAME/.bashrc

# Change user
USER mavs

# Install julia and Chrono
RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.add("MichiganAutonomousVehicles");ENV["PYTHON"]="/usr/bin/python2.7";Pkg.build("PyCall");Pkg.build("Ipopt");' \
 && sudo wget http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.4.zip \
 && sudo unzip irrlicht-1.8.4.zip -d /home/mavs \
 && cd /home/mavs/irrlicht-1.8.4/source/Irrlicht \
 && sudo make sharedlib \
 && sudo make install \
 # copy Chrono makefiles to the docker and build it
 && cd /opt ; sudo mkdir -p chrono/chrono_source chrono/chrono_build ; cd chrono/chrono_source \
 && sudo git clone -b develop https://github.com/projectchrono/chrono.git

# Copy chrono_build and make file
COPY chrono_build /opt/chrono/chrono_build
RUN cd /opt/chrono/chrono_build ; sudo make

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
