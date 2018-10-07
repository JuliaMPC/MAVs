FROM c1yxuan/kinetic-desktop-opengl
MAINTAINER Yu-Hsuan Chang <yuxuanch@umich.edu>

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
RUN sudo wget https://julialang-s3.julialang.org/bin/linux/x64/0.6/julia-0.6.2-linux-x86_64.tar.gz \
 && sudo tar -xvf julia-0.6.2-linux-x86_64.tar.gz -C /opt \
 && rm -rf /var/lib/apt/lists/* \
 && rm julia-0.6.2-linux-x86_64.tar.gz*

# Add basic user
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

# create a folder and copy ros with mavs owner
RUN mkdir -p /home/$USERNAME/MAVs/ros
COPY --chown=mavs:mavs ros /home/$USERNAME/MAVs/ros
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; sudo rosdep init; rosdep update; catkin_init_workspace; cd ..; catkin_make' \
    && echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc \
    && echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

RUN mkdir -p /home/$USERNAME/shared_dir

# Default CMD
CMD ["/bin/bash"]
