FROM osrf/ros:kinetic-desktop

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt-get update \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
    mesa-utils \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# Get Gazebo binaries
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
    # for obstacle_detector
    libarmadillo-dev \
    # for RobotOS.jl
    python-yaml \
 && rm -rf /var/lib/apt/lists/*

# to deal with rviz issue https://github.com/ros-visualization/rviz/issues/1291
RUN sudo add-apt-repository ppa:ubuntu-x-swat/updates \
 && sudo apt-get update -y \
 && sudo apt-get dist-upgrade -y  \
 && rm -rf /var/lib/apt/lists/*

# Install julia
RUN sudo wget https://julialang-s3.julialang.org/bin/linux/x64/0.6/julia-0.6.2-linux-x86_64.tar.gz \
    && sudo tar -xvf julia-0.6.2-linux-x86_64.tar.gz -C /opt \
            #TODO, delete julia .tar file
    && rm -rf /var/lib/apt/lists/*

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
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/$USERNAME/.bashrc && \
        #Fix for qt and X server errors
        echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
        # cd to home on login
        echo "cd" >> /home/$USERNAME/.bashrc

# Change user
USER mavs
# change number (below in RUN echo ###) to force rebuild starting from here
RUN echo 7 && \
    # install MichiganAutonomousVehicles.jl
    /opt/julia-d386e40c17/bin/julia -e 'Pkg.add("MichiganAutonomousVehicles");ENV["PYTHON"]="/usr/bin/python2.7";Pkg.build("PyCall");' && \
    # clone mavs
    git clone -b master https://github.com/JuliaMPC/MAVs /home/$USERNAME/MAVs && \
    # build mavs
    /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; catkin_init_workspace; cd ..; catkin_make' && \
    echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc && \
    echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

RUN echo 4 && \
    # get latest MichiganAutonomousVehicles.jl
    /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("MichiganAutonomousVehicles")'

RUN export LIBGL_ALWAYS_SOFTWARE=1

# Default CMD
CMD ["/bin/bash"]
