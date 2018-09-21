FROM  nvidia/opengl:1.0-glvnd-devel-ubuntu16.04
MAINTAINER Huckleberry Febbo <febbo@umich.edu>

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
    build-essential gfortran pkg-config liblapack-dev libblas-dev \
 && rm -rf /var/lib/apt/lists/*

    # Setup your computer to accept software from packages.ros.org.
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    # Set up your keys
 && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
    # First, make sure your Debian package index is up-to-date
 && apt-get update \
 && apt-get install -y \
    ros-kinetic-ros-base \
    # Get Gazebo binaries
 && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
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
    ros-kinetic-rviz \
    # for obstacle_detector
    libarmadillo-dev \
    # for RobotOS.jl
    python-yaml \
    # to deal with rviz issue https://github.com/ros-visualization/rviz/issues/1291
 && add-apt-repository ppa:ubuntu-x-swat/updates \
 && apt-get update -y \
 && apt-get dist-upgrade -y  \
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

USER mavs
RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.add("MichiganAutonomousVehicles");ENV["PYTHON"]="/usr/bin/python2.7";Pkg.build("PyCall");Pkg.build("Ipopt");'

RUN git clone -b master https://github.com/JuliaMPC/MAVs /home/$USERNAME/MAVs && \
   /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; catkin_init_workspace; cd ..; catkin_make' && \
   echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc && \
   echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc && \
   echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

# Install Chrono
RUN sudo wget http://downloads.sourceforge.net/irrlicht/irrlicht-1.8.4.zip && \
sudo unzip irrlicht-1.8.4.zip -d /home/mavs && \
cd /home/mavs/irrlicht-1.8.4/source/Irrlicht && \
sudo make sharedlib && sudo make install

# copy chrono to the docker and build it
RUN cd /opt ; sudo mkdir -p chrono/chrono_source chrono/chrono_build ; cd chrono/chrono_source && \
sudo git clone -b master https://github.com/projectchrono/chrono.git
COPY chrono_build /opt/chrono/chrono_build
RUN cd /opt/chrono/chrono_build ; sudo make
#TODO: change to develop branch on Chrono

# get latest MichiganAutonomousVehicles.jl
#RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("MichiganAutonomousVehicles")'

# Default CMD
CMD ["/bin/bash"]
