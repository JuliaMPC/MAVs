FROM avpg/cain:base_cudagl

#install pyqtgraph for visualization
RUN sudo apt-get update \
    && sudo apt-get install -y python-pip \
    && sudo pip install pyqtgraph

RUN git clone -b develop https://github.com/JuliaMPC/MAVs /home/$USERNAME/MAVs \
    && /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; catkin_init_workspace;sudo rosdep init;rosdep update;cd ..; catkin_make' \
    && echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc \
    && echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

# update MichiganAutonomousVehicles.jl and remove .cache to avoid errors with PyCall.jl
RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("NLOptControl"); Pkg.checkout("MichiganAutonomousVehicles")' \
    && echo "rm -rf /home/mavs/.julia/.cache" \
    && /opt/julia-d386e40c17/bin/julia -e 'ENV["PYTHON"]="/usr/bin/python2.7"; Pkg.build("PyCall");' \
    && /opt/julia-d386e40c17/bin/julia -e 'Base.compilecache("PyCall")' \
    && /opt/julia-d386e40c17/bin/julia -e 'Base.compilecache("RobotOS")' \
    && /opt/julia-d386e40c17/bin/julia -e 'Base.compilecache("NLOptControl")'


# Default CMD
CMD ["/bin/bash"]
