FROM avpg/mavs:base

# Change user
USER mavs
RUN git clone -b master https://github.com/JuliaMPC/MAVs /home/$USERNAME/MAVs && \
    /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; catkin_init_workspace; cd ..; catkin_make' && \
    echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc && \
    echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

# get latest MichiganAutonomousVehicles.jl
RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("MichiganAutonomousVehicles")'

# Default CMD
CMD ["/bin/bash"]
