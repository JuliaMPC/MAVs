FROM avpg/cain:base_cudagl

# create a folder and copy all the files in ros/ with MAVs owner
RUN mkdir -p /home/$USERNAME/MAVs/ros
COPY --chown=mavs:mavs ros /home/$USERNAME/MAVs/ros
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; sudo rosdep init; rosdep update; catkin_init_workspace; cd ..; catkin_make' \
    && echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc \
    && echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

RUN mkdir -p /home/$USERNAME/MAVs/shared_dir

# update MichiganAutonomousVehicles.jl and remove .cache to avoid errors with PyCall.jl
RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("MichiganAutonomousVehicles")' \
    && echo "rm -rf /home/mavs/.julia/.cache"

# Default CMD
CMD ["/bin/bash"]
