FROM c1yxuan/avpg_base

# create a folder and copy all the files in ros/ with mavs owner
RUN mkdir -p /home/$USERNAME/MAVs/ros
COPY --chown=mavs:mavs ros /home/$USERNAME/MAVs/ros
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; sudo rosdep init; rosdep update; catkin_init_workspace; cd ..; catkin_make' \
    && echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc \
    && echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

RUN mkdir -p /home/$USERNAME/MAVs/shared_dir

# Default CMD
CMD ["/bin/bash"]
