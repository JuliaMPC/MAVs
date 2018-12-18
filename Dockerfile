FROM avpg/cain:middle

RUN /bin/bash -c 'rm -rf /home/$USERNAME/MAVs/ros/devel/* /home/$USERNAME/MAVs/ros/build/*'

COPY ros /home/$USERNAME/MAVs/ros
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/; catkin_make'

# Default CMD
CMD ["/bin/bash"]
