FROM avpg/cain:middle

COPY ros /home/$USERNAME/MAVs/ros
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/; catkin_make'

# Default CMD
CMD ["/bin/bash"]
