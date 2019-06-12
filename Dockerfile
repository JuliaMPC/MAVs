FROM avpg/cain:middle_b

ENV IGN_IP=127.0.0.1
RUN export IGN_IP

# COPY knitro-10.3.0-z-Linux-64 /home/$USERNAME/knitro-10.3.0-z-Linux-64
# RUN echo 'export PATH="$HOME/knitro-10.3.0-z-Linux-64/knitroampl:$PATH"' >> ~/.bashrc
# RUN echo 'export LD_LIBRARY_PATH="$HOME/knitro-10.3.0-z-Linux-64/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
# RUN /bin/bash -c 'sudo chmod -R a+rX /home/mavs/knitro-10.3.0-z-Linux-64/*'
# COPY artelys_lic.txt /home/$USERNAME/

RUN /bin/bash -c 'rm -rf /home/$USERNAME/MAVs/ros/*'
COPY ros /home/$USERNAME/MAVs/ros

# build the ros packages
RUN sudo /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/mavs/MAVs/ros/; catkin_make'

# modify permissions for results folder
#RUN /bin/bash -c 'sudo chmod -R 777 $HOME/MAVs/results/*'
RUN /bin/bash -c 'sudo chmod -R ugo+rw ~/MAVs/results'

# Default CMD
CMD ["/bin/bash"]
