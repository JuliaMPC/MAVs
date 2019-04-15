FROM avpg/cain:middle

RUN sudo apt-get update && sudo apt-get install -y \
 ffmpeg \
 ros-kinetic-rosparam \
 && sudo rm -rf /var/lib/apt/lists/*

#COPY knitro-10.3.0-z-Linux-64 /home/$USERNAME/knitro-10.3.0-z-Linux-64
#RUN echo 'export PATH="$HOME/knitro-10.3.0-z-Linux-64/knitroampl:$PATH"' >> ~/.bashrc
#RUN echo 'export LD_LIBRARY_PATH="$HOME/knitro-10.3.0-z-Linux-64/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
#RUN /bin/bash -c 'sudo chmod -R a+rX /home/mavs/knitro-10.3.0-z-Linux-64/*'
#COPY artelys_lic.txt /home/$USERNAME/

RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("MichiganAutonomousVehicles")'

RUN /bin/bash -c 'rm -rf /home/$USERNAME/MAVs/ros/*'

COPY ros /home/$USERNAME/MAVs/ros

# RUN /bin/bash -c 'chown -R $USERNAME:$USERNAME /opt/ros/kinetic; chown -R $USERNAME:$USERNAME /home/mavs/MAVs/results'

RUN sudo /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/mavs/MAVs/ros/; catkin_make'

#RUN echo 'alias sweepdir='cd "$HOME/MAVs/ros/src/system/param_sweep/scripts" '' >> ~/.bashrc

# Default CMD
CMD ["/bin/bash"]
