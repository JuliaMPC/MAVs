FROM avpg/cain:middle

RUN /bin/bash -c 'rm -rf /home/$USERNAME/MAVs/ros/*'

COPY ros /home/$USERNAME/MAVs/ros
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/; catkin_make'

COPY knitro-10.3.0-z-Linux-64 /home/$USERNAME/knitro-10.3.0-z-Linux-64
RUN echo 'export PATH="$HOME/knitro-10.3.0-z-Linux-64/knitroampl:$PATH"' >> ~/.bashrc
RUN echo 'export LD_LIBRARY_PATH="$HOME/knitro-10.3.0-z-Linux-64/lib:$LD_LIBRARY_PATH"' >> ~/.bashrc
RUN /bin/bash -c 'sudo chmod -R a+rX /home/mavs/knitro-10.3.0-z-Linux-64/*'

# laptop
COPY artelys_lic_897_umich_mecheng_mm_2018-02-06_knitro_1a-af-55-b8-be.txt /home/$USERNAME/

# home
#COPY artelys_lic_404_umich_mecheng_mm_2017-03-27_knitro_73-34-b5-18-fc.txt /home/$USERNAME/


# Default CMD
CMD ["/bin/bash"]
