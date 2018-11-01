FROM avpg/virgil:base

# TODO: move this to another image.
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# Change user
USER mavs
RUN git clone -b master https://github.com/JuliaMPC/MAVs /home/$USERNAME/MAVs \
    && /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd /home/$USERNAME/MAVs/ros/src; catkin_init_workspace;sudo rosdep init;rosdep update;cd ..; catkin_make' \
    && echo "source /home/$USERNAME/MAVs/ros/devel/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'alias julia='/opt/julia-d386e40c17/bin/julia'' >> ~/.bashrc \
    && echo 'export PATH="$PATH:/opt/julia-d386e40c17/bin"' >>  ~/.bashrc

# get latest MichiganAutonomousVehicles.jl
#RUN /opt/julia-d386e40c17/bin/julia -e 'Pkg.checkout("MichiganAutonomousVehicles")'

#to avoid: https://forum.manjaro.org/t/kde-plasma-related-error-messages-in-journal/21790/2
RUN echo 'QT_LOGGING_RULES=’*=false’' \
 'export QT_LOGGING_RULES' \
 'rm -r /home/mavs/.julia/.cache '

# Default CMD
CMD ["/bin/bash"]
