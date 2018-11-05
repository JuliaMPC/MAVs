system_pkg_path="$(rospack find system)"
rosbag filter ~/.ros/robot_tf.bag $system_pkg_path/data/desired_tf.bag 'm.transforms[0].header.frame_id == "map" and m.transforms[0].child_frame_id == "base_footprint"'
rostopic echo -b $system_pkg_path/data/desired_tf.bag -p /tf > $system_pkg_path/data/robot_tf.txt


# rostopic echo -b /home/mavs/MAVs/shared_dir/opt.bag -p /tf >/home/mavs/MAVs/shared_dir/opt.bag /opt.txt
