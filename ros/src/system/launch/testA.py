#!/usr/bin/env python
import roslaunch
import rospy

rospy.init_node('testA', anonymous=True)
rospy.on_shutdown(self.shutdown)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,["demoA.launch"])

launch.start()

launch.shutdown()
