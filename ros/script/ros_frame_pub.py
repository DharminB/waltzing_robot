#! /usr/bin/env python

from __future__ import print_function

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

class ROSFramePub(object):

    """Publish ros messages after reading gazebo messages"""

    def __init__(self):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_model_state_cb)
        self.robot_name = 'ropod'
        self.tf_br = tf.TransformBroadcaster()

    def gazebo_model_state_cb(self, msg):
        # rospy.loginfo(msg)
        robot_index = msg.name.index(self.robot_name)
        # rospy.loginfo(msg.pose[robot_index].position)
        pos = [msg.pose[robot_index].position.x, msg.pose[robot_index].position.y, msg.pose[robot_index].position.z]
        ori = [msg.pose[robot_index].orientation.x, msg.pose[robot_index].orientation.y, msg.pose[robot_index].orientation.z, msg.pose[robot_index].orientation.w]
        self.tf_br.sendTransform(pos, ori, rospy.Time.now(), '/ropod/base_link', '/ropod/odom')
        self.tf_br.sendTransform([0.3, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], rospy.Time.now(), '/ropod/laser/scan', '/ropod/base_link')

if __name__ == "__main__":
    rospy.init_node('ros_frame_pub')
    OBJ = ROSFramePub()
    rospy.loginfo("Started ROSFramePub")
    rospy.spin()
