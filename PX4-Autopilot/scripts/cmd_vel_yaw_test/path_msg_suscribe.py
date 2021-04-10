#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class EchoPath:
    def __init__(self):
        # self.path_pub = rospy.Publisher('/path', Path, latch=True, queue_size=10)
        # self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.odom_cb, queue_size=10)
        # self.path = Path()

        self.path_sub = rospy.Subscriber('/path', Path, self.path_cb, queue_size=10)


    def path_cb(self, msg):
        des_pos = PoseStamped()
        # if self.current_route is not None:
        traj_local = msg
        # while(i<np.size(traj_local.poses)):
        size = (np.size(traj_local.poses)) - 1
        # print(size)
        # print(traj_local.poses[0].pose.position.x)
        print(traj_local.poses[size].pose.position.z)




        # print(msg)
        # cur_pose = PoseStamped()
        # cur_pose.header = msg.header
        # cur_pose.pose = msg.pose.pose
        # self.path.header = msg.header
        # self.path.poses.append(cur_pose)
        # self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('path_echo')
    odom_to_path = EchoPath()
    rospy.spin()