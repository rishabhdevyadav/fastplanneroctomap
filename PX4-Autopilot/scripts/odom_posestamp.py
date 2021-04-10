#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
import numpy as np

class Controller:
    # initialization method
    def __init__(self):

        self.x, self.y, self.z = 0, 0, 0
        self.vx, self.vy, self.vz = 0, 0, 0
        self.q0, self.q1, self.q2, self.q3 = 0, 0, 0, 0
        self.wx, self.wy, self.wz = 0,0,0

        self.pose_msg = PoseStamped()

        self.att_pub = rospy.Publisher('/camera/pose', PoseStamped, queue_size=10)


    # Callbacks
    def odomCb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.q0 = msg.pose.pose.orientation.w
        self.q1 = msg.pose.pose.orientation.x
        self.q2 = msg.pose.pose.orientation.y
        self.q3 = msg.pose.pose.orientation.z

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z

        self.wx = msg.twist.twist.angular.x
        self.wy = msg.twist.twist.angular.y
        self.wz = msg.twist.twist.angular.z

    ## local position callback
    def posCb(self, msg):
        now = rospy.Time.now()
        self.pose_msg.header.stamp = now
        self.pose_msg.pose.position.x = self.x
        self.pose_msg.pose.position.y = self.y
        self.pose_msg.pose.position.z = self.z

        self.pose_msg.pose.orientation.x = self.q1
        self.pose_msg.pose.orientation.y = self.q2
        self.pose_msg.pose.orientation.z = self.q3
        self.pose_msg.pose.orientation.w = self.q0


    def pub_att(self):
        self.posCb()
        self.att_pub.publish(self.pose_msg)



# Main function
def main():
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    rospy.Subscriber('mavros/local_position/pose', Odometry, cnt.odomCb)


    while not rospy.is_shutdown():
        cnt.pub_att()
        rate.sleep()
         

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass