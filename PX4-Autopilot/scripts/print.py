
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.orientation
    position_x =   msg.pose.position
    position_list = [ position_x.x ,position_x.y , position_x.z ] 
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    (roll, pitch, yaw)  = np.rad2deg((roll, pitch, yaw) )
    ##(X, Y, Z ) = position_list
    print "orentation", np.round(roll , 3) , np.round(pitch , 3), np.round(yaw , 3)   
    
    print( "posiiton"+ str(np.round(position_list ,  3)) )
    print "----------"

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/mavros/local_position/pose', PoseStamped, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()
    time.sleep(0.5)
