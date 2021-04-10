#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from tf.transformations import *
import numpy as np

# from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Transform, Quaternion
import std_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import tf

import cubic_spline_planner


# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e


class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()

        self.x, self.y, self.z = 0, 0, 0
        self.q0, self.q1, self.q2, self.q3 = 0, 0, 0, 0

        self.error = 0
        self.vel_cmd = Twist()
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.setpose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.Kpvx, self.Kpvy, self.Kpvz, self.Kpwz = 3, 3, 3, 0.05
        self.Kdvx, self.Kdvy, self.Kdvz, self.Kdwz = 0.001, 0.001, 0.001, 0.001

        self.des_x, self.des_y, self.des_z = 0, 0, 2
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.des_vx, self.des_vy, self.des_vz = 0, 0, 0

        self.pre_time = rospy.get_time()

        self.prev_delx = (self.x - self.des_x) 
        self.prev_dely = (self.y - self.des_y) 
        self.prev_delz = (self.z - self.des_z) 
        self.prev_error = 0


        self.sp = PoseStamped()
        self.sp.pose.position.x = 0
        self.sp.pose.position.y = 0
        self.sp.pose.position.z = 2


    def path_cb(self, msg):
        traj_local = msg
        size = (np.size(traj_local.poses)) - 1
        # print(traj_local.poses[size].pose.position.z)

        self.des_x = (traj_local.poses[size].pose.position.x)
        self.des_y = (traj_local.poses[size].pose.position.y)
        self.des_z = (traj_local.poses[size].pose.position.z)


    def posCb(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.q0 = msg.pose.orientation.w
        self.q1 = msg.pose.orientation.x
        self.q2 = msg.pose.orientation.y
        self.q3 = msg.pose.orientation.z

        orientation_list = [self.q1, self.q2, self.q3, self.q0]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def newPoseCB(self, msg):
        # if(self.sp.pose.position != msg.pose.position):
            # print("New pose received")
        # self.des_x = msg.pose.position.x
        # self.des_y = msg.pose.position.y
        # self.des_z = msg.pose.position.z

        # now = rospy.Time.now()
        # self.sp.header.stamp = now
        self.sp.pose.position.x = msg.pose.position.x
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
   
        self.sp.pose.orientation.x = msg.pose.orientation.x
        self.sp.pose.orientation.y = msg.pose.orientation.y
        self.sp.pose.orientation.z = msg.pose.orientation.z
        self.sp.pose.orientation.w = msg.pose.orientation.w


    def angError(self):
        yaw_new = (np.rad2deg((self.yaw)))
        # print(yaw_new)
    
        if yaw_new < 0:
            yaw_new = yaw_new + 360
        dy = -self.y + self.des_y
        dx = -self.x + self.des_x

        self.des_yaw = ((math.atan2(dy, dx)))

        theta = np.rad2deg((math.atan2(dy, dx)))
        if theta < 0:
            theta = theta + 360
        self.error = (theta - yaw_new)
        if self.error > 180:
            self.error = self.error - 360
        if self.error <- 180:
            self.error = self.error + 360

    def update_desxy(self, desx, desy, endpath):
        self.des_x = desx
        self.des_y = desy
        self.pathstatus = endpath

    def PD_control(self):
        self.angError()
        self.delx = (self.x - self.des_x) 
        self.dely = (self.y - self.des_y)
        self.delz = (self.z - self.des_z)

        # self.des_vx = self.Kpvx * (self.x - self.des_x) 
        # self.des_vy = self.Kpvy * (self.y - self.des_y)
        # self.des_vz = self.Kpvz * (self.z - self.des_z)
        # self.des_wz = self.Kpwz * (self.error) 
        dt = rospy.get_time() - self.pre_time
        self.pre_time = self.pre_time + dt
        if dt > 0.04:
            dt = 0.04

        self.des_vx = self.Kpvx * self.delx + self.Kdvx * (self.delx - self.prev_delx)/dt
        self.des_vy = self.Kpvy * self.dely + self.Kdvy * (self.dely - self.prev_dely)/dt
        self.des_vz = self.Kpvz * self.delz + self.Kdvz * (self.delz - self.prev_delz)/dt
        self.des_wz = self.Kpwz * (self.error) + self.Kdwz * (self.error - self.prev_error)/dt

        self.prev_delx = self.delx
        self.prev_dely = self.dely
        self.prev_delz = self.delz
        self.prev_error = self.error


        self.des_vx = np.maximum(-0.5, np.minimum(self.des_vx, 0.5))
        self.des_vy = np.maximum(-0.5, np.minimum(self.des_vy, 0.5))
        self.des_vz = np.maximum(-2.0, np.minimum(self.des_vz, 0.5))


        if  np.sqrt(np.square(self.x - self.des_x)) <= 0.1:
            self.des_vx = 0
        if  np.sqrt(np.square(self.y - self.des_y)) <= 0.1:
            self.des_vy = 0
        if  np.sqrt(np.square(self.z - self.des_z)) <= 0.1:
            self.des_vz = 0
        if  np.sqrt(np.square(self.error)) <= 5:
            self.des_wz = 0

        # print(self.des_vx, self.des_vy, self.des_vz)

        self.vel_cmd.linear.x = -self.des_vx
        self.vel_cmd.linear.y = -self.des_vy
        self.vel_cmd.linear.z = -self.des_vz
        if self.pathstatus == 0:
            self.des_wz = 0
        self.vel_cmd.angular.z = self.des_wz

    def setPoselocal(self):
        # des_ang_list = [0, 0, 0]
        if self.pathstatus == 1:
            q = (quaternion_from_euler(0.0, 0.0 , self.des_yaw))
            self.lastdesyaw = self.des_yaw
            # print(lastdesyaw)
            # print("1")
        
        if self.pathstatus == 0:
            # print("0")
            q = (quaternion_from_euler(0.0, 0.0 , self.lastdesyaw))

        now = rospy.Time.now()
        self.sp.header.stamp = now
        self.sp.pose.position.x = self.des_x
        self.sp.pose.position.y = self.des_y
        self.sp.pose.position.z = self.des_z
   
        # self.sp.pose.orientation.x = q[0]
        # self.sp.pose.orientation.y = q[1]
        # self.sp.pose.orientation.z = q[2]
        # self.sp.pose.orientation.w = q[3]

        self.sp.pose.orientation = Quaternion(*q)


    def pub_vel(self):
        self.PD_control()
        self.vel_pub.publish(self.vel_cmd)

        self.setPoselocal()
        # self.setpose_pub.publish(self.sp)

    def stateCb(self, msg):
        self.state = msg

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Main function
def main():

    theta = np.linspace(0, -2*np.pi, 200)
    radius = 3
    ax = radius*np.cos(theta)
    ay = radius*np.sin(theta)


    # ax = [0,  2,4,6,9,11,12,14,16]
    # ay = [0,  1,0,-1,0,0,0,0,0 ]

    cx, cy, _, _, _ = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)


    rospy.init_node('setpoint_node', anonymous=True)
    modes = fcuModes()  #flight modes


    cnt = Controller()  # controller object
    rate = rospy.Rate(30)

    # Subscribe to drone's local position

    # rospy.Subscriber('/path', Path, self.path_cb, queue_size=10)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)

    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("ARMING")
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    k=0
    while k<20:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    i = 0
    while not rospy.is_shutdown():
        error = np.sqrt(np.square(cnt.x - cnt.des_x) +
                       np.square(cnt.y - cnt.des_y) +
                       np.square(cnt.z - cnt.des_z))
        if error <= 0.3:
            i = i+1

        pathend = 1
        if i > np.size(cx)-2:
            i = np.size(cx)-2
            pathend = 0

        desx = cx[i]
        desy = cy[i]
        
        
        # print (error, desx, desy)
        cnt.update_desxy(desx, desy, pathend)

        cnt.pub_vel()
        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass