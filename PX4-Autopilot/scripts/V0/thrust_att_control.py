#!/usr/bin/env python

import numpy as np
import math
import tf
from scipy.integrate import odeint
from geometry_msgs.msg import Point, PoseStamped
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import *

Phi_p = np.zeros((3, 3)) #postive gain matrix
Lamd_p = np.zeros((3, 3))

# np.fill_diagonal(Phi_p, [1.5, 1.5, 2.2])
# np.fill_diagonal(Lamd_p, [1.5, 1.5, 0.6])

np.fill_diagonal(Phi_p, [1.5, 1.5, 2])
np.fill_diagonal(Lamd_p, [1.5, 1.5, 0.5])

g = 9.8 #acceleration due to gravity 
Kp0 = np.ones(3) 
Kp1 = np.ones(3)
m_est = 0.01
Lamd_p = np.array([1.0, 1.0, 1.0])


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
        
    
    	self.state = State()

    	self.x, self.y, self.z = 0, 0, 0
    	# self.xd, self.yd, self.zd = self.sp.pose.position.x, self.sp.pose.position.x, self.sp.pose.position.x

    	self.vx, self.vy, self.vz = 0, 0, 0
    	self.vxd, self.vyd, self.vzd = 0, 0, 0

        self.des_yaw = 0

    	self.q0, self.q1, self.q2, self.q3 = 0, 0, 0, -1 
    	self.wx, self.wy, self.wz = 0,0,0

    	self.roll, self.pitch, self.yaw = 0,0,0 #current roll, pitch, yaw
        self.rd, self.pd, self.yawd = 0, 0, 0 #desired rpy

        self.qd_dot = np.array([0, 0, 0]) #desired angular acceleration

        self.sp = PoseStamped()
        self.ALT_SP = 1.0
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])

    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = self.x
        self.sp.position.y = self.y

    def newPoseCB(self, msg):
        self.sp.pose.position.x = msg.pose.position.x
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
    
        self.sp.pose.orientation.x = msg.pose.orientation.x
        self.sp.pose.orientation.y = msg.pose.orientation.y
        self.sp.pose.orientation.z = msg.pose.orientation.z
        self.sp.pose.orientation.w = msg.pose.orientation.w

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_quat[0] = msg.pose.orientation.x
        self.local_quat[1] = msg.pose.orientation.y
        self.local_quat[2] = msg.pose.orientation.z
        self.local_quat[3] = msg.pose.orientation.w

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

        orientation_list = [self.q1, self.q2, self.q3, self.q0]
    	(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def getTime(self, time):
        self.time = time

    def Sigmoid(self, s, var_pi):
        if s > var_pi:            
            return s/np.absolute(s)
        else:
            return s/var_pi

    def vector2Array(vector):
        return np.array([vector.x, vector.y, vector.z])

    def getThrust(self):

        ep = vector2Array(self.)

        Kp0 += dot_Kp0*dt
        Kp1 += dot_Kp1*dt
        m_est += dot_m*dt
        Xi_p = ep
        Rho_p = Kp0 + np.absolute(Xi_p).dot(Kp1)

        delTau = np.zeros(3)
        delTau[0] = Rho_p[0]*Sigmoid(svp[0], var_pi)
        delTau[1] = Rho_p[1]*Sigmoid(svp[1], var_pi)
        delTau[2] = Rho_p[2]*Sigmoid(svp[2], var_pi)
        tau = -np.multiply(Lamd_p,svp) - delTau + m_est*np.array([0.0,0.0,9.8])

    # def thrust(self):
    #  	ep_dot = np.array([self.vx - self.vxd, self.vy - self.vyd, self.vz - self.vzd])
    #  	ep = np.array([self.x - self.sp.pose.position.x, self.y - self.sp.pose.position.y, self.z -self.sp.pose.position.z])
    #  	self.svp = ep_dot + Phi_p.dot(ep)

    #  	self.zeta_p = np.array([ep, ep_dot])
    #  	zeta_norm_p = (np.linalg.norm(self.zeta_p))

    #     Kp0 = self.sol_Kp(self.time,0.01,1)
    #     Kp1 = self.sol_Kp(self.time,0.01,2) 

    #  	Rho_p = Kp0 + Kp1*(zeta_norm_p)
    #  	self.gp = np.array([0,0,g])

    #     m_est = self.sol_m(self.time, 0.01)
    #  	self.tou_p = - Lamd_p.dot(self.svp) - Rho_p*np.sign(self.svp) + m_est*self.gp

    #     R, R_inv ,R_transp = self.GRotationMatricn(self.roll, self.pitch, self.yaw)
    #     self.tou_p = R_inv.dot(self.tou_p)
    #     self.tou_p = np.around(self.tou_p, decimals = 5)

    # def a_des(self):
        
    #     desVel = np.array([self.vxd, self.vyd, self.vzd])

    #     currPos = np.array([self.x, self.y, self.z])
    #     desPos = np.array([self.sp.pose.position.x, self.sp.pose.position.y, self.sp.pose.position.z])
    #     currVel = np.array([self.vx, self.vy, self.vz])

    #     errPos = (currPos - desPos) 
    #     errVel = (currVel - desVel)
    #     self.des_a = Kpos*errPos + Kvel*errVel + self.gp

    def acc2quaternion(self):
        proj_xb_des = np.array([np.cos(self.des_yaw), np.sin(self.des_yaw), 0.0])

        zb_des = self.des_a / np.linalg.norm(self.des_a)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
        rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))
        return rotmat

    def desired_quat(self):
        self.a_des()
        r_cur = quaternion_matrix(self.local_quat)[:3,:3]
        r_des = self.acc2quaternion()
        rot_44 = np.vstack((np.hstack((r_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))
        self.quat_des = quaternion_from_matrix(rot_44)

    def GRotationMatricn(self, roll, pitch, yaw): #R_W_B current roll pitvh yaw
        Rhomo = euler_matrix(roll, pitch, yaw)
        R = Rhomo[:-1,:-1]
        Rinv = np.linalg.inv(R)
        Rtransp = np.transpose(R)
        return R, Rinv, Rtransp

    # def sol_m(self, t, y0):
    #     y0 = 1
    #     alpham = 50
    #     b = -(self.gp).dot(self.svp)
    #     # print(b)
    #     a = alpham
    #     c = y0 - (b/a)
    #     y = (b/a) + c*np.exp(-a*t)
    #     # print(b, b/a)
    #     return y

    # def sol_Kp(self,t,y0,k): #k=1,2 KP0 and KP1
    #     alphap = 1
    #     a = alphap
    #     b = np.linalg.norm(self.svp)*np.power(np.linalg.norm(self.zeta_p),k)
    #     c = y0 - (b/a)
    #     y = (b/a) + c*np.exp(-a*t)
    #     return y



def main():
    rospy.init_node('setpoint_node', anonymous=True)
    
    modes = fcuModes() # flight mode object
    cnt = Controller()
    rate = rospy.Rate(50.0)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)

    movement_cmd = AttitudeTarget()
    thrust_cmd = Thrust()
    orientation_cmd = PoseStamped()

    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    bodyrate_thrust_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    orientation_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
    thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    k=0
    while k < 10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1
    modes.setOffboardMode() # activate OFFBOARD mode

    # ROS main loop
    now1 = rospy.Time.now()
    time1 = now1.secs + now1.nsecs/1e9
    
    while not rospy.is_shutdown():

        #---------------------------------------
        now = rospy.Time.now()
        time = (now.secs + now.nsecs/1e9) - time1
        cnt.getTime(time)

        cnt.thrust()

        num_array = [0,0,0,0,0,0,0] #take 10 elements
        num_array.append(num_array.pop(0)) 
        num_array[-1] = cnt.tou_p[2]
        Avg = sum(num_array)/np.size(num_array)
        print(Avg)

        thrust_cmd.header.stamp = now
        thrust_cmd.thrust = Avg
        thrust_pub.publish(thrust_cmd)

        cnt.desired_quat()
        orientation_cmd.header.stamp = now
        orientation_cmd.pose.orientation.x = cnt.quat_des[0]
        orientation_cmd.pose.orientation.y = cnt.quat_des[1]
        orientation_cmd.pose.orientation.z = cnt.quat_des[2]
        orientation_cmd.pose.orientation.w = cnt.quat_des[3]
        orientation_pub.publish(orientation_cmd)

        rate.sleep()


        #------------------------


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

