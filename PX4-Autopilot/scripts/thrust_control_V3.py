#!/usr/bin/env python
# https://robotics.stackexchange.com/questions/8516/getting-pitch-yaw-and-roll-from-rotation-matrix-in-dh-parameter
# https://learnopencv.com/rotation-matrix-to-euler-angles/


import numpy as np
import math
# 3D point & Stamped Pose msgs
import tf
from geometry_msgs.msg import Point, PoseStamped
import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import *

Phi_p = np.zeros((3, 3)) #postive gain matrix
np.fill_diagonal(Phi_p, [1.5, 1.5, 2.2])

g = 9.8 #acceleration due to gravity 
Kp0, Kp1, m_est = 0.01, 0.01, 0.01

Lamd_p = np.zeros((3, 3))
np.fill_diagonal(Lamd_p, [1.5, 1.5, 0.6])

Kpa, Kda = 1.2, 5

Kq0, Kq1, Kq2 = 0.001, 0.001, 0.001

Lamd_q = np.zeros((3, 3))
np.fill_diagonal(Lamd_q, [0.25, 0.25, 0.6])

Phi_q = np.zeros((3, 3)) #postive gain matrix
np.fill_diagonal(Phi_q, [0.35, 0.35, 0.4])

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
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
                # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.position.z = 4
    	self.state = State()

    	self.x, self.y, self.z = 0, 0, 0
    	self.xd, self.yd, self.zd = 0, 0, 4

    	self.vx, self.vy, self.vz = 0, 0, 0
    	self.vxd, self.vyd, self.vzd = 0, 0, 0

    	self.q0, self.q1, self.q2, self.q3 = 0,0,0,1 
    	self.wx, self.wy, self.wz = 0,0,0

    	self.roll, self.pitch, self.yaw = 0,0,0 #current roll, pitch, yaw
        self.rd, self.pd, self.yawd = 0, 0, 0 #desired rpy

        self.x_acc, self.y_acc =0, 0
        self.qd_dot = np.array([0, 0, 0]) #desired angular acceleration

    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = self.x
        self.sp.position.y = self.y

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


    # basetrans = tf.transformations.translation_matrix((pose.position.x,
    #                                                    pose.position.y,
    #                                                    pose.position.z))

    # baserot = tf.transformations.quaternion_matrix((pose.orientation.x,
    #                                                 pose.orientation.y,
    #                                                 pose.orientation.z,
    #                                                 pose.orientation.w))

    # def RotationMatric(self): #R_W_B current roll pitvh yaw
    # 	RR = tf.transformations.quaternion_matrix((self.q1,
    #                                                 self.q2,
    #                                                 self.q3,
    #                                                 self.q0))
    # 	rot = RR[:-1,:-1]
    # 	RRinv = np.linalg.inv(rot)
    #     RRtransp = np.transpose(rot)
    #     return rot, RRinv ,RRtransp

    def thrust(self):
     	ep_dot = np.array([self.vx - self.vxd, self.vy - self.vyd, self.vz - self.vzd])
     	ep = np.array([self.x - self.xd, self.y - self.yd, self.z -self.zd])
     	svp = ep_dot + Phi_p.dot(ep)

     	zeta_p = np.array([ep, ep_dot])
     	zeta_norm_p = (np.linalg.norm(zeta_p))

     	Rho_p = Kp0 + Kp1*(zeta_norm_p)
     	gp = np.array([0,0,g])
     	self.tou_p = - Lamd_p.dot(svp) - Rho_p*np.sign(svp) + m_est*gp

        R, R_inv ,R_transp = self.GRotationMatricn(self.roll, self.pitch, self.yaw)
        self.tou_p = R_inv.dot(self.tou_p)

    def desired_xy_acceleeration(self): #PID for x and y error
        # print("desired_xy_acceleeration")
        xd_acc = Kpa*(self.xd - self.x) + Kda*(self.vxd - self.vx)
        yd_acc = Kpa*(self.yd - self.y) + Kda*(self.vyd - self.vy)
        # print(xd_acc, yd_acc)

        return xd_acc, yd_acc

    def desired_rp(self): #desired roll and pitch
        # print("desired_rp")
        xd_acc, yd_acc = self.desired_xy_acceleeration()
        self.rd = (1/g)* (xd_acc * np.sin(self.yawd) - yd_acc*np.cos(self.yawd))
        self.pd = (1/g)* (xd_acc * np.cos(self.yawd) + yd_acc*np.sin(self.yawd))

    def rpyRate(self):

        #conclusion
        # 1. GRotationMatricn, GRotationMatric are same 'sxyz'
        # 2. euler_matrix(roll, pitch, yaw) & euler_from_matrix(R) are same
        
        self.desired_rp()

        print(np.rad2deg(self.roll), np.rad2deg(self.pitch), np.rad2deg(self.yaw))

        R, R_inv ,R_transp = self.GRotationMatricn(self.roll, self.pitch, self.yaw)
        # rn, pn,yn = euler_from_matrix(R)
        # print(np.rad2deg(rn), np.rad2deg(pn), np.rad2deg(yn))
        # Rn, Rn_inv ,Rn_transp = self.GRotationMatric(self.roll, self.pitch, self.yaw)
        # print(R-Rn)
        # print()
        Rd, Rd_inv ,Rd_transp = self.GRotationMatricn(self.rd, self.pd, self.yawd)

        # print(Rd_transp.dot(R))
        # print(R_transp.dot(Rd))

        eq_mat = Rd_transp*(R) - R_transp*(Rd)
        # eq_mat = 1*(R - Rd)
        # print(eq_mat)
        al, be, ga = euler_from_matrix(eq_mat)
        

        # al, be, ga = self.rotationMatrixToEulerAngles(eq_mat)
        # print(np.rad2deg(al), np.rad2deg(be), np.rad2deg(ga))
        eq1 = np.array([al, be, math.pi/2 +ga])
        # eq1 = (eq1 + math.pi) % (2 * math.pi) - math.pi
        


        R, R_inv ,R_transp = self.GRotationMatric(self.roll, self.pitch, self.yaw)
        Rd, Rd_inv ,Rd_transp = self.GRotationMatric(self.rd, self.pd, self.yawd)
        
        eq = np.array([self.rd - self.roll, self.pd - self.pitch, self.yawd - self.yaw])
        # eq = (eq + math.pi) % (2 * math.pi) - math.pi

        # print("eq: ", np.rad2deg(eq))
        # print("eq1: ", np.rad2deg(eq1))
        # print(eq - eq1 )
        print("------")
        # 
        self.getRd()

        # eq[0] = eq1[0]
        # eq[2] = eq1[2]

        eq_dot = - np.array([self.wx, self.wy, self.wz]) + (Rd_transp.dot(R)).dot(self.qd_dot)
        svq = eq_dot + Phi_q.dot(eq)

        zeta_q = np.array([eq, eq_dot])
        zeta_norm_q = (np.linalg.norm(zeta_q))       
        
        Rho_q = Kq0 + Kq1*(zeta_norm_q) + Kq2*(zeta_norm_q)*(zeta_norm_q)
        self.tou_q = - Lamd_q.dot(svq) - Rho_q*np.sign(svq)

    def GRotationMatricn(self, roll, pitch, yaw): #R_W_B current roll pitvh yaw
        Rhomo = euler_matrix(roll, pitch, yaw)
        R = Rhomo[:-1,:-1]
        Rinv = np.linalg.inv(R)
        Rtransp = np.transpose(R)
        return R, Rinv, Rtransp

    def getRd(self):
        zBdes = self.tou_p / (self.tou_p**2).sum()**0.5
        # print((zBdes**2).sum()**0.5)

        xCdes = np.array([np.cos(self.yawd), np.sin(self.yawd), 0])

        yBdes = np.cross(zBdes, xCdes)
        yBdes = yBdes / (yBdes**2).sum()**0.5
        # print(yBdes)

        xBdes = np.cross(yBdes, zBdes)
        xBdes = xBdes / (xBdes**2).sum()**0.5
        # print(zBdes)
        # print(xCdes)
        # print(yBdes)
        # print(xBdes)

        Rd = np.array([zBdes, yBdes, xBdes])
        Rd = np.transpose(Rd)
        al, be, ga = euler_from_matrix(Rd)
        # print(np.rad2deg(al), np.rad2deg(be), np.rad2deg(ga))
        # print(Rd)



    def GRotationMatric(self, roll, pitch, yaw): #R_W_B current roll pitvh yaw
        sr, cr = np.sin(roll), np.cos(roll) 
        sp, cp = np.sin(pitch), np.cos(pitch)
        sy, cy = np.sin(yaw), np.cos(yaw)

        R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], 
                        [-sp , sr*cp , cp*cr]])
        Rinv = np.linalg.inv(R)
        Rtransp = np.transpose(R)
        return R, Rinv, Rtransp

        # Checks if a matrix is a valid rotation matrix.

    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        # assert(self.isRotationMatrix(R))
        singular = sy < 1e-6
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        # return np.array([x, y, z])
        # x =roll; y = pitch; z =yaw
        return x,y,z



def main():
    rospy.init_node('setpoint_node', anonymous=True)
    movement_cmd = AttitudeTarget()
    
    modes = fcuModes() # flight mode object
    cnt = Controller()
    rate = rospy.Rate(20.0)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)

    thrust_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    modes.setOffboardMode() # activate OFFBOARD mode

    # k=0
    # while k<10:     
    #     cnt.updateSp()     #give current position
    #     # cnt.x_dir()
    #     sp_pub.publish(cnt.sp)  
    #     rate.sleep()
    #     k = k + 1

    # ROS main loop
    while not rospy.is_shutdown():
    	cnt.thrust()
        # print(cnt.tou_p)
        U_pn = np.sqrt(cnt.tou_p[0]*cnt.tou_p[0] + cnt.tou_p[1]*cnt.tou_p[1] + cnt.tou_p[2]*cnt.tou_p[2])
        U_pn = np.sign(cnt.tou_p[2])*U_pn
        # R, R_inv ,R_transp = cnt.GRotationMatricn(cnt.roll, cnt.pitch, cnt.yaw)
        # R, R_inv ,R_transp = cnt.RotationMatric()
        # U_p = R_inv.dot(cnt.tou_p)
        # print(U_pn, U_p[2], (cnt.tou_p**2).sum()**0.5)


        # print(cnt.tou_p)
        # print(U_p)
        # print("--------")

        cnt.rpyRate()
        # print("current rpy: ",cnt.roll, cnt.pitch, cnt.yaw)
        # print(cnt.tou_q)
        movement_cmd.body_rate.x = -cnt.tou_q[0]
        movement_cmd.body_rate.y = -cnt.tou_q[1]
        movement_cmd.body_rate.z = -cnt.tou_q[2]
    	movement_cmd.thrust = U_pn
    	# print(movement_cmd.thrust)
    	thrust_pub.publish(movement_cmd)
        # rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

