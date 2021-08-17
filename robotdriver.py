#!/usr/bin/env python
from __future__ import print_function

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from actionlib_msgs.msg import GoalID


import numpy as np
import time
import array
from math import pi,cos,sin,exp,atan2,sqrt
import json

class robotdriver():
    def __init__(self):
        rospy.init_node('robotdriver')   
        
        self.initial_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel_motor_in', Twist, queue_size=10)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel',GoalID,queue_size=10)
        self.enc_initial = rospy.Publisher('encoder_initial_pose',PoseWithCovarianceStamped,queue_size=10)

        self.linear_x = 0
        self.angular_z = 0
        self.offline_linear_x = 0
        self.offline_angular_z = 0
        self.online_linear_x = 0
        self.online_angular_z = 0
        self.initial_x = 0
        self.initial_y = 0
        self.initial_z = 0
        self.initial_angle = 0
        self.amcl_settled = False
        self.orientation_settled = False
        self.position_settled = False

        self.lgps_x_sum = 0
        self.lgps_y_sum = 0
        self.lgps_z_sum = 0

        self.position_counter = 0




        #rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.on_path_data)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.on_amcl_pose)
        rospy.Subscriber('/lgps_pose', PointStamped, self.on_lgps_data)
        rospy.Subscriber('/imu/data2', Imu, self.on_imu_data)
        rospy.Subscriber('/offline_cmd_vel', Twist, self.on_offline_cmd_vel)
        rospy.Subscriber('/online_cmd_vel', Twist, self.on_online_cmd_vel)
        rospy.Subscriber('/cmd_vel_planner', Twist, self.on_cmd_vel)
            
        




    def on_offline_cmd_vel(self,data):
        self.offline_linear_x = data.linear.x
        self.offline_angular_z = data.angular.z
        self.linear_x = 0
        self.angular_z = 0
        self.online_linear_x = 0
        self.online_angular_z = 0
        cmdData = data
        self.cmd_pub.publish(cmdData)
        cancelData = GoalID()
        self.move_base_cancel.publish(cancelData)

    def on_online_cmd_vel(self,data):
        self.online_linear_x = data.linear.x
        self.online_angular_z = data.angular.z
        if((self.offline_linear_x == 0)and(self.offline_angular_z == 0)):
            cmdData = data
            cancelData = GoalID()
            self.move_base_cancel.publish(cancelData)
        else:
            cancelData = GoalID()
            self.move_base_cancel.publish(cancelData)
            self.linear_x = 0
            self.angular_z = 0
            cmdData = Twist()
            cmdData.linear.x = self.offline_linear_x
            cmdData.angular.z = self.offline_angular_z
        self.cmd_pub.publish(cmdData)

    def on_cmd_vel(self,data):
        if((self.offline_linear_x == 0)and(self.offline_angular_z == 0)):
            if((self.online_linear_x == 0)and(self.online_angular_z == 0)):
                cmdData = data
            else:
                cancelData = GoalID()
                self.move_base_cancel.publish(cancelData)
                self.linear_x = 0
                self.angular_z = 0
                cmdData = Twist()
                cmdData.linear.x = self.online_linear_x
                cmdData.angular.z = self.online_angular_z
        else:
            cancelData = GoalID()
            self.move_base_cancel.publish(cancelData)
            self.linear_x = 0
            self.angular_z = 0
            self.online_linear_x = 0
            self.online_angular_z = 0
            cmdData = Twist()
            cmdData.linear.x = self.offline_linear_x
            cmdData.angular.z = self.offline_angular_z
        self.cmd_pub.publish(cmdData)
    
    def on_imu_data(self,data):
        self.orientation = data.orientation
        if(self.orientation_settled == False):
            self.initial_angle = euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])[2]

        self.orientation_settled = True
        
    
    def eucDist(self,a,b):
        dist = np.linalg.norm(np.asarray(a)-np.asarray(b))
        return dist
        
    def on_lgps_data(self,data):
        self.lgps_x = data.point.x
        self.lgps_y = data.point.y
        self.lgps_z = data.point.z
        if((self.position_settled == False)and(self.orientation_settled)):
            self.position_counter = self.position_counter + 1
            self.lgps_x_sum = self.lgps_x_sum + self.lgps_x
            self.lgps_y_sum = self.lgps_y_sum + self.lgps_y
            self.lgps_z_sum = self.lgps_z_sum + self.lgps_z

            if(self.position_counter>4):
                self.position_counter=5
                print("Position Settled")
                self.initial_x = self.lgps_x_sum/5.0
                self.initial_y = self.lgps_y_sum/5.0
                self.initial_z = self.lgps_z_sum/5.0

                print(self.initial_x,self.initial_y,self.initial_z)
                amclinitialpose = PoseWithCovarianceStamped()
                amclinitialpose.header.frame_id = "map"
                amclinitialpose.header.stamp = rospy.Time.now()
                amclinitialpose.pose.pose.position.x = self.lgps_x
                amclinitialpose.pose.pose.position.y = self.lgps_y
                amclinitialpose.pose.pose.orientation = self.orientation
                amclinitialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

                self.initial_pub.publish(amclinitialpose)
                print("Position Published")
                self.position_settled = True
        

    

    def mod_pi(self,angle):
        while(angle>(2*pi)):
            angle=angle-2*pi
        while(angle<0):
            angle=angle+2*pi
        return (angle)
        
    def on_amcl_pose(self,data):
        cov = list(data.pose.covariance)
        quaternion = data.pose.pose.orientation
        eulers = euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
        self.amclAngle = eulers[2]
        self.amclX = data.pose.pose.position.x
        self.amclY = data.pose.pose.position.y
        if(self.position_settled):
            dist = self.eucDist([self.amclX,self.amclY],[self.lgps_x,self.lgps_y])
            if(dist>4.0):
                self.amcl_settled = False
            
        if((self.amcl_settled == False)and(self.position_settled)and(self.orientation_settled)):
            self.amcl_settled = True
            amclinitialpose = PoseWithCovarianceStamped()
            amclinitialpose.header.frame_id = "map"
            amclinitialpose.header.stamp = rospy.Time.now()
            amclinitialpose.pose.pose.position.x = self.lgps_x
            amclinitialpose.pose.pose.position.y = self.lgps_y
            amclinitialpose.pose.pose.orientation = self.orientation
            amclinitialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
            self.initial_pub.publish(amclinitialpose)
            print("AMCL Settled")
        


    def shutdown(self):
        pass          



if __name__ == '__main__':
    rrl = robotdriver()
    rospy.on_shutdown(rrl.shutdown)
    r = rospy.Rate(0.01)
    while not rospy.is_shutdown():
        r.sleep()
    rrl.shutdown()
    
