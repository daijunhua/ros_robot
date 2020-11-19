#!/usr/bin/env python

"""
    ar_follower.py - Version 1.0 2013-08-25
    
    Follow an AR tag published on the /ar_pose_marker topic.  The /ar_pose_marker topic
    is published by the ar_track_alvar package
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Transform, Point
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
from math import copysign, atan2, sin, cos, sqrt, radians

class ARFollower():
    def __init__(self):
        rospy.init_node("follow")
                        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate) 
        
        # The maximum rotation speed in radians per second
        #self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        #self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.5)
        
        # The maximum distance a target can be from the robot for us to track
        #self.max_x = rospy.get_param("~max_x", 20.0)
        
        # The goal distance (in meters) to keep between the robot and the marker
        self.goal_x = rospy.get_param("~goal_x", 0.6)
        
        # Do not change goal's position within this distance range of goal distance(in meters) adjustment
        self.distance_threshold = rospy.get_param("~robot_goal_distance_threshold", 0.1)
        
        # Do not change goal's dirction(orientation) within this angle range of goal direction adjustment
        self.yaw_threshold = rospy.get_param("~robot_goal_yaw_threshold", radians(10))
        self.last_rt_yaw = 0

        # How much do we weight the goal distance (x) when making a movement
        #self.x_scale = rospy.get_param("~x_scale", 0.5)

        # How much do we weight y-displacement when making a movement        
        #self.y_scale = rospy.get_param("~y_scale", 1.0)
        
        # The max linear speed in meters per second
        #self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        #self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)

        # Publisher to control the robot's movement
	self.goal_topic = 'move_base_simple/goal'
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=5)
        
        # Intialize the movement command
        self.goal_cmd = PoseStamped()
        self.last_rt_position = Point()
        self.goal = Point()

        # Set flag to indicate when the AR marker is visible
        self.target_visible = False
        
	#self.pitch_joint = 'pitch_joint'
	self.yaw_joint = 'yaw_joint'

        # the transformation between 'base_link' and 'odom'
        self.tf = Transform()

	# Subscribe the the 'tf' topic so we can know the transformation between 'base_link' and 'odom'
        rospy.loginfo("Subscribing to tf...")
        rospy.Subscriber('tf', tfMessage, self.get_base_link)

        # Subscribe the the 'joint_states' topic so we can know how the joints are positioned
        rospy.loginfo("Subscribing to joint_states...")
        
        self.joint_state = JointState()
        
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)
        
        # Wait until we actually have joint state values
        while self.joint_state == JointState():
	    rospy.loginfo("Waiting for joint_states topic...")
            rospy.sleep(1)

        # Wait for the ar_pose_marker topic to become available
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        
        # Subscribe to the ar_pose_marker topic to get the image width and height
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)
        
        rospy.loginfo("Marker messages detected. Starting follower...")
        
        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
            # Send the Twist command to the robot
            self.goal_pub.publish(self.goal_cmd)
            
            # Sleep for 1/self.rate seconds
            r.sleep()


    def get_base_link(self, msg):
        try:
            tf_child = msg.transforms[0].child_frame_id
            tf_ = msg.transforms[0].transform
            if tf_child == 'base_link':
                self.tf = tf_
                #rospy.loginfo("---------------tf.x: %f",self.tf.translation.x)
            else:
                return
        except:
            pass


    def update_joint_state(self, msg):
        try:
            test = msg.name.index(self.yaw_joint)
            self.joint_state = msg
        except:
            pass
        

    def set_cmd_vel(self, msg):
        # Pick off the first marker (in case there is more than one)
        try:
            marker = msg.markers[0]
            #if not self.target_visible:
                #rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
        except:
            #if self.target_visible:
                #rospy.loginfo("FOLLOWER LOST Target!")
            self.target_visible = False
            
            return
                
        # Get pan(yaw) of the head
        pan = self.joint_state.position[self.joint_state.name.index(self.yaw_joint)]
        
        # Calculate the marker's yaw to the camera transform
        cx = marker.pose.pose.position.x
        cy = marker.pose.pose.position.y
        theta_c = atan2(cy, cx)

        # Calculate the marker's yaw to the robot transform(base_link)
        theta_r = theta_c + pan

        # Get the robot('base_link') rotation to the 'odom'
        rt_q_z = self.tf.rotation.z
        rt_q_w = self.tf.rotation.w
        data_y = 2 * rt_q_z * rt_q_w
        data_x = 1 - (2 * rt_q_z * rt_q_z)
        rt_yaw = atan2(data_y, data_x)

        # Set the goal for robot('base_link') rotation to the 'odom'
        rt_yaw = rt_yaw + theta_r

        # If the angle of the goal pose changes sufficiently, change the goal orientation
        if abs(rt_yaw - self.last_rt_yaw) > self.yaw_threshold:
            self.goal_cmd.pose.orientation.x = 0
            self.goal_cmd.pose.orientation.y = 0
            self.goal_cmd.pose.orientation.z = sin(rt_yaw/2)
            self.goal_cmd.pose.orientation.w = cos(rt_yaw/2)
            self.last_rt_yaw = rt_yaw

        # the distance between marker and robot
        distance = sqrt(cx * cx + cy * cy)

        # the distance between goal and robot
        distance = distance - self.goal_x

        # the position vector between goal and robot in the 'odom' transform
        r_x = distance * cos(rt_yaw)
        r_y = distance * sin(rt_yaw)

        # the goal position vector in the 'odom' transform
        self.goal.x = r_x + self.tf.translation.x
        self.goal.y = r_y + self.tf.translation.y

        # the distance of goal position to the last goal position
        dx = self.goal.x - self.last_rt_position.x
        dy = self.goal.y - self.last_rt_position.y
        d_dis = sqrt(dx * dx + dy * dy)
        rospy.loginfo("---------------d_dis: %f",d_dis)
        rospy.loginfo("---------------last_x: %f,last_y: %f",self.last_rt_position.x, self.last_rt_position.y)

        # If the position of the goal pose changes sufficiently, change the goal position
        if d_dis > self.distance_threshold:
            self.goal_cmd.pose.position.x = self.goal.x
            self.goal_cmd.pose.position.y = self.goal.y
            self.goal_cmd.pose.position.z = self.tf.translation.z

            # record self.goal_cmd.pose.position
            self.last_rt_position.x = self.goal.x
            self.last_rt_position.y = self.goal.y
            self.last_rt_position.z = self.tf.translation.z

        # the frame id of the goal msg
        self.goal_cmd.header.frame_id = 'map'


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.goal_pub.publish(PoseStamped())
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        ARFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("follow node terminated.")

