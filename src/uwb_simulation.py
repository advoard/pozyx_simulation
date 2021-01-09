#!/usr/bin/env python3
'''
__author__ = "Bekir Bostanci"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Bekir Bostanci"
__email__ = "bekirbostanci@gmail.com"
'''

import rospy

from pozyx_simulation.msg import  uwb_data
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import tf 

import math
import numpy as np

import time
import threading
import os, sys
import random




rospy.init_node('uwb_simulation', anonymous=True)
#distances are publishing with uwb_data_topic
pub = rospy.Publisher('uwb_data_topic', uwb_data, queue_size=10)

def get_anchors_pos():
    max_anchor = 100
    sensor_pos = []   
    uwb_id = 'uwb_anchor_'
    listener = tf.TransformListener()
    
    for i in range(max_anchor):
        try:
            time.sleep(0.3)
            (trans,rot) = listener.lookupTransform('/map', uwb_id+str(i), rospy.Time(0))
            sensor_pos.append(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            break



    if sensor_pos == [] :
        rospy.logwarn("There is not found any anchors. Function is working again.")    
        get_anchors_pos()
    else: 
        rospy.loginfo("UWB Anchor List:\nWarning : uint is mm \n" + str(sensor_pos))    


    return sensor_pos

def calculate_distance(uwb_pose):
    #pose comes in gazebo/model_states (real position)

    #describe 2 points
    p1 = np.array(uwb_pose)
    p2 = np.array([robot_pose.x, robot_pose.y, robot_pose.z])


    #difference between robot and uwb distance
    uwb_dist = np.sum((p1-p2)**2, axis=0)
    #add noise 
    uwb_dist=uwb_dist+np.random.normal(0, uwb_dist*0.015,1)  
    return np.sqrt(uwb_dist)


def uwb_simulate():

    time.sleep(0.1)
    all_distance = [] 
    all_destination_id = []

    for i in range(len(sensor_pos)):
        #calculate distance uwb to robot for all anchors 
        dist = calculate_distance(sensor_pos[i])   
        all_distance.append(dist) 
        
        all_destination_id.append(i)
        
    #publish data with ROS             
    publish_data(all_destination_id , all_distance)    


def publish_data(all_destination_id, all_distance):
    #uwb message type is a special message so that firstly describe this message 
    uwb_data_cell = uwb_data()
    uwb_data_cell.destination_id=all_destination_id
    uwb_data_cell.stamp = [rospy.Time.now(),rospy.Time.now(),rospy.Time.now()]
    uwb_data_cell.distance = all_distance
    pub.publish(uwb_data_cell)


def subscribe_data(data):
    #for the get real position of robot subscribe model states topic  
    global robot_pose
    robot_pose =data.pose.pose.position
        
    uwb_simulate()

if __name__ == "__main__":
    #get uwb anchors postion
    global sensor_pos 
    sensor_pos = get_anchors_pos()


    time.sleep(0.5)

    #gazebo urdf file you have to add plugin firstly 
    rospy.Subscriber('/ground_truth/state', Odometry, subscribe_data)


    rospy.spin()
    
sys.exit()