#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 14 17:56:55 2022

@author: lukas
"""

from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import *

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
# spawn new beer can
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(
    model_name='beer',
    model_xml=open('/home/lukas/.gazebo/models/beer/model.sdf', 'r').read(),
    robot_namespace='/park',
    initial_pose=Pose(position=Point(-2,0.3,0),orientation=Quaternion(0,0,0,0)),
    reference_frame='world'
)

# spawn new beer can
spawn_model_client(
    model_name='beer_0',
    model_xml=open('/home/lukas/.gazebo/models/beer/model.sdf', 'r').read(),
    robot_namespace='/park',
    initial_pose=Pose(position=Point(-4.5,-3,0),orientation=Quaternion(0,0,0,0)),
    reference_frame='world'
)

# spawn new beer can
spawn_model_client(
    model_name='beer_1',
    model_xml=open('/home/lukas/.gazebo/models/beer/model.sdf', 'r').read(),
    robot_namespace='/park',
    initial_pose=Pose(position=Point(0.5,-2,0),orientation=Quaternion(0,0,0,0)),
    reference_frame='world'
)