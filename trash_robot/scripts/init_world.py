#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 15 20:27:04 2022

@author: lukas
"""

from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import *
from gazebo_msgs.srv import *

# remove all beer cans
delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
delete('beer')
delete('beer_0')
delete('beer_1')

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

# spawn trash bin
spawn_model_client(
    model_name='trash_bin',
    model_xml=open('/home/lukas/.gazebo/models/cardboard_box/model.sdf', 'r').read(),
    robot_namespace='/park',
    initial_pose=Pose(position=Point(2.5,-4.1,0),orientation=Quaternion(0,0,0,0)),
    reference_frame='world'
)

# spawn cinder_block
spawn_model_client(
    model_name='cinder_block1',
    model_xml=open('/home/lukas/.gazebo/models/cinder_block/model.sdf', 'r').read(),
    robot_namespace='/park',
    initial_pose=Pose(position=Point(-2,-2.1,0),orientation=Quaternion(0,0,0,0)),
    reference_frame='world'
)

# spawn cinder_block
spawn_model_client(
    model_name='cinder_block2',
    model_xml=open('/home/lukas/.gazebo/models/cinder_block/model.sdf', 'r').read(),
    robot_namespace='/park',
    initial_pose=Pose(position=Point(-2,-2.9,0),orientation=Quaternion(0,0,0,0)),
    reference_frame='world'
)
