#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 11 15:53:10 2022

@author: lukas

activate python3: https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/
"""
import os
import numpy as np
import rospy
import geometry_msgs
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, DeleteModel, SetModelState
from std_msgs.msg import String
from rosplan_dispatch_msgs.srv import PlanningService


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def createMapping():
    x = [-5.9, 3.5]
    y = [0.3, -4.1]
    
    size = 99
    
    x_range = np.linspace(x[0], x[1], size)
    y_range = np.linspace(y[0], y[1], size)
    
    mapping = list()
    for i in range(0, size):
        for j in range(0, size):
            mapping.append([x_range[i], y_range[j]])
    return mapping

def createPddl(mapping, robotPosition, beerPosition, notclear):
    print("Create problem for planner")
    f=open("..//planner//problem.pddl", "w")
    f.write("(define (problem robot)  \r\n")
    f.write("  (:domain robot)  \r\n")
    
    # objects
    f.write("  (:objects")
    
    for i in range(0, len(mapping)):
        f.write(" a%d" %i)
        
    f.write(")  \r\n")
    
    f.write("  (:init  \r\n")
    
    # clear
    notclearlist = [-1]
    if notclear != 0:
        for j in range(notclear-10, notclear+10):
            for i in range(j, j + 1980, 99):
                notclearlist.append(i)
                
    for i in range(0, len(mapping)):
        if i in notclearlist:
            f.write("	 (not(clear a%d))" %i)
        else:
            f.write("	 (clear a%d)" %i)
    f.write("\r\n")            
    
    # neighbors
    for i in range(0, len(mapping)):
        
        # top
        neighborTop = i - 1
        if(i%99 != 0):
            f.write("	 (neighborTop a%d a%d)" %(i, neighborTop))
            
        # bootom
        neighborBottom = i + 1
        if(i == 0 or i%98 != 0):
            f.write("	 (neighborBottom a%d a%d)" %(i ,neighborBottom))
            
        # left
        neighborLeft = i - 99
        if(i>98):
            f.write("	 (neighborLeft a%d a%d)" %(i ,neighborLeft))
        
        # right
        neighborRight = i + 99
        if(i<9702):
            f.write("	 (neighborRight a%d a%d)" %(i ,neighborRight))
            
        f.write("\r\n")
    
            
    f.write("	 (position a%d))  \r\n" %robotPosition)
    f.write("  (:goal (position a%d))  \r\n" %beerPosition)
    f.write(")")
    f.close()

   
def getRobotPosition():
    try:
        model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        
        # get position of robot
        turtlebot_coordinates = model_coordinates("turtlebot3_burger", "")                
        turtlebot_y = turtlebot_coordinates.pose.position.y
        turtlebot_x = turtlebot_coordinates.pose.position.x
        
        #print("Position of robot is [%f, %f]" %(turtlebot_x, turtlebot_y))
        return turtlebot_x, turtlebot_y
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    
    
def getTrashPosition():
    rospy.wait_for_service('gazebo/get_model_state')
    
    try:
        model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        
        # get position of beer can 1
        beer_coordinates = model_coordinates("beer", "")                
        beer_y = beer_coordinates.pose.position.y
        beer_x = beer_coordinates.pose.position.x
        print("Position of beer1 is [%f, %f]" %(beer_x, beer_y))
        
        # get position of beer can 2
        beer_coordinates = model_coordinates("beer_0", "")                
        beer_0_y = beer_coordinates.pose.position.y
        beer_0_x = beer_coordinates.pose.position.x
        print("Position of beer2 is [%f, %f]" %(beer_0_x, beer_0_y))
        
        # get position of beer can 3
        beer_coordinates = model_coordinates("beer_1", "")                
        beer_1_y = beer_coordinates.pose.position.y
        beer_1_x = beer_coordinates.pose.position.x
        
        print("Position of beer3 is [%f, %f]" %(beer_1_x, beer_1_y))
        return beer_x, beer_y, beer_0_x, beer_0_y, beer_1_x, beer_1_y
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
        
        
def gazCoordinateToPddl(x, y, mapping):
    
    # x
    smallest = 10
    for i in range(0, len(mapping)):
        diff = abs(mapping[i][0]-x)
        if diff < smallest:
            smallest = diff
            num = i
            
    #y
    smallest = 10
    for i in range(num, num+98):
        diff = abs(mapping[i][1]-y)
        if diff < smallest:
            smallest = diff
            num_y = i
            
    return num_y

def PddlToGazCoordinate(num, mapping): 
    return mapping[num]

def callPlanner():
    print("Start planning")
    os.system('rosservice call /rosplan_planner_interface/planning_server')
    print("Planning complete")
    
def readPlan():
    with open('..//planner//plan.pddl') as f:
        lines = f.readlines()
    for i in range(0, len(lines)):
        if "Time" in lines[i]:
            num = i
    out = list()     
    for i in range(num+1, len(lines)):
        line = lines[i].split("(")
        line = line[1].split(")")
        line = line[0].split(" ")
        out.append(line)   
    print("Read in planned route")
    return out

class RobotMover(object)
    """class containing all ROS nodes for moving the robot
    """
    def __init__(self):
        rospy.init_node('GoForward', anonymous=False)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(100)

    def turnRobot(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 1.0
        t=0
        while t < 170:
            self.cmd_vel.publish(move_cmd)
            t = t+1
            r.sleep()
        self.cmd_vel.publish(Twist())
        r.sleep()
        
        
    def driveRobot_x(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0
        t=0
        while t < 107:
            self.cmd_vel.publish(move_cmd)
            t = t+1
            r.sleep()
        self.cmd_vel.publish(Twist())
        r.sleep()
        
    def driveRobot_y(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0
        t=0
        while t < 52:
            self.cmd_vel.publish(move_cmd)
            t = t+1
            r.sleep()
        self.cmd_vel.publish(Twist())
        r.sleep()
        
    def setYaw(self, yaw):
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        x,y = getRobotPosition()
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        quaternion = get_quaternion_from_euler(0, 0, yaw)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        rospy.wait_for_service('gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
            set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            
       
def driveToTrash(plan, direction, mapping):
    print("Drive to trash")
    mover = RobotMover()
    for i in range(0, len(plan)):
        coordinates_from = PddlToGazCoordinate(int(plan[i][1].split('a')[1]), mapping)
        coordinates_to = PddlToGazCoordinate(int(plan[i][2].split('a')[1]), mapping)
        print("%s from [%f, %f] to [%f, %f]" %(plan[i][0], coordinates_from[0], coordinates_from[1], coordinates_to[0], coordinates_to[1]))
        coordinates_actual = getRobotPosition()
        robotPosition = gazCoordinateToPddl(coordinates_actual[0], coordinates_actual[0], mapping)
        deviation_x, deviation_y = calcDeviation(coordinates_from, coordinates_actual) 
        if (deviation_x > 0.3 or deviation_y > 0.3):
            if direction == "movestraight": 
                mover.setYaw(1.5708)
                notclear = 0
            elif direction == "moveback":
                mover.setYaw(4.71239)
                notclear = 0
            elif direction == "moveleft" :
                mover.setYaw(3.14159)
                notclear = 0
            elif direction == "moveright":
                mover.setYaw(0)
                notclear = robotPosition + 99
                
            print('Plan failed --> replan!')
            success = False
            return notclear, success, direction
        
        if plan[i][0] == direction:
            if direction == "movestraight": 
                mover.setYaw(1.5708)
                mover.driveRobot_y()
            elif direction == "moveback":
                mover.setYaw(4.71239)
                mover.driveRobot_y()
            elif direction == "moveleft" :
                mover.setYaw(3.14159)
                mover.driveRobot_x()
            elif direction == "moveright":
                mover.setYaw(0)
                mover.driveRobot_x()

        elif plan[i][0] == "moveleft":
            if direction == "movestraight":
                mover.turnRobot()
                mover.setYaw(3.14159)
                mover.driveRobot_x()
            elif direction == "moveright":
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(3.14159)
                mover.driveRobot_x()
            elif direction == "moveback":
                mover.turnRobot()
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(3.14159)
                mover.driveRobot_x()
            direction = "moveleft"
            
        elif plan[i][0] == "moveback":
            if direction == "moveleft":
                mover.turnRobot()
                mover.setYaw(4.71239)
                mover.driveRobot_y()
            elif direction == "movestraight":
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(4.71239)
                mover.driveRobot_y()
            elif direction == "moveright":
                mover.turnRobot()
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(4.71239)
                mover.driveRobot_y()
            direction = "moveback"
            
        elif plan[i][0] == "moveright":
            if direction == "moveback":
                mover.turnRobot()
                mover.setYaw(0)
                mover.driveRobot_x()
            elif direction == "moveleft":
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(0)
                mover.driveRobot_x()
            elif direction == "movestraight":
                mover.turnRobot()
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(0)
                mover.driveRobot_x()
            direction = "moveright"
            
        elif plan[i][0] == "movestraight":
            if direction == "moveright":
                mover.turnRobot()
                mover.setYaw(1.5708)
                mover.driveRobot_y()
            elif direction == "moveback":
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(1.5708)
                mover.driveRobot_y()
            elif direction == "moveleft":
                mover.turnRobot()
                mover.turnRobot()
                mover.turnRobot()
                mover.setYaw(1.5708)
                mover.driveRobot_y()
            direction = "movestraight"
    
    notclear = 0
    success = True
    return notclear, success, direction
            
def removeTrash(trash):
    print("Grasp trash")
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete(trash)


def calcDeviation(coordinates_target, coordinates_actual):
    deviation_x = abs(coordinates_target[0] - coordinates_actual[0])
    deviation_y = abs(coordinates_target[1] - coordinates_actual[1])
    return deviation_x, deviation_y
    

if __name__ == '__main__':
    try:
        mapping = createMapping()
        beer_x, beer_y, beer_0_x, beer_0_y, beer_1_x, beer_1_y = getTrashPosition()
        success = False
        direction = 'moveleft'
        notclear = 0
        # beer
        print("Start collecting beer1")
        while success != True:
            turtlebot_x, turtlebot_y = getRobotPosition()
            robotPosition = gazCoordinateToPddl(turtlebot_x, turtlebot_y, mapping)
            beerPosition = gazCoordinateToPddl(beer_x, beer_y, mapping)
            createPddl(mapping, robotPosition, beerPosition, notclear)
            callPlanner()
            plan = readPlan()
            notclear, success, direction = driveToTrash(plan, direction, mapping)
            time.sleep(1)
        removeTrash('beer')
        success = False
        
        # beer_0
        print("Start collecting beer2")
        while success != True:
            turtlebot_x, turtlebot_y = getRobotPosition()
            robotPosition = gazCoordinateToPddl(turtlebot_x, turtlebot_y, mapping)
            beerPosition = gazCoordinateToPddl(beer_0_x, beer_0_y, mapping)
            createPddl(mapping, robotPosition, beerPosition, notclear)
            callPlanner()
            plan = readPlan()
            notclear, success, direction = driveToTrash(plan, direction, mapping)
            time.sleep(1)
        removeTrash('beer_0')
        success = False
        
        # beer_1
        while success != True:
            turtlebot_x, turtlebot_y = getRobotPosition()
            robotPosition = gazCoordinateToPddl(turtlebot_x, turtlebot_y, mapping)
            beerPosition = gazCoordinateToPddl(beer_1_x, beer_1_y, mapping)
            createPddl(mapping, robotPosition, beerPosition, notclear)
            callPlanner()
            plan = readPlan()
            notclear, success, direction = driveToTrash(plan, direction, mapping)
            time.sleep(1)
        removeTrash('beer_1')
        success = False
        
        # drive to bin
        print("Drive to bin")
        while success != True:
            turtlebot_x, turtlebot_y = getRobotPosition()
            robotPosition = gazCoordinateToPddl(turtlebot_x, turtlebot_y, mapping)
            binPosition = gazCoordinateToPddl(2.5, -3.93, mapping)
            createPddl(mapping, robotPosition, binPosition, notclear)
            callPlanner()
            plan = readPlan()
            notclear, success, direction = driveToTrash(plan, direction, mapping)
            time.sleep(1)
        print("All beer cans are at collection point")
    except rospy.ROSInterruptException:
        pass
 


