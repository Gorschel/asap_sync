#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 11 15:53:10 2022

@author: lukas
"""
# imports
import os
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, DeleteModel
from std_msgs.msg import String
from rosplan_dispatch_msgs.srv import PlanningService

# create mapping of gazebo-coordinates and planner-coordinates
def createMapping():
    # x and y limits in gazebo
    x = [-5.9, 3.5]
    y = [0.3, -4.1]
    
    # size of planner-coordinates
    size = 99
    
    # create mapping
    x_range = np.linspace(x[0], x[1], size)
    y_range = np.linspace(y[0], y[1], size)
    mapping = list()
    for i in range(0, size):
        for j in range(0, size):
            mapping.append([x_range[i], y_range[j]])
    return mapping

# create problem.pddl with mapped coordinates and robotPostion(==start) & beerPosition(==goal)
def createPddl(mapping, robotPosition, beerPosition):
    f=open("..//planner//problem.pddl", "w")
    f.write("(define (problem robot)  \r\n")
    f.write("  (:domain robot)  \r\n")
    
    # objects
    f.write("  (:objects")
    
    for i in range(0, len(mapping)):
        f.write(" a%d" %i)
        
    f.write(")  \r\n")
    
    f.write("  (:init  \r\n")
    
    # neighbors
    for i in range(0, len(mapping)):
        
        # top
        neighborTop = i - 1
        if(neighborTop >= 0 and neighborTop<=(len(mapping)-1)):
            f.write("	 (neighborTop a%d a%d)" %(i, neighborTop))
            
        # bootom
        neighborBottom = i + 1
        if(neighborBottom >= 0 and neighborBottom<=(len(mapping)-1)):
            f.write("	 (neighborBottom a%d a%d)" %(i ,neighborBottom))
            
        # left
        neighborLeft = i - 99
        if(neighborLeft >= 0 and neighborLeft<=(len(mapping)-1)):
            f.write("	 (neighborLeft a%d a%d)" %(i ,neighborLeft))
        
        # right
        neighborRight = i + 99
        if(neighborRight >= 0 and neighborRight<=(len(mapping)-1)):
            f.write("	 (neighborRight a%d a%d)" %(i ,neighborRight))
            
        f.write("\r\n")
    
    f.write("	 (position a%d))  \r\n" %robotPosition)
    f.write("  (:goal (position a%d))  \r\n" %beerPosition)
    f.write(")")
    f.close()

# get postion of turtlebot3 from gazebo   
def getRobotPosition():
    try:
        model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        
        # get position of robot
        turtlebot_coordinates = model_coordinates("turtlebot3_burger", "")                
        turtlebot_y = turtlebot_coordinates.pose.position.y
        turtlebot_x = turtlebot_coordinates.pose.position.x
        
        return turtlebot_x, turtlebot_y
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
    
# get postion of the three beer cans    
def getTrashPosition():
    rospy.wait_for_service('gazebo/get_model_state')
    
    try:
        model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        
        # get position of beer can 1
        beer_coordinates = model_coordinates("beer", "")                
        beer_y = beer_coordinates.pose.position.y
        beer_x = beer_coordinates.pose.position.x
        
        # get position of beer can 2
        beer_coordinates = model_coordinates("beer_0", "")                
        beer_0_y = beer_coordinates.pose.position.y
        beer_0_x = beer_coordinates.pose.position.x
        
        # get position of beer can 3
        beer_coordinates = model_coordinates("beer_1", "")                
        beer_1_y = beer_coordinates.pose.position.y
        beer_1_x = beer_coordinates.pose.position.x
        
        return beer_x, beer_y, beer_0_x, beer_0_y, beer_1_x, beer_1_y
        
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
        

# convert gazebo-coordinates in planner-coordinates        
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

# convert planner-coordinates in gazebo-coordinates
def PddlToGazCoordinate(num, mapping): 
    return mapping[num]

# call planner
def callPlanner():
    os.system('rosservice call /rosplan_planner_interface/planning_server')

# read plan    
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
    return out

# turn robot 45° in left direction
def turnRobot():
    rospy.init_node('GoForward', anonymous=False)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    r = rospy.Rate(10)
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0.3
    t=0
    while t < 55:
        cmd_vel.publish(move_cmd)
        t = t+1
        r.sleep()
    cmd_vel.publish(Twist())
    r.sleep()
    
# drive one box in x-direction    
def driveRobot_x():
    rospy.init_node('GoForward', anonymous=False)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    r = rospy.Rate(10)
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = 0
    t=0
    while t < 10:
        cmd_vel.publish(move_cmd)
        t = t+1
        r.sleep()
    cmd_vel.publish(Twist())
    r.sleep()

# drive one box in y-direction    
def driveRobot_y():
    rospy.init_node('GoForward', anonymous=False)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    r = rospy.Rate(10)
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = 0
    t=0
    while t < 5:
        cmd_vel.publish(move_cmd)
        t = t+1
        r.sleep()
    cmd_vel.publish(Twist())
    r.sleep()

# drive all the steps planned by the planner    
def driveToTrash(plan):
    direction = "moveleft"
    for i in range(0, len(plan)):
        if plan[i][0] == direction:
            if direction == "movestraight" or direction == "moveback":
                driveRobot_y()
            elif direction == "moveleft" or direction == "moveright":
                driveRobot_x()

        elif plan[i][0] == "moveleft":
            if direction == "movestraight":
                turnRobot()
                driveRobot_x()
            elif direction == "moveright":
                turnRobot()
                turnRobot()
                driveRobot_x()
            elif direction == "moveback":
                turnRobot()
                turnRobot()
                turnRobot()
                driveRobot_x()
            direction = "moveleft"
            
        elif plan[i][0] == "moveback":
            if direction == "moveleft":
                turnRobot()
                driveRobot_y()
            elif direction == "movestraight":
                turnRobot()
                turnRobot()
                driveRobot_y()
            elif direction == "moveright":
                turnRobot()
                turnRobot()
                turnRobot()
                driveRobot_y()
            direction = "moveback"
            
        elif plan[i][0] == "moveright":
            if direction == "moveback":
                turnRobot()
                driveRobot_x()
            elif direction == "moveleft":
                turnRobot()
                turnRobot()
                driveRobot_x()
            elif direction == "movestraight":
                turnRobot()
                turnRobot()
                turnRobot()
                driveRobot_x()
            direction = "moveright"
            
        elif plan[i][0] == "movestraight":
            if direction == "moveright":
                turnRobot()
                driveRobot_y()
            elif direction == "moveback":
                turnRobot()
                turnRobot()
                driveRobot_y()
            elif direction == "moveleft":
                turnRobot()
                turnRobot()
                turnRobot()
                driveRobot_y()
            direction = "movestraight"
        
# remove trash from gazebo model            
def removeTrash(trash):
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete(trash)

# main
if __name__ == '__main__':
    try:
        mapping = createMapping()
        turtlebot_x, turtlebot_y = getRobotPosition()
        robotPosition = gazCoordinateToPddl(turtlebot_x, turtlebot_y, mapping)
        beer_x, beer_y, beer_0_x, beer_0_y, beer_1_x, beer_1_y = getTrashPosition()
        beerPosition = gazCoordinateToPddl(beer_x, beer_y, mapping)
        createPddl(mapping, robotPosition, beerPosition)
        callPlanner()
        plan = readPlan()
        driveToTrash(plan)
        removeTrash('beer')
    except rospy.ROSInterruptException:
        pass
 


