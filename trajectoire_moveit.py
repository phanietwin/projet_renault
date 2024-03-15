#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
## =========================================================================== ## 
MIT License
Copyright (c) 2020 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
## =========================================================================== ## 
Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak
File Name: test.py
## =========================================================================== ## 
"""

# System (Default Lib.)
import sys

# Python client library for ROS
import rospy

# Package offers wrappers for the functionality provided in MoveIt
import moveit_commander

# Data types (messages, services)
import geometry_msgs
from std_srvs.srv import Trigger
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Float64

# Package creates links between the target and the object
import copy

# Numpy (Array computing Lib.)
import numpy as np
import math as m

#partie force

from geometry_msgs.msg import WrenchStamped, Twist
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import time 
import threading
import test_norme_force2

import tf

data_force = Float32()
data_force = 0
pos_angle = []



#a bouger mais c'est ci mtn parce que je sias pas ou le mettre encore
#la c'est mon publisher, il put la merde, je dois rectifier ça.

def publi_angle(vect_angle,pub,data_force):
    
    msg=JointTrajectory()
    point1 = JointTrajectoryPoint()

   

    msg.joint_names = ["elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"]

    point1.velocities=[0,0,0,0,0,0]
    point1.accelerations= [0,0,0,0,0,0]
    point1.effort = [0,0,0,0,0,0]
    point1.time_from_start = rospy.Duration(0.5)
    #point1.positions = vect_angle
    point1.positions= vect_angle
    msg.points.append(point1)
    print("bonjrrrrr")
    print(vect_angle)
    
    print(type(vect_angle))

    print("hello msg")
    print(msg) 
    pub.publish(msg)
    print("l'erreur est plus basse")

def eulerZYX_to_matrix(alpha,beta,gamma):

    R = np.identity(4)
    R = np.array([[m.cos(alpha)*m.cos(beta), -m.sin(alpha)*m.cos(gamma)+m.cos(alpha)*m.sin(beta)*m.sin(gamma), m.sin(alpha)*m.sin(gamma)+m.cos(alpha)*m.sin(beta)*m.cos(gamma), 0],
                  [m.sin(alpha)*m.cos(beta), m.cos(alpha)*m.cos(gamma)+m.sin(alpha)*m.sin(beta)*m.sin(gamma), -m.cos(alpha)*m.sin(gamma)+m.sin(alpha)*m.sin(beta)*m.cos(gamma),0 ],
                  [     -m.sin(beta),                        m.cos(beta)*m.sin(gamma),                                m.cos(beta)*m.cos(gamma)                  ,0 ],
                  [0,0,0,1]])
    
    return R


#fin de la partie force 
def init_pos(group, cartesian, nbpoints, vel_f, acc_f, step, allow_collision):
    

    waypoints = []
    w_pose_target  = geometry_msgs.msg.Pose()

    #Récupère le position initiale du robot
    w_pose_target = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(w_pose_target))
    
    # Initialize the robot's position on the entire trajectory
    w_pose_target.position.x = cartesian[0]
    w_pose_target.position.y = cartesian[1]
    w_pose_target.position.z = cartesian[2]

    # Initialize the robot's orientation on the entire trajectory
    w_pose_target.orientation.w = cartesian[3]
    w_pose_target.orientation.x = cartesian[4]
    w_pose_target.orientation.y = cartesian[5]
    w_pose_target.orientation.z = cartesian[6]

    waypoints.append(copy.deepcopy(w_pose_target))

    # Calculate the Cartesian path from waipoints 1 - 3
    (plan, fraction) = group.compute_cartesian_path(waypoints, step, 0.0, avoid_collisions = allow_collision, path_constraints = None)

    if 1-fraction < 0.01:
        rospy.loginfo('Path computed successfully.')
    else:
        rospy.loginfo('Path planning failed')

    rospy.loginfo('Fraction: %f' % fraction)

    # Get the planned trajectory (Re-Time TCP/Cartesian Control -> multiple points)
    plan = group.retime_trajectory(group.get_current_state(), plan, vel_f, acc_f)

    return plan

def cartesian_trajectory_2(group, cartesian, O_x, O_y, R, alpha, nbpoints, vel_f, acc_f, step, allow_collision):
    """
    Description:
       A simple example of Cartesian control of several points (trajectory planning from specified initial Cartesian positions). 
       Other parameters are calculated from the function (example of a triangle path).

        More information about MoveIt at:
        https://moveit.ros.org/ print(data_force)
    Args:
        (1) group [MoveGroup Class]: Client class for the MoveGroup action.
        (2) cartesian [Float Array]: Array of robot cartesian positions (x, y, z, Quaternion(w..z)).
        (3) offset [Float Array]: Offset positions (x, y, z).
        (4 - 5) vel_f, acc_f [Float]: Velocity/Acceleration scaling factor.
        (6) step [INT]: Number of steps (Cartesian calculation).
        (7) allow_collision [Bool]: Allow collision object or not.

    Returns:
        (1) param{1} [INT]: The representation of a motion plan (time, start state, trajectory, etc).
    """

    waypoints = []
    w_pose_target  = geometry_msgs.msg.Pose()
    
    #R = eulerZYX_to_matrix(orientation[0], orientation[1], orientation[2])
    #quaternion = tf.transformations.quaternion_from_matrix(R)
    # Initialize the robot's position on the entire trajectory
    w_pose_target.position.x = cartesian[0]
    w_pose_target.position.y = cartesian[1]
    w_pose_target.position.z = cartesian[2]
    

    # Initialize the robot's orientation on the entire trajectory
    w_pose_target.orientation.w = cartesian[3]
    w_pose_target.orientation.x = cartesian[4]
    w_pose_target.orientation.y = cartesian[5]
    w_pose_target.orientation.z = cartesian[6]
    #waypoints.append(copy.deepcopy(w_pose_target))
    
    """
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    #type(pose) = geometry_msgs.msg.Pose
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    """


    for i in range(nbpoints) :
        print(i)
        #Trajectoire circulaire
        w_pose_target.position.x = cartesian[0] + O_x - R*(1-m.cos(alpha * i/ nbpoints))
        w_pose_target.position.y = cartesian[1] + O_y + R*(m.sin(alpha * i/ nbpoints))

        #Trajectoire rectiligne
        #w_pose_target.position.x = cartesian[0]
        #w_pose_target.position.y = cartesian[1] + O_y + R* i/nbpoints

        w_pose_target.position.z = cartesian[2]

        # Initialize the robot's orientation on the entire trajectory
        #R = eulerZYX_to_matrix(0, 0, alpha *i/nbpoints)
        orientation    = tf.transformations.quaternion_from_euler(-alpha * i/ nbpoints,0,1.57)
        print(i*alpha/nbpoints)
        w_pose_target.orientation.w = orientation[0]
        w_pose_target.orientation.x = orientation[1]
        w_pose_target.orientation.y = orientation[2]
        w_pose_target.orientation.z = orientation[3]
        

        waypoints.append(copy.deepcopy(w_pose_target))
    
    # Waipoint
    #w_pose_target.position.x = cartesian[0]
    #w_pose_target.position.y = cartesian[1]
    #w_pose_target.position.z = cartesian[2]
    #waypoints.append(copy.deepcopy(w_pose_target))

    # Calculate the Cartesian path from waipoints 1 - 3
    (plan, fraction) = group.compute_cartesian_path(waypoints, step, 0.0, avoid_collisions = allow_collision, path_constraints = None)

    if 1-fraction < 0.01:
        rospy.loginfo('Path computed successfully.')
    else:
        rospy.loginfo('Path planning failed')

    rospy.loginfo('Fraction: %f' % fraction)

    # Get the planned trajectory (Re-Time TCP/Cartesian Control -> multiple points)
    plan = group.retime_trajectory(group.get_current_state(), plan, vel_f, acc_f)

    return plan


def callback(data):
    global data_force
    vectF = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
    data_force = vectF[2]
    #data_force = np.linalg.norm(vectF)
    #rospy.loginfo(rospy.get_caller_id() + " I heard %f", data.data)
    #data_force = data.data

def callback2(data):
    global pos_angle
    pos_angle.append(data.position)
    


def main():
    rospy.Subscriber("wrench", WrenchStamped, callback)

    
    # Visible -> on/off in the RVIZ environment
    #visible_object = True
    #rospy.set_param('object_visible', visible_object)
    print("hello")

    # Moveit Initialization
    group = moveit_commander.MoveGroupCommander('manipulator')
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    # Reset robot positions (go to home position)
    #rospy.wait_for_service('/reset_robot')
    #reset_robot  = rospy.ServiceProxy('/reset_robot', Trigger)
    #reset_robot.call()

    rospy.sleep(0.5)

    # Initialize the current position of the robot
    w_pose_initial = rospy.wait_for_message('/current_tcp_pose', geometry_msgs.msg.PoseStamped, timeout=None)
    #position       = [w_pose_initial.pose.position.x, w_pose_initial.pose.position.y, w_pose_initial.pose.position.z]
    #orientation    = [w_pose_initial.pose.orientation.w, w_pose_initial.pose.orientation.x, w_pose_initial.pose.orientation.y, w_pose_initial.pose.orientation.z]
    position       = [0.3, 0.13, 0.4]
    orientationeuler = [0,0,1.57]
    
    orientation    = tf.transformations.quaternion_from_euler(orientationeuler[0],orientationeuler[1],orientationeuler[2])
    
    # Set the parameters of the Object (display -> environment)
    #rospy.set_param('object_pos', position)

    # Setting parameters for planning
    vel_scaling_f = 1.0
    acc_scaling_f = 1.0
    group.set_max_velocity_scaling_factor(vel_scaling_f)
    group.set_max_acceleration_scaling_factor(acc_scaling_f)
    # Planner -> OMPL (Default) or BiTRRTkConfigDefault
    group.set_planner_id('OMPL')

    # Joint, Cartesian_1, Cartesian_2 or None
    mode = 'Cartesian_2'

    # caracteristiques de la trajectoire
    O_x = -0.0
    O_y = -0.0
    R = 0.3
    alpha = np.pi*90./180
    nbpoints = 60

    #Création de la trajectoire initiale
    init_plan = init_pos(group, 
        [position[0], position[1], position[2], 
        orientation[0], orientation[1], orientation[2], orientation[3]], 
        nbpoints,
        vel_scaling_f, acc_scaling_f, 0.01, True
        )
    
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(init_plan)

    print("Position initiale")
    #Exécution de la position initiale
    group.execute(init_plan, wait=True)
    
    # Reset
    group.stop()
    group.clear_path_constraints()
    group.clear_pose_targets()

    rospy.sleep(2)

    # Generate Cartesian Trajectory (2)
    plan = cartesian_trajectory_2(group, 
        [position[0], position[1], position[2], 
        orientation[0], orientation[1], orientation[2], orientation[3]], O_x, O_y,
        R, alpha, nbpoints,
        vel_scaling_f, acc_scaling_f, 0.01, True
        )
    rospy.loginfo('Intermediate points on the robot trajectory: %f' % len(plan.joint_trajectory.points))
    # Show trajectory
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)


    #Récupération des positions articulaires de la trajectoire
    global pos_angle
    #rospy.Subscriber("joints_state", 10, callback2)
    recup_posi=rospy.Subscriber("joint_states", JointState, callback2)
    group.execute(plan, wait=True)
    traj=pos_angle
    traj_final = []
    
    #kaou mettre un time.sleep si la ligne si lit tout de suite. On pourra deviner le temps parce que c'est nous
    #qui donnons une vitesse à la traj, le calcule doit pas être foufou.
    #pour ca mettre un print apres la traj et voir quand elle apparait.
    
    pub=rospy.Publisher('scaled_pos_joint_traj_controller/command', JointTrajectory , queue_size=1
                        )
  
    #recup_posi.shutdown()



    
    for i in range(0,len(traj)):
        traj_final.append(traj[i])
    Mid = len(traj_final)/2
    print(Mid)
    rospy.sleep(1)
    publi_angle(traj_final[Mid],pub,5)
    posact=Mid

    while not rospy.is_shutdown():
        if(data_force > 3 and posact<len(traj_final)-1) : 
            publi_angle(traj_final[posact+1],pub,data_force)
            posact+=1
            #rospy.sleep(1)
        if(data_force < -3 and posact>1):
            publi_angle(traj_final[posact-1],pub,data_force)
            posact-=1
            #rospy.sleep(1)
        print(data_force,posact,len(traj_final),Mid)
    

    
    

    # Reset
    group.stop()
    group.clear_path_constraints()
    group.clear_pose_targets()


if __name__ == '__main__':
    # Robot Node initialization
    
    rospy.init_node('robot_init', anonymous=True)
    

    sys.exit(main())
