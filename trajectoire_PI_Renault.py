#! /usr/bin/env python
# -*- coding: utf-8 -*-

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
from math import *

#partie force

from geometry_msgs.msg import WrenchStamped, Twist,TransformStamped, Transform
from std_msgs.msg import Float32
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_msgs.msg import TFMessage


import socket
import time 
import threading

import tf
import tf2_ros
import tf2_geometry_msgs

import threading
import time
import keyboard
from six.moves import input as ip



# Variables UDP
UDP_IP = "192.168.1.112"
UDP_Port = 32200
UDP_Positions = []

# Création du socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# Variables globales
data_force = Float32()
erreur_force =0

pos_angle = []
position = [0.3, 0.13, 0.4]
# Angles d'Euler
Alpha = 0
Beta = 0
Gamma = 1.57

pos_prec = None

or_prec = None

 #modif : pour appliquer un filtre

#fin de la modif 

# Envoi des données de position en UDP
def send_positions(position_x, position_y, position_z,rx,ry,rz):
    # Formatage des positions en une chaîne de caractères
    ''''
    position_x += 

    '''
   
   
    message = "{};{};{};{};{};{}".format(position_x, position_y, position_z,rx,ry,rz)
    #print (message)
    # Envoi du message au destinataire
    #print(position_x, position_y, position_z)
    sock.sendto(message.encode(), (UDP_IP, UDP_Port))
    #print(message)



# Position initiale 
def init_pos(group, cartesian, vel_f, acc_f, step, allow_collision):
    
    # Création d'une liste de points
    waypoints = []
    # Création d'une variable point
    w_pose_target  = geometry_msgs.msg.Pose()

    # Récupère la position actuelle du robot
    w_pose_target = group.get_current_pose().pose
    waypoints.append(copy.deepcopy(w_pose_target))
    
    # Position initiale du robot
    w_pose_target.position.x = cartesian[0]
    w_pose_target.position.y = cartesian[1]
    w_pose_target.position.z = cartesian[2]

    # Orientation initiale du robot
    w_pose_target.orientation.w = cartesian[3]
    w_pose_target.orientation.x = cartesian[4]
    w_pose_target.orientation.y = cartesian[5]
    w_pose_target.orientation.z = cartesian[6]

    waypoints.append(copy.deepcopy(w_pose_target))

    # Calcule le chemin cartésien à partir des points de la liste waypoints
    (plan, fraction) = group.compute_cartesian_path(waypoints, step, 0.0, avoid_collisions = allow_collision, path_constraints = None)

    # Vérifie si le chemin à bien été planifiée
    if 1-fraction < 0.01:
        rospy.loginfo('Path computed successfully.')
    else:
        rospy.loginfo('Path planning failed')

    rospy.loginfo('Fraction: %f' % fraction)

    # Obtenir la trajectoire planifiée
    plan = group.retime_trajectory(group.get_current_state(), plan, vel_f, acc_f)

    return plan

# Trajectoire Accoudoir
def Calcul_Trajectoire(group, cartesian, O_x, O_y, mode, L, R, theta, nbpoints, vel_f, acc_f, step, allow_collision,D):
    """
    Description:
       A simple example of Cartesian control of several points (trajectory planning from specified initial Cartesian positions). 
       Other parameters are calculated from the function (example of a triangle path).

    Args:
        (1) group [MoveGroup Class]: Client class for the MoveGroup action.
        (2) cartesian [Float Array]: Array of robot cartesian positions (x, y, z, Quaternion(w..z)).
        (3 - 4) offset [Float Array]: Offset positions (x, y).
        (5) rayon
        (6) angle de rotation
        (7) nombre de points
        (8 - 9) vel_f, acc_f [Float]: Velocity/Acceleration scaling factor.
        (10) step [INT]: Number of steps (Cartesian calculation).
        (11) allow_collision [Bool]: Allow collision object or not.

    Returns:
        (1) param{1} [INT]: The representation of a motion plan (time, start state, trajectory, etc).
    """

    # Création d'une liste de points
    waypoints = []
    # Création d'une variable point
    w_pose_target  = geometry_msgs.msg.Pose()
    
    # Position initiale du robot
    w_pose_target.position.x = cartesian[0]
    w_pose_target.position.y = cartesian[1]
    w_pose_target.position.z = cartesian[2]

    # Orientation initiale du robot
    w_pose_target.orientation.w = cartesian[3]
    w_pose_target.orientation.x = cartesian[4]
    w_pose_target.orientation.y = cartesian[5]
    w_pose_target.orientation.z = cartesian[6]

    waypoints.append(copy.deepcopy(w_pose_target))

    if mode == 'Accoudoir':
        #Trajectoire rectiligne
        w_pose_target.position.x = cartesian[0]
        w_pose_target.position.y = cartesian[1] + O_y + L 
        w_pose_target.position.z = cartesian[2]

        # Initialise l'orientation du robot à partir des angles d'Euler
        orientation = tf.transformations.quaternion_from_euler(0,0,0)    # effecteur vers le haut
        
        # Orientation du robot
        w_pose_target.orientation.w = orientation[0]
        w_pose_target.orientation.x = orientation[1]
        w_pose_target.orientation.y = orientation[2]
        w_pose_target.orientation.z = orientation[3]
        
        waypoints.append(copy.deepcopy(w_pose_target))

    elif mode == 'Portière':
        for i in range(2,nbpoints) :
            #Trajectoire circulaire
            w_pose_target.position.x = cartesian[0] + R*(sin(theta * i/ nbpoints)) #+D*(1-(cos(theta * i/ nbpoints)))
            w_pose_target.position.y = cartesian[1] - R*(1-cos(theta * i/ nbpoints)) #-D*(sin(theta * i/ nbpoints))
            w_pose_target.position.z = cartesian[2] 

            # Initialise l'orientation du robot à partir des angles d'Euler
            orientation = tf.transformations.quaternion_from_euler(-1.57+theta * i/ nbpoints,0,-1.57)
            
            # Orientation du robot
            w_pose_target.orientation.w = orientation[0]
            w_pose_target.orientation.x = orientation[1]
            w_pose_target.orientation.y = orientation[2]
            w_pose_target.orientation.z = orientation[3]

        waypoints.append(copy.deepcopy(w_pose_target))


    # Calcule le chemin cartésien à partir des points de la liste waypoints
    (plan, fraction) = group.compute_cartesian_path(waypoints, step, 0.0, avoid_collisions = allow_collision, path_constraints = None)

    # Vérifie si le chemin à bien été planifiée
    if 1-fraction < 0.01:
        rospy.loginfo('Path computed successfully.')
    else:
        rospy.loginfo('Path planning failed')

    rospy.loginfo('Fraction: %f' % fraction)

    # Obtenir la trajectoire planifiée
    plan = group.retime_trajectory(group.get_current_state(), plan, vel_f, acc_f)

    return plan


# Fonction de rappel pour l'accoudoir sur le topic "wrench" (force)
def callbackAccoudoir(data):
    global data_force   # on précise le caractère global de la variable pour la modifier

    # Vecteur des composantes linéaires (x,y,z)
    vectF = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
    data_force = -vectF[1]


# Fonction de rappel  pour la portière sur le topic "wrench" (force)
def callbackPortiere(data):
    global data_force
    global erreur_force
     # on précise le caractère global de la variable pour la modifier

    # Vecteur des composantes linéaires (x,y,z)
    vectF = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
   
    data_force = vectF[2] - erreur_force

# Fonction de rappel sur le topic 'joint_states'

def callbackpos(data):
    global pos_prec
    global or_prec
   
    position = data.transforms[0].transform.translation
    orientation = data.transforms[0].transform.rotation
    angle = tf.transformations.euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
    #print(angle[2]*180/pi)
    send_positions(position.x,position.y,position.z,angle[0]*180/pi,angle[1]*180/pi,angle[2]*180/pi)
    '''
    errpos = 0.1
    errrot = 0.1
    
    if ((pos_prec is not None) and (or_prec is not None)) :
         if (abs(pos_prec.x-position.x)<errpos):
            if ((abs(or_prec[0]-angle[0])<errrot )and (abs(or_prec[1]-angle[1])<errrot )and( abs(or_prec[2]-angle[2])<errrot)):
    
    pos_prec = position
    or_prec = angle
    '''

            

    #modif
    #fin de modif


def callbackArti(data):
    global pos_angle
    pos_angle.append(data.position)
    global UDP_Positions   # on précise le caractère global de la variable pour la modifier
    w_pose_target  = geometry_msgs.msg.Pose()
    UDP_Positions.append(w_pose_target)

# Fonction pour publier les angles au robot
def publi_angle(vect_angle,pub):
    msg=JointTrajectory()
    vect_pos = JointTrajectoryPoint()

    # Nom des articulations du robot
    msg.joint_names = ["elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"]

    # On défini toute les composantes du message
    vect_pos.velocities=[0,0,0,0,0,0]
    vect_pos.accelerations= [0,0,0,0,0,0]
    vect_pos.effort = [0,0,0,0,0,0]
    vect_pos.time_from_start = rospy.Duration(0.5)
    vect_pos.positions = vect_angle

    # On ajoute le vecteur au message
    msg.points.append(vect_pos)
    pub.publish(msg)


def getMode():
    choix  = 0
    global position
    global Alpha
    global Beta
    global Gamma
    while (choix != 1 or choix != 2) :
        rospy.loginfo("\nVeuillez entrer le numéro correspondant au type de mouvement du robot :\n(1) Accoudoir\n(2) Portière")
        choix  = input('Choix : ')

        if choix == 1:
            position = [0.3, 0.13, 0.4]
            Alpha = 0
            Beta = 0
            Gamma = 0

            # Récupération de l'effort au niveau de l'effecteur
            rospy.Subscriber("wrench", WrenchStamped, callbackAccoudoir)
            
            return 'Accoudoir'

        elif choix == 2:
            position = [0.21, 0.325, 0.4]
            Alpha = -1.57
            Beta = 0
            Gamma = -1.57

            # Récupération de l'effort au niveau de l'effecteur
            rospy.Subscriber("wrench", WrenchStamped, callbackPortiere)

            return 'Portière'

        else :
            rospy.loginfo("Choix invalide. Réessayez")
   

def main():
    rospy.loginfo("Démarrage")


 
    # Initialisation de Moveit
    group = moveit_commander.MoveGroupCommander('manipulator')
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    # Reset robot positions (go to home position)
    #rospy.wait_for_service('/reset_robot')
    #reset_robot  = rospy.ServiceProxy('/reset_robot', Trigger)
    #reset_robot.call()

    # Initialize the current position of the robot
    w_pose_initial = rospy.wait_for_message('/current_tcp_pose', geometry_msgs.msg.PoseStamped, timeout=None)
    #position       = [w_pose_initial.pose.position.x, w_pose_initial.pose.position.y, w_pose_initial.pose.position.z]
    #orientation    = [w_pose_initial.pose.orientation.w, w_pose_initial.pose.orientation.x, w_pose_initial.pose.orientation.y, w_pose_initial.pose.orientation.z]
    
    #Position (x,y,z) de la poignée 
    #position = [0.2, -0.2, 0.4]    // Autre position accoudoir

    # Initialisation de data_force à 0
    global data_force
    global erreur_force
    erreur_force =0
    data_force = 0

    # Choix de type de Trajectoire
    mode = getMode()
    
    if (mode=='Accoudoir'):
        position = [0.3, 0.13, 0.4]
    if (mode=='Portière'):
        position = [0.0, 0.35, 0.4]
    

    # Orientation de la poignée (quaternions)
    orientation  = tf.transformations.quaternion_from_euler(Alpha,Beta,Gamma)
    
    # Paramètres de planification (vitesse et accélération)
    vel_scaling_f = 0.01
    acc_scaling_f = 1.0
    group.set_max_velocity_scaling_factor(vel_scaling_f)
    group.set_max_acceleration_scaling_factor(acc_scaling_f)

    # Planificateur -> OMPL (par défaut) ou BiTRRTkConfigDefault
    group.set_planner_id('OMPL')

    # Caractéristiques de la trajectoire
    O_x = -0.0
    O_y = -0.0
    R = 0.93
    theta = +np.pi*30./180
    nbpoints = 60
    L = 0.2
    D = 0.09

    #Création de la trajectoire initiale
    init_plan = init_pos(group, 
        [position[0], position[1], position[2], 
        orientation[0], orientation[1], orientation[2], orientation[3]], 
        0.1, acc_scaling_f, 0.01, True
        )
    

    # Montrer la trajectoire
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(init_plan)

    #Exécution de la position initiale
    rospy.loginfo("Position initiale")
    group.execute(init_plan, wait=True)
    
    # Reset
    group.stop()
    group.clear_path_constraints()
    group.clear_pose_targets()

    rospy.sleep(2)

    # Générer la trajectoire
    plan = Calcul_Trajectoire(group, 
        [position[0], position[1], position[2], 
        orientation[0], orientation[1], orientation[2], orientation[3]], O_x, O_y,
        mode, L, R, theta, nbpoints,
        vel_scaling_f, acc_scaling_f, 0.01, True,D
        )

    rospy.loginfo('Intermediate points on the robot trajectory: %f' % len(plan.joint_trajectory.points))
    
    # Montrer la trajectoire
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    #Récupération des positions articulaires de la trajectoire
    global pos_angle
    recup_posi = rospy.Subscriber("joint_states", JointState, callbackArti)    # Commence l'enregistrement
    group.execute(plan, wait=True)  # Exécute la trajectoire avec MoveIt
    traj_final = copy.deepcopy(pos_angle)  # Arrête l'enregistrement
    
    # Création d'un publisher pour publier les trajectoires articulaires directement au robot
    pub = rospy.Publisher('scaled_pos_joint_traj_controller/command', JointTrajectory , queue_size=1)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("tf", TFMessage, callbackpos) # pour recevoir la position actuelle cartesienne.
    #rospy.Rate(20)
    # Position actuelle du robot (on la place initialement au milieu de la trajectoire)
    posact = len(traj_final)/2
    for i in range (len(traj_final)-1,posact,-1):
        publi_angle(traj_final[i],pub)

    rospy.loginfo("Vous pouvez faire bouger le robot...")
    # Boucle principale
    if (mode=='Accoudoir'): # En plus d'avoir une vitesse proportionelle à la force, il y a un retour de force semblable à des crans.
        
        while not rospy.is_shutdown():
            C=7 # Le nombre de crans le long de la trajectoire
            if (posact/float(len(traj_final)/C)%1)<=0.5: #position actuelle dans la liste%nombre de position par cran.
                
                if ((posact+int(data_force/3)>150) and (posact+int(data_force/3)+1<len(traj_final)-150)): 
                    publi_angle(traj_final[posact+int(data_force/3)+1],pub)
                    posact+=int(data_force/3)+1
            if (posact/float(len(traj_final)/C)%1)==0.5:
                if ((posact+int(data_force/3)>150) and (posact+int(data_force/3)<len(traj_final)-150)):
                    publi_angle(traj_final[posact+int(data_force/3)],pub)
                    posact+=int(data_force/3)

            if (posact/float(len(traj_final)/C)%1)>0.5: 
                if ((posact+int(data_force/3)-1>150) and (posact+int(data_force/3)-1<len(traj_final)-150)):
                    publi_angle(traj_final[posact+int(data_force/3)-1],pub)
                    posact+=int(data_force/3)-1
            #print(data_force,posact,len(traj_final),posact/float(len(traj_final)/C)%1)
            #send_positions(UDP_Positions[-1].position.x, UDP_Positions[-1].position.y, UDP_Positions[-1].position.z)


    else : # Ici la vitesse est proportionnelle à la force et on est en mode portière
        '''
        while not rospy.is_shutdown():
            if(data_force > 3 and posact<len(traj_final)-int(abs(data_force/3))) : 
                publi_angle(traj_final[posact+int(abs(data_force/3))],pub)
                posact+=int(abs(data_force/3))
                """
                print("erreur :")
                print(erreur_force)
                print("data_force :")
                print(data_force)
                """
                #rospy.sleep(1)
            if(data_force < -3 and posact>int(abs(data_force/3))):
                publi_angle(traj_final[posact-int(abs(data_force/3))],pub)
                posact-=int(abs(data_force/3))
                """
                print("erreur :")
                print(erreur_force)
                print("data_force :")
                print(data_force)
            
                """
            
           
            if T.touche == 'r':
                
                print("changement thread activé : erreur :")
                print(erreur_force)
                print("data_force :")
                print(data_force)
                print(T.touche)
                erreur_force = data_force 
        '''
        rate1 = rospy.Rate(150)
        vitesse=0.0
        while not rospy.is_shutdown():
            val =1.5
            if(abs(vitesse)>150):
                print("trop rapide",data_force,posact,len(traj_final),vitesse)
                vitesse = 149
            else:
                if T.touche == 'r':
                
                    T.touche = 'e'
                    print("changement thread activé : erreur :")
                    print(erreur_force)
                    print("data_force :")
                    print(data_force)
                    print(T.touche)
                    erreur_force = data_force + erreur_force
                
                if (vitesse>0.0):
                    if (posact+int(vitesse+data_force/val -10.0/val)<len(traj_final)-500):
                        vitesse += data_force/val -15.0/val -(vitesse*vitesse)/1000
                        publi_angle(traj_final[posact+int(vitesse)],pub)
                        posact+=int(vitesse)
                        print("1")
                
                if (vitesse<0.0):
                    if (posact+int(vitesse+data_force/val +10.0/val)>500):
                        vitesse += data_force/val +15.0/val +(vitesse*vitesse)/1000
                        publi_angle(traj_final[posact+int(vitesse)],pub)
                        posact += int(vitesse)
                        print("2")
                    
                if ((vitesse==0.0) and (posact+int(vitesse+data_force/val +10.0/val)<len(traj_final)-500)):
                    vitesse += data_force/val
                    publi_angle(traj_final[posact+int(vitesse)],pub)
                    posact += int(vitesse)
                    print("3")
                print(data_force,posact,len(traj_final),vitesse)
                rate1.sleep()
        
            


    # Reset
    group.stop()
    group.clear_path_constraints()
    group.clear_pose_targets()

class Threadtouche (threading.Thread):
    def __init__(self):      # jusqua = donnée supplémentaire
        threading.Thread.__init__(self)  # ne pas oublier cette ligne
        # (appel au constructeur de la classe mère)
        self.touche = 'a'   # donnée supplémentaire ajoutée à la classe

    def run(self):
        while (not rospy.is_shutdown()):
            self.touche = 'E'
            self.touche = ip("press space to reset data_force").strip()
            rospy.sleep(1)
        


if __name__ == '__main__':
    # Initialisation du node du robot
    T = Threadtouche()
    T.start()
    rospy.init_node('robot_init', anonymous=True)
    sys.exit(main())