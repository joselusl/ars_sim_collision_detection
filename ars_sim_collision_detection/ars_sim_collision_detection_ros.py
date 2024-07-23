import numpy as np
from numpy import *

import os


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

import visualization_msgs.msg
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers



class ArsSimCollisionDetectionRos(Node):
  #######

  # Robot size radius
  robot_size_radius = 0.3


  # Robot pose subscriber
  robot_pose_sub = None

  # Obstacles static sub
  obstacles_static_sub = None
  
  # Obstacles dynamic sub
  obstacles_dynamic_sub = None

  # Robot collision pub
  robot_collision_pub = None

  
  # Robot Pose
  flag_robot_pose_set = False
  robot_posi = None
  robot_atti_quat_simp = None

  # Obstacles static
  obstacles_static_msg = None

  # Obstacles dynamic
  obstacles_dynamic_msg = None
  

  #########

  def __init__(self, node_name='ars_sim_collision_detection_node'):
    # Init ROS
    super().__init__(node_name)

    # Robot size radius
    self.robot_size_radius = 0.3

    #
    self.flag_robot_pose_set = False
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()

    #
    self.obstacles_static_msg = MarkerArray()

    #
    self.obstacles_dynamic_msg = MarkerArray()

    #
    self.__init()

    # end
    return


  def __init(self):
    
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_sim_collision_detection')
      print(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      print("Package not found")
    

    #### READING PARAMETERS ###
    
    ###


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.robotPoseCallback, qos_profile=10)
    
    # 
    self.obstacles_static_sub = self.create_subscription(MarkerArray, 'obstacles_static', self.obstaclesStaticCallback, qos_profile=10)
    #
    self.obstacles_dynamic_sub = self.create_subscription(MarkerArray, 'obstacles_dynamic', self.obstaclesDynamicCallback, qos_profile=10)


    # Publishers

    # 
    self.robot_collision_pub = self.create_publisher(Bool, 'robot_collision', qos_profile=10)


    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def robotPoseCallback(self, robot_pose_msg):

    #
    self.flag_robot_pose_set = True

    # Position
    self.robot_posi[0] = robot_pose_msg.pose.position.x
    self.robot_posi[1] = robot_pose_msg.pose.position.y
    self.robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.checkCollisionRobotObstacles()
    
    #
    return



  def obstaclesStaticCallback(self, obstacles_static_msg):

    self.obstacles_static_msg = obstacles_static_msg

    #
    return



  def obstaclesDynamicCallback(self, obstacles_dynamic_msg):

    self.obstacles_dynamic_msg = obstacles_dynamic_msg

    #
    return


  def checkCollisionRobotObstacles(self):

    flag_collision_detected = False


    # Check
    if(self.flag_robot_pose_set):

      # Obstacles static
      for obst_i_msg in self.obstacles_static_msg.markers:

        if(obst_i_msg.action == 0):

          # Check distance
          if(obst_i_msg.type == 3):

            obst_i_posi = np.zeros((3,), dtype=float)
            obst_i_posi[0] = obst_i_msg.pose.position.x
            obst_i_posi[1] = obst_i_msg.pose.position.y
            obst_i_posi[2] = obst_i_msg.pose.position.z

            obst_i_rad = obst_i_msg.scale.x/2.0

            distance = np.linalg.norm(obst_i_posi-self.robot_posi)

            if(distance <= obst_i_rad+self.robot_size_radius):
              flag_collision_detected = True

          else:
            print("Unknown obstacle type:"+obst_i_msg.type)



      # Obstacles dynamic
      for obst_i_msg in self.obstacles_dynamic_msg.markers:

        if(obst_i_msg.action == 0):

          # Check distance
          if(obst_i_msg.type == 3):

            obst_i_posi = np.zeros((3,), dtype=float)
            obst_i_posi[0] = obst_i_msg.pose.position.x
            obst_i_posi[1] = obst_i_msg.pose.position.y
            obst_i_posi[2] = obst_i_msg.pose.position.z

            obst_i_rad = obst_i_msg.scale.x/2.0

            distance = np.linalg.norm(obst_i_posi-self.robot_posi)

            if(distance <= obst_i_rad+self.robot_size_radius):
              flag_collision_detected = True

          else:
            print("Unknown obstacle type!!")



    # Publish

    flag_collision_detected_msg = Bool()
    flag_collision_detected_msg.data = flag_collision_detected

    self.robot_collision_pub.publish(flag_collision_detected_msg)


    #
    return