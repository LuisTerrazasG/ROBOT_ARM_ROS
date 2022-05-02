#!/usr/bin/env pythonS
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_ros
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
from math import pi


class Solution_Planner():

  def __init__(self):


    #Inicializar moveit_commander, nodo rospy y RobotCommander para modelo kinematico
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('solution_template', anonymous = True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    ##Inicializacion de grupos para nuestro brazo y su gripper con moveit commander
    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    gripper_group_name = "xarm_gripper"
    gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
    
    
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

     # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')
    
    # Cargo names
    targets = ["GreenBox",
               "RedBox",
               "BlueBox"]
    # Goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

    # Misc variables
    self.display_trajectory_publisher = display_trajectory_publisher
    self.targets = targets
    self.boxes = boxes
    self.i = 0

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Variables utilizadas
    self.box_name = ''
    self.robot = robot
    self.scene =  scene
    self.move_group = move_group
    self.gripper_group = gripper_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def goToPose(self, pose_goal):
    #TODO: Code used to move to a given position using move it

    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    
    
  def Obstacles_MoveIt(self):
    
    #Lista objetos 
    Boxes = ["RedBox",
               "BlueBox",
               "GreenBox"]
    bx_s = 0.06 # 6 cm box size

    #inicializar escena
    scene = moveit_commander.PlanningSceneInterface()
    
    for i in Boxes:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = i
        box_name = i

        #Agregar posicion de cajas
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.0
        
        #Agregar objeto a escena
        scene.add_box(box_name, box_pose, size=(bx_s, bx_s, bx_s))     
      
    
    #Lista de contenedores 
    deposit_boxes = ["DepositBoxGreen", "DepositBoxRed","DepositBoxBlue"]
    
    for i in deposit_boxes:

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = i
        box_name = i

        #Agregar posicion de depositos 
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.0
        
        #Agregar objeto a escena
        scene.add_box(box_name, box_pose, size=(0.377264, 0.174976, 0.109964))
      
    
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  #Funciones para abrir y cerrar el gripper
  def close_gripper(self):
      self.gripper_group.set_named_target('open')
      self.gripper_group.go(wait=True)
      
  def open_gripper(self):
      self.gripper_group.set_named_target('close')
      self.gripper_group.go(wait=True)
      

#Funcion para mover posicion de robot dependiendo del commando indicado por el usuario
  def go_Position(self, command):
      
    if command == "take":
      pose = self.tf_goal(self.targets[self.i])
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = 1.0 # Rotat 180 grados
      pose_goal.position.x = pose.transform.translation.x
      pose_goal.position.y = pose.transform.translation.y
      pose_goal.position.z = pose.transform.translation.z + 0.05 # 5cm arriba del cubo
      self.goToPose(pose_goal)
      # Move down 5 cm in z
      self.close_gripper()
      pose_goal.position.z -= 0.05
      self.goToPose(pose_goal)
      # Attach box 
      self.attach_handler = rospy.ServiceProxy('AttachObject', AttachObject)
      self.attach_handler(True, box_name)
      # Move up 5 cm in z
      pose_goal.position.z += 0.05
      self.goToPose(pose_goal)

    elif command == "drop":

      pose = self.tf_goal(self.boxes[self.i])
      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.orientation.x = 1.0 # Rotar 180 grados
      pose_goal.position.x = pose.transform.translation.x
      pose_goal.position.y = pose.transform.translation.y
      pose_goal.position.z = pose.transform.translation.z + 0.12 # 12cm arriba 
      self.goToPose(pose_goal)      
      # Mover 6cm arriba
      pose_goal.position.z -= 0.06
      self.goToPose(pose_goal)
      # Detach box
      self.attach_handler = rospy.ServiceProxy('AttachObject', AttachObject)
      self.attach_handler(False, box_name)
      self.open_gripper()
      # Mover 6cm arriba 
      pose_goal.position.z += 0.06
      self.goToPose(pose_goal)
      self.i += 1


#Funcion para mover robor a posicion inicial
  def init_Pos(self):
        
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3

    self.move_group.go(joint_goal, wait = True)


  def tf_goal(self, goal):
    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pose = tfBuffer.lookup_transform("link_base", goal, rospy.Time(), rospy.Duration(1.0))
    print(goal, pose)
    return pose
    
  def main(self):
    #Codigo main
    self.Obstacles_MoveIt()
    self.open_gripper()
    self.init_Pos()
    for j in range(len(self.targets)):
      self.go_Position("take")
      self.go_Position("drop")
    self.init_Pos()
    #self.planner.go_to_joint_state()
    rospy.signal_shutdown("Task Completed")
    



if __name__ == '__main__':
  try:
    node = Solution_Planner()
    node.main()

  except rospy.ROSInterruptException:
    pass
