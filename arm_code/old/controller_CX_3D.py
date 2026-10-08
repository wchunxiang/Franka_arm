#!/usr/bin/env python3
import sys
import copy
from timeit import repeat
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint

import std_msgs.msg
from std_msgs.msg import Bool, Float64MultiArray
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PointStamped
from sensor_msgs.msg import Joy

import tf.transformations as ttf
from tf.transformations import quaternion_from_euler, quaternion_multiply
import numpy as np

from math import radians, degrees, pi

vel_scale_init = [0.5,0.5]#[0.5,0.5] vel, acc
thresh_singular = [1.2, 2.5, 3.5, 4.3]#[0.8, 1.5, 2]
planning_repeat = [3, 5, 5, 2]

joints_init = [-0.0659491400771005, 0.3258821602345, 0.05019939570510706, -2.0891858852620175, 1.5067939636488183, 1.5371167519861653, -0.8406383118695672]

joints_reset= [-0.0659491400771005, 0.3258821602345, 0.05019939570510706, -2.0891858852620175, 1.5067939636488183, 1.5371167519861653, -0.8406383118695672]

#[0.4228227117448439, -0.5020007234359518, 0.4182932243933341, -2.6002492670440063, 2.159595560929852, 2.086919989235376, -0.6475585003370271]
#[0.06506359204277674, 0.051898228899820695, -0.06842529037391355, -1.29558596646238, 0.003286425360022128, 1.3484211778436792, -0.002053053647361129]

#!!!!!!!!!!!!!!!!!!!!!!!!quaternion rxyz!!!!!!!!!!!!!!!!!!!!!!!!#


class controller_CX_rviz(object):
  '''Move the arm according to the received link message'''
  def __init__(self):
    self.b_constraint = False
    #initialize moveit_commander and a rospy node
    self.node_name = rospy.get_name()
    rospy.init_node('controller_CX',anonymous=True)
    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
    robot = moveit_commander.RobotCommander()
    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    if len(vel_scale_init)>0:
      self.vel_scale = vel_scale_init[0]
      self.acc_scale = vel_scale_init[1]
      group.set_max_velocity_scaling_factor(self.vel_scale)
      group.set_max_acceleration_scaling_factor(self.acc_scale)
    # display trajectories in Rviz
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # subscriber: subscibe the movement command from joy
    self.control = rospy.Subscriber("/Motion_controller/robot_command", Pose, self.callback, queue_size=5)
    self.joint_control = rospy.Subscriber('/Motion_controller/robot_joints', Float64MultiArray, self.joint_callback, queue_size=5)
    self.vel_control = rospy.Subscriber('/Motion_controller/robot_vel_scale', Float64MultiArray, self.vel_callback, queue_size=5)


    ##### show visualization information #####
    # Create a publisher to visualize the position constraints in Rviz
    self.marker_publisher = rospy.Publisher(
        "/visualization_marker",
        visualization_msgs.msg.Marker,
        queue_size=20,
    )
    rospy.sleep(0.5)  # publisher needs some time to connect Rviz
    self.remove_all_markers()
    self.marker_id_counter = 0  # give each marker a unique idea

    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print ("============ Reference frame: %s" % planning_frame)

    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)

    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())

    print ("============ Printing robot state")
    # print (group.get_current_joint_values())
    print (robot.get_current_state())
    # print(group.get_end_effector_link())


    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.ref_link = group.get_pose_reference_frame()
    # publish whether the robot has reached the destination
    self.b_reach_publisher = rospy.Publisher('move_group/b_reach_destination',
                                                   Bool,
                                                   queue_size=10)


    # initialize the arm eef pose
    self.eef_x_init = self.group.get_current_pose().pose.position.x
    self.eef_y_init = self.group.get_current_pose().pose.position.y
    self.eef_z_init = self.group.get_current_pose().pose.position.z
    self.eef_orient_init = self.group.get_current_pose().pose.orientation    # in the quaternion format

    self.flag = False#whether the current movement is completed
    self.b_first = True

    if len(joints_init) > 0:
      self.group.go(joints_init, wait=True)

    #self.group.set_goal_tolerance(0.5)

  def callback(self,msg):
    #print('receive the command')
    x_link = msg.position.x
    y_link = msg.position.y
    z_link = msg.position.z

    roll_link = msg.orientation.x
    pitch_link = msg.orientation.y
    yaw_link = msg.orientation.z
    wait = msg.orientation.w

    # plan based on link 8 configuration
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = x_link
    pose_goal.position.y = y_link
    pose_goal.position.z = z_link
    quaternion = quaternion_from_euler(roll_link, pitch_link, yaw_link,'rxyz')
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    # set constraint
    if (not self.b_first) and (self.b_constraint):
      group.set_path_constraints(self.end_constraints)
    else:
      self.clear_constraints()

    # plan mode: cartesian
    # flag, joints_goal = self.plan_cartesian_path(pose_goal)
    # self.group.go(joints_goal, wait=True)

    # plan mode: plan
    try:
      flag, trajectory = self.plan_path(pose_goal)
      # return the reset position and plan the trajectory
      if not flag:
        if len(joints_reset) > 0:
          self.group.go(joints_reset, wait=True)
          flag, trajectory = self.plan_path(pose_goal)
    except Exception:
      print('!!!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!!!')
      pass
    group.execute(trajectory, wait=True)
    ####### Plan mode: 3
    #self.group.set_pose_target(pose_goal)
    #plan = group.go(wait=wait)

    self.b_reach_publisher.publish(True)

    # if wait:
    #   group.stop()
    #   group.clear_pose_targets()

    print('x, y, z: (%.3f, %.3f, %.3f)' %(x_link, y_link, z_link))
    print('roll, pitch, yaw: (%.3f, %.3f, %.3f)' %(degrees(roll_link), degrees(pitch_link), degrees(yaw_link)))
    print (group.get_current_joint_values())

    if self.b_first:
      self.b_first = False
      # self.init_constrains()

  def joint_callback(self,msg):
    print('receive the joint control command')
    group = self.group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = msg.data[0]
    joint_goal[1] = msg.data[1]
    joint_goal[2] = msg.data[2]
    joint_goal[3] = msg.data[3]
    joint_goal[4] = msg.data[4]
    joint_goal[5] = msg.data[5]
    joint_goal[6] = msg.data[6]

    group.go(joint_goal, wait=True)
    self.b_reach_publisher.publish(True)

  def vel_callback(self,msg):
    print('-----------set velocity scale---------')
    self.vel_scale = msg.data[0]
    self.acc_scale = msg.data[1]
    self.group.set_max_velocity_scaling_factor(self.vel_scale)
    self.group.set_max_acceleration_scaling_factor(self.acc_scale)

  def plan_path(self,pose_goal):
    self.group.set_pose_target(pose_goal)
    flag_plan, trajectory, time, error_code = self.group.plan()
    #---------------check singularity----------------------
    self.group.set_pose_target(pose_goal)
    joints_current = self.group.get_current_joint_values()
    joints_next = trajectory.joint_trajectory.points[-1].positions
    error_joints = (np.array(joints_current)-np.array(joints_next))[:-1]#first 6 joints
    error_sum = np.sum(np.abs(error_joints))

    flag = True
    if error_sum > thresh_singular[0]:
      flag = False
      print("!!!!!!!!!!!!!  replan path  !!!!!!!!!!!!!")
      print("Error: %.5f" %error_sum)
      for i, thresh, repeat in zip(list(range(len(thresh_singular))),thresh_singular, planning_repeat):
        flag, trajectory, error = self.replan(pose_goal, thresh, repeat)
        if flag:
          print("^-^ %d, Succeed - error: %.5f ^-^" %(i, error))
          break
        else:
          print("X-X %d, Fail -  error: %.5f X-X" %(i, error))

    return flag, trajectory

  def plan_cartesian_path(self, pose_goal):
    trajectory, fraction = self.group.compute_cartesian_path(
        [pose_goal], 0.001, 0.0 )# waypoints to follow  # eef_step  # jump_threshold
      # calculate errors
    #---------------check singularity----------------------
    joints_current = self.group.get_current_joint_values()
    joints_next = trajectory.joint_trajectory.points[-1].positions
    error_joints = (np.array(joints_current)-np.array(joints_next))[:-1]#first 6 joints
    error_sum = np.sum(np.abs(error_joints))
    flag = True
    if error_sum > thresh_singular[0]:
      flag = False
      print("!!!!!!!!!!!!!  replan catesian path  !!!!!!!!!!!!!")
      print("Error: %.5f" %error_sum)
      for i, thresh, repeat in zip(list(range(len(thresh_singular))),thresh_singular, planning_repeat):
        flag, trajectory, error = self.replan(pose_goal, thresh, repeat)
        if flag:
          print("^-^ %d, Succeed - error: %.5f ^-^" %(i, error))
          break
        else:
          print("X-X %d, Fail -  error: %.5f X-X" %(i, error))

    return flag, trajectory.joint_trajectory.points[-1].positions



  def replan(self, pose_goal, thresh, repeat):
    trajects = []
    indexes = []
    for _ in range(repeat):
      self.group.set_pose_target(pose_goal)
      plan_flag, trajectory, time, error_code = self.group.plan()
      # calculate errors
      joints_current = np.array(self.group.get_current_joint_values())
      joints_next = np.array([trajectory.joint_trajectory.points[i].positions for i in range(len(trajectory.joint_trajectory.points))])
      no = joints_current.shape[0]
      joints_current = joints_current.reshape(-1,no)
      joints_next = joints_next.reshape(-1, no)[1:,:]

      errors_abs = np.abs(joints_current-joints_next)[:,:-1]# neglect the end effector
      errors_sum = np.sum(errors_abs,axis=-1)
      errors_sum_max = np.max(errors_sum)
      # append to list
      trajects.append(trajectory)
      indexes.append(errors_sum_max)
    # find the trajectory with the minimum error
    indice = np.argmin(np.array(indexes))
    flag = True if min(indexes) < thresh else False
    return flag, trajects[indice], indexes[indice]

  def replan_cartesian(self, pose_goal, thresh, repeat):
    trajects = []
    indexes = []
    for _ in range(repeat):
      self.group.set_pose_target(pose_goal)
      trajectory, fraction = self.group.compute_cartesian_path(
        [pose_goal], 0.001, 0.0 )# waypoints to follow  # eef_step  # jump_threshold
      # calculate errors
      joints_current = np.array(self.group.get_current_joint_values())
      joints_next = np.array([trajectory.joint_trajectory.points[i].positions for i in range(len(trajectory.joint_trajectory.points))])
      no = joints_current.shape[0]
      joints_current = joints_current.reshape(-1,no)
      joints_next = joints_next.reshape(-1, no)[1:,:]

      errors_abs = np.abs(joints_current-joints_next)[:,:-1]# neglect the end effector
      errors_sum = np.sum(errors_abs,axis=-1)
      errors_sum_max = np.max(errors_sum)
      # append to list
      trajects.append(trajectory)
      indexes.append(errors_sum_max)
    # find the trajectory with the minimum error
    indice = np.argmin(np.array(indexes))
    flag = True if min(indexes) < thresh else False
    return flag, trajects[indice], indexes[indice]

  def init_constrains(self):
    print("Set motion constraints")
    self.end_constraints = Constraints()
    self.end_constraints.name = "upright"
    pose = self.group.get_current_pose()

    # end orientation constraint
    # end_orientation_constraint = OrientationConstraint()
    # end_orientation_constraint.header = pose.header
    # end_orientation_constraint.link_name = self.group.get_end_effector_link()

    # end_orientation_constraint.orientation = pose.pose.orientation

    # end_orientation_constraint.absolute_x_axis_tolerance = radians(5)
    # end_orientation_constraint.absolute_y_axis_tolerance = radians(5)
    # end_orientation_constraint.absolute_z_axis_tolerance = pi
    # end_orientation_constraint.weight = 0.3

    # self.end_constraints.orientation_constraints.append(end_orientation_constraint)
    # plane constraints
    end_pos_constraint = PositionConstraint()
    end_pos_constraint.header.frame_id = self.group.get_pose_reference_frame()
    end_pos_constraint.link_name = self.group.get_end_effector_link()

    cbox = SolidPrimitive()
    cbox.type = SolidPrimitive.BOX
    cbox.dimensions = [1.0, 1.0, 1e-3]
    end_pos_constraint.constraint_region.primitives.append(cbox)

    cbox_pose = Pose()
    cbox_pose.position.x = pose.pose.position.x
    cbox_pose.position.y = pose.pose.position.y
    cbox_pose.position.z = pose.pose.position.z

    quat = quaternion_from_euler(0, 0, 0)
    cbox_pose.orientation.x = quat[0]
    cbox_pose.orientation.y = quat[1]
    cbox_pose.orientation.z = quat[2]
    cbox_pose.orientation.w = quat[3]
    end_pos_constraint.constraint_region.primitive_poses.append(cbox_pose)
    ###weight###
    end_pos_constraint.weight = 1
    ###weight###
    self.display_box(cbox_pose, cbox.dimensions)

    self.end_constraints.position_constraints.append(end_pos_constraint)
    # set the constraint
    if self.b_constraint:
      self.group.set_path_constraints(self.end_constraints)
    else:
      self.clear_constraints()
    print("Finish setting motion constraints")

    ###For visualization
  def remove_all_markers(self):
    """Utility function to remove all Markers that we potentially published in a previous run of this script."""
    # setup cube / box marker type
    marker = visualization_msgs.msg.Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    # marker.id = 0
    # marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.DELETEALL
    self.marker_publisher.publish(marker)
  def display_box(self, pose, dimensions):
    """Utility function to visualize position constraints."""
    assert len(dimensions) == 3

    # setup cube / box marker type
    marker = visualization_msgs.msg.Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = self.marker_id_counter
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.color = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)
    marker.header.frame_id = self.ref_link

    # fill in user input
    marker.pose = pose
    marker.scale.x = dimensions[0]
    marker.scale.y = dimensions[1]
    marker.scale.z = dimensions[2]

    # publish it!
    self.marker_publisher.publish(marker)
    self.marker_id_counter += 1
  def clear_constraints(self):
    self.group.clear_path_constraints()




def main():

    try:
      controller = controller_CX_rviz()
      rospy.spin()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
  main()
