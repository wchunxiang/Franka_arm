#!/usr/bin/env python3
#import copy
import rospy

import numpy as np
import cv2 as cv
#import os
from math import radians, degrees, pi
from std_msgs.msg import Bool, Float64MultiArray

from utils.arduino_control import Stepper_servo
from utils.show import Show_3D
from utils.link_back import link_back
from utils.data_record import Datawriter
from geometry_msgs.msg import Pose, PointStamped
from std_msgs.msg import Bool
from collections import deque
from sensor_msgs.msg import Joy, Image
#from cv_bridge import CvBridge
#from datetime import datetime
from math import radians, degrees, pi



#-------------------------------------------#
step_big = 10e-3
step_small = 2e-3

step_big_pitch = radians(10)
step_small_pitch = radians(5)

step_big_roll = radians(10)
step_small_roll = radians(5)

step_rotation = int(800*0.4) #800 per rotation

vel_scale, acc_scale = 0.1*2, 0.05*2#0.0074*0.05, 0.1#
#-------------------------------------------#
robot_xyz_init = (0,0,200)# holder height, mm
#-------------------------------------------#
end_effector_length =  0.122# 0.122: 50mm [m]
init_arm_x = 0.543 # [m]
init_arm_y = 0.1+end_effector_length
init_arm_z = 0.16 +150e-3#0.16 +125e-3

init_arm_roll = -pi/2
init_arm_pitch = -radians(3) + radians(0)
init_arm_yaw = 0
pose_init = [init_arm_x, init_arm_y, init_arm_z, init_arm_roll, init_arm_pitch, init_arm_yaw]

joints_init = [-0.0659491400771005, 0.3258821602345, 0.05019939570510706, -2.0891858852620175, 1.5067939636488183, 1.5371167519861653, -0.8406383118695672]
#[-0.020323001092885765, 0.34044145924836194, 0.5014064941420989, -2.151300124064228, -1.2973924779843813, 1.16944960256049, 0.9746227700648553]

#-------------------------------------------#
#rotation = 1000
#arm accuracy
arm_acc = 1e-4
serial_no = '/dev/ttyACM0'
b_show_text = True

class Controller(object):
    def __init__(self):
        ###############ros###############
        rospy.init_node('arm_control_2D',anonymous=True)
        self.node_name = rospy.get_name()
        # publish the next destination of link8 to the robot arm
        self.robot_command_pub = rospy.Publisher('/Motion_controller/robot_command',
                                            Pose,
                                            queue_size=5)
        self.robot_joints_pub = rospy.Publisher('/Motion_controller/robot_joints',
                                            Float64MultiArray,
                                            queue_size=5)
        self.robot_vel_scale_pub = rospy.Publisher('/Motion_controller/robot_vel_scale',
                                            Float64MultiArray,
                                            queue_size=5)

        # subscriber: subscibe the reach destination flag from the robot arm
        self.move_sub = rospy.Subscriber('/move_group/b_reach_destination', Bool, self.move_callback, queue_size=10)

        #receive joy information
        # subscriber: subscibe the movement command from joy
        self.joy_info = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)

        #initialize the stepper motor
        self.stepper = Stepper_servo(serial_no=serial_no, servo_vel_step=step_rotation)

        # show text
        if b_show_text:
            self.show = Show_3D(robot_xyz_init= robot_xyz_init)
        self.b_run = False # whether the arm is running
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)
        ###############arm_control###############
        self.thresh_trans=arm_acc
        
        ###############information###############
        self.pose = np.array(pose_init)

        ###############initiallize the pose###############
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = -1, -1, -1, -1, -1, -1# they my=ust be -1 for movepub function
        
        ###############wait for 1s for ros to respond###############
        self.rate = rospy.Rate(2)
        rate = rospy.Rate(1) # 1 Hz
        rate.sleep() 

        # set the velocity and acceleration scale
        array_msg = Float64MultiArray()  
        array_msg.data = [0.2, acc_scale]
        self.robot_vel_scale_pub.publish(array_msg)
        rate.sleep() 

        if len(joints_init)>0:
            self.move_joint(joints_init)
            rate.sleep() 
        self.move_pub(pose_init)

        array_msg = Float64MultiArray()  
        array_msg.data = [vel_scale, acc_scale]
        rate.sleep() 
        self.robot_vel_scale_pub.publish(array_msg)
        rate.sleep() 

        print("----initialize position----")
    
    def joy_callback(self, msg):
        b_up = (msg.axes[7]>0.5)
        b_down = (msg.axes[7]<(-0.5))
        b_left = (msg.axes[6]>(0.5))
        b_right = (msg.axes[6]<(-0.5))

        b_X = (msg.buttons[2]==1)
        b_Y = (msg.buttons[3]==1)
        b_B = (msg.buttons[1]==1)
        b_A = (msg.buttons[0]==1)

        b_left_up = (msg.buttons[4]==1)
        b_right_up = (msg.buttons[5]==1)

        b_start = (msg.buttons[-4]==1)
        b_end = (msg.buttons[-5]==1)

        ##########control position##########
        dx,dy,dz=0,0,0
        #control dx
        if b_right and (not b_left_up) and (not b_right_up):
            dx = step_small
            print("dx go")
        if b_right and (b_left_up) and (not b_right_up):
            dx = step_big
        
        if b_left and (not b_left_up) and (not b_right_up):
            dx = -1*step_small
        if b_left and (b_left_up):
            dx = -1*step_big

        #control dy
        if b_up and (not b_left_up) and (not b_right_up):
            dy = step_small
        if b_up and (b_left_up):
            dy = step_big
        
        if b_down and (not b_left_up) and (not b_right_up):
            dy = -1*step_small
        if b_down and (b_left_up) and (not b_right_up):
            dy = -1*step_big      

        #control dz
        if b_up and (b_right_up) and (not b_left_up):
            dz = step_small
            print('---z---')
        if b_down and (b_right_up) and (not b_left_up):
            dz = -1*step_small

        if b_right and (b_right_up) and (not b_left_up):
            dz = step_big
        if b_left and (b_right_up) and (not b_left_up):
            dz = -1*step_big 

        #print(f'dx:{dx} dy:{dy} dz:{dz}')
        # control pitch
        d_pitch = 0

        if b_X and (not b_right_up) and (not b_left_up):
            d_pitch = -1*step_small_pitch
        if b_B and (not b_right_up) and (not b_left_up):
            d_pitch = step_small_pitch
        if b_X and (b_right_up) and (not b_left_up):
            d_pitch = -1*step_big_pitch
        if b_B and (b_right_up) and (not b_left_up):
            d_pitch = step_big_pitch

        # control roll
        d_roll = 0


        if b_Y and (not b_right_up) and (not b_left_up):
            d_roll = -1*step_small_roll
        if b_A and (not b_right_up) and (not b_left_up):
            d_roll = step_small_roll
        if b_Y and (b_right_up) and (not b_left_up):
            d_roll = -1*step_big_roll
        if b_A and (b_right_up) and (not b_left_up):
            d_roll = step_big_roll

        ### move the robot arm
        pose = [self.x + dx, self.y +dy, self.z +dz, self.roll+d_roll, self.pitch+d_pitch, self.yaw]
        # send command to the arm
        self.move_pub(pose)
        

        #rotate the magnet
        # if b_start and (not b_right_up) and (not b_left_up):
        #     self.stepper.servo_rotate(1000)
        # if b_end and (not b_right_up) and (not b_left_up):
        #     self.stepper.servo_rotate(-1000)
        # # speed
        # if b_start and (b_right_up) and (not b_left_up):
        #     self.stepper.stepper_vel_adjust(1)
        # if b_end and (b_right_up) and (not b_left_up):
        #     self.stepper.stepper_vel_adjust(-1)
        
        # if (b_end or b_start) and (b_left_up) and (not b_right_up):
        #     self.stepper.servo_stop()
        #rotate the magnet
        if b_Y and (not b_right_up) and b_left_up:
            self.stepper.servo_rotate(1000)
        if b_A and (not b_right_up) and b_left_up:
            self.stepper.servo_rotate(-1000)
        # speed
        if b_X and (not b_right_up) and b_left_up:
            self.stepper.stepper_vel_adjust(1)
        if b_B and (not b_right_up) and b_left_up:
            self.stepper.stepper_vel_adjust(-1)
        
        if (b_right_up)and (b_left_up):
            self.stepper.servo_stop()

    def move_pub(self, pose):
        # check whether the translation is beyond the threshold
        xyz_ = np.array(pose)[:3]
        xyz = np.array([self.x, self.y, self.z])
        cond1 = np.any(np.abs(xyz-xyz_)>(self.thresh_trans-self.thresh_trans/100))
        if cond1:
            self.x, self.y, self.z = xyz_
            rospy.loginfo("Arm translation")
        # check whether the rotation is beyond the threshold
        rpy_ = np.array(pose)[3:]
        rpy = np.array([self.roll, self.pitch, self.yaw])
        cond2 = np.any(np.abs(rpy_-rpy)>radians(1-0.01))
        if cond2:
            rospy.loginfo("Arm rotation")
            self.roll, self.pitch, self.yaw = rpy_
        #if the conditions are fulfilled, start moving the robot arm
        if cond1 or cond2:
            self.b_run = True
            self.pose = np.array(pose)
            
            link_pose = link_back(self.x, self.y, self.z, self.roll, self.pitch, self.yaw, end_effector_length)

            #link_pose = self.pose
            wpose = Pose()
            # Arm translation accuracy: 1mm
            wpose.position.x = np.round(link_pose[0]/arm_acc, 0) * arm_acc
            wpose.position.y = np.round(link_pose[1]/arm_acc, 0) * arm_acc
            wpose.position.z = np.round(link_pose[2]/arm_acc, 0) * arm_acc

            wpose.orientation.x = link_pose[3]
            wpose.orientation.y = link_pose[4]
            wpose.orientation.z = link_pose[5]
            wpose.orientation.w = 1 # wait = True
            #print('publish')
            self.robot_command_pub.publish(wpose)
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #self.stepper.servo_rotate(rotation)
            self.b_reach_flag = False
            
            while not rospy.is_shutdown():
                if self.b_reach_flag:
                    #self.stepper.servo_stop()
                    rate = rospy.Rate(100) # 100 Hz
                    rate.sleep() 
                    self.b_reach_flag = False
                    self.b_run = False
                    break


    def move_joint(self, joints):
        array_msg = Float64MultiArray()  
        array_msg.data = joints

        self.robot_joints_pub.publish(array_msg)
        self.b_reach_flag = False
        while not rospy.is_shutdown():
            if self.b_reach_flag:
                break


    def move_callback(self, msg):
        self.b_reach_flag = msg 



    def custom_shutdown(self):
        try:
            if b_show_text:
                cv.destroyAllWindows()
        except Exception:
            pass
        
        rospy.loginfo("[%s] is shutting down..., captain!" %self.node_name)


    def run_show(self):
        while not rospy.is_shutdown():
            if b_show_text:
                x_show = int(np.round((self.x - init_arm_x)*1000,0)) #mm
                y_show = int(np.round((self.y - init_arm_y)*1000,0))
                z_show = int(np.round((self.z - init_arm_z)*1000,0))

                alpha_show = int(degrees(self.roll - init_arm_roll))
                theta_show = int(degrees(self.pitch - init_arm_pitch))

                
                if not self.b_reach_flag and self.b_run: 
                    self.show.draw([x_show, y_show, z_show, alpha_show, theta_show],color =(0,255,0))
                else:
                    #show the xyz and pitch
                    self.show.draw([x_show, y_show, z_show, alpha_show, theta_show],color =(255,255,255))
            else:
                cv.destroyAllWindows()
                break

def main():
    try:
      controller = Controller()
      if b_show_text:
          controller.run_show()
      rospy.spin()

    except rospy.ROSInterruptException:
        #controller.data_writer.release()
        pass

if __name__ == "__main__":
    main()
    # def img_feedback(self,msg):
    #     self.img = self.br.imgmsg_to_cv2(msg)