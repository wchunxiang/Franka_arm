#!/usr/bin/env python3
from tele_xy_thorlab_v5 import LinearStage
from arduino_control import Stepper_servo

import rospy
import numpy as np
from math import radians, cos, sin
from threading import Thread
from sensor_msgs.msg import Joy

step_stage_big = 6#mm
step_stage_small = 2#mm

step_small_stepper = 10 #deg
step_big_stepper = 15 #deg

L_mag = 42 #mm

b_enable_init = True
serial_no = '/dev/ttyACM0'

class Thorlab_stage():
    '''use joy0 to control
    
    make sure the stepper motor align with the x axis at first frame'''
    def __init__(self) -> None:
        rospy.init_node('thorlab_stage',anonymous=True)
        self.node_name = rospy.get_name()
        self.joy_info = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=10)
        rospy.on_shutdown(self.custom_shutdown)

        self.stage = LinearStage(Axis_avail=(True,True,False))
        self.stepper_servo = Stepper_servo(serial_no=serial_no)

        self.xyz = None
        self.ang_stepper = 0

        #for controlling the running of joy callback
        self.b_run = False
        self.b_init = True
        
        print('make sure the stepper motor align with the x axis at first frame')

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

        b_stage = False

        #enable or disable stage
        if b_A and b_Y:
            self.b_run = True
        if b_X and b_B:
            self.b_run = False
        
        #######xyz stage
        dx,dy,dz=0,0,0
        #control dx
        if b_right and (not b_left_up) and (not b_right_up):
            dx = step_stage_small
        if b_right and (b_left_up) and (not b_right_up):
            dx = step_stage_big
        
        if b_left and (not b_left_up) and (not b_right_up):
            dx = -1*step_stage_small
        if b_left and (b_left_up):
            dx = -1*step_stage_big

        #control dy
        if b_up and (not b_left_up) and (not b_right_up):
            dy = step_stage_small
        if b_up and (b_left_up):
            dy = step_stage_big
        
        if b_down and (not b_left_up) and (not b_right_up):
            dy = -1*step_stage_small
        if b_down and (b_left_up) and (not b_right_up):
            dy = -1*step_stage_big
        
        #control dz
        if b_up and (b_right_up) and (not b_left_up):
            dz = step_stage_small
        if b_down and (b_right_up) and (not b_left_up):
            dz = -1*step_stage_small

        if b_right and (b_right_up) and (not b_left_up):
            dz = step_stage_big
        if b_left and (b_right_up) and (not b_left_up):
            dz = -1*step_stage_big

        if np.any(np.abs(np.array([dx,dy,dz]))>1e-3):
            b_stage = True
        
        run_list = [-1*dx,dy,-1*dz]
        if b_stage and self.b_run:
            print(run_list)
            self.stage.moveRelative(run_list)
        if b_left_up and b_right_up and self.b_run:
            if self.b_init and b_enable_init:
                self.b_init = False
                self.stage.goHome()

        if not self.b_run:
            print('--print button Y (X) and A (B) to enable (disable) the stage--')
            #print('print button X and B to disable the stage')
        
        #### control servo
        if self.b_run:
            #### contro servo motor
            #direction
            if b_Y and (not b_right_up):
                self.stepper_servo.servo_rotate(1000)
            if b_A and (not b_right_up):
                self.stepper_servo.servo_rotate(-1000)
            # speed
            if b_Y and (b_right_up):
                self.stepper_servo.servo_vel_adjust(1)
            if b_A and (b_right_up):
                self.stepper_servo.servo_vel_adjust(-1)
            
            if (b_A or b_Y) and (b_left_up):
                self.stepper_servo.servo_stop()
            #### control stepper
            ang_before = self.ang_stepper 
            b_run_stepper = False
            if b_X and (not b_right_up):
                b_run_stepper = True
                self.stepper_servo.stepper_move(-1,step_small_stepper)
                self.ang_stepper -= step_small_stepper
            if b_B and (not b_right_up):
                b_run_stepper = True
                self.stepper_servo.stepper_move(1,step_small_stepper)  
                self.ang_stepper += step_small_stepper
            if b_X and (b_right_up):
                b_run_stepper = True
                self.stepper_servo.stepper_move(-1,step_big_stepper)
                self.ang_stepper -= step_big_stepper
            if b_B and (b_right_up):
                b_run_stepper = True
                self.stepper_servo.stepper_move(1,step_big_stepper)  
                self.ang_stepper += step_big_stepper
            
            # rotate the stepper motor
            if b_run_stepper:
                print('Stepper motor angle: {} deg.'.format(self.ang_stepper))
                run_list = self.dxy_fix_end(ang_before,self.ang_stepper)
                self.stage.moveRelative(run_list)
    
    def dxy_fix_end(self, ang_before, ang_after):
        """_summary_
        Args:
            ang_before (deg): the angle before rotation
            ang_after (deg): the angle after rotation
        Return:
        dx: mm
        dy: mm
        dz: 0
        """        

        x_before = L_mag*cos(radians(ang_before))
        y_before = L_mag*sin(radians(ang_before))

        x_after = L_mag*cos(radians(ang_after))
        y_after = L_mag*sin(radians(ang_after))

        dx = -1*(x_before - x_after)
        dy = -1*(y_before - y_after)

        return (dx,dy,0)



    def custom_shutdown(self):
        if self.ang_stepper > 0:
            self.stepper_servo.stepper_move(-1,self.ang_stepper) 
        else:
            self.stepper_servo.stepper_move(1,abs(self.ang_stepper)) 

        rospy.loginfo("[%s] is shutting down..., captain!" %self.node_name)

def main():
    try:
      controller = Thorlab_stage()
      rospy.spin()

    except rospy.ROSInterruptException:
        #controller.data_writer.release()
        pass

if __name__ == "__main__":
    main()