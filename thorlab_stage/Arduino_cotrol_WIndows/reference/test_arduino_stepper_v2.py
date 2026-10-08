#!/usr/bin/env python
import roslib; roslib.load_manifest('mag_motionstage')
import rospy
import time

from mag_msgs.msg._stage_parameter import stage_parameter
#from mag_msgs.msg._stage_command import stage_command
# from mag_msgs.msg._states_stickslip import states_stickslip

from geometry_msgs.msg import Twist
import sys, select, termios, tty, time

from std_msgs.msg import Float32

## for arduino
import serial
import os

import numpy as np


"Adjust pos with coarse and accurate mode, adjust speed"
"y n/big step   u m/small step   i ,/change speed  q z/record data   a d/extra big step  w x/run or stop"


class ArduinoStepper(object):
    def __init__(self,serial_no = '/dev/ttyACM0'): 
        self.pitch_controller = serial.Serial(serial_no,9600)
        self.speed = None
        self.pos = None
        self.acc= None
        self.stage_sub = rospy.Subscriber("/stageCommand", stage_parameter, self.commandCallback) 
        #adjust pos with coarse mode or precise mode
        self.step_b =10
        self.step_bt = 0
        self.step_s = 1
        self.step_st = 0

        self.continuous_step= 360*3
        self.continuous_pos_t = 0

        self.py_t = 0

        # adjust speed
        self.speed_t = 0
        self.speed_step = 180

        #save txt
        self.save_txt = None

        self.b_first = True
        self.start_time = None
        self.name=None        
        

    def initialize_stepper(self,speed=360,acc=360*10):
        # initializing stage position
        self.speed = speed
        self.acc = acc
        #self.adjustSpeed(speed)
        #time.sleep(0.2)
        #self.adjustAcc(acc)

        #reset arduino
        self.reset_arduino()
    
    def zero_arduino(self): 
        self.pos = 0
        str_pos = 'p'+str(int(self.pos)) + ','
        #str_pos = str(int(self.pos)) + ','
        self.pitch_controller.write(str_pos)

    def reset_arduino(self):
        reset_str = 'r'+str(0) + ','
        print("Reset Arduino")
        self.pitch_controller.write(reset_str) 

    def adjustSpeed(self,speed):
        if speed<0:
            speed = -1*speed
        self.speed = speed
        str_speed = 's'+str(int(self.speed)) + ','
        print("Stepper motor speed="+str(speed))
        self.pitch_controller.write(str_speed)       
    
    def adjustAcc(self,acc):
        if acc<0:
            acc = -1*acc
        self.acc = acc
        str_acc = 'a'+str(int(self.speed)) + ','
        print("Stepper motor acc="+str(acc))
        self.pitch_controller.write(str_acc)  

    def gotoPos(self,pos):
        self.pos = pos
        str_pos = 'p'+str(int(self.pos)) + ','
        self.pitch_controller.write(str_pos)
        print('Stepper motor pos:'+str(self.pos))

    def adjustStep(self,step_change):
        self.stepsize = self.stepsize+step_change
        print("Stepper motor step_size:" +str(self.stepsize))

    def Stop(self):
        str_stop = 'r'+str(int(0)) + ','#reset
        print("!!!!!!!!!!!!!Stop and reset arduino!!!!!!!!!!!!!")
        self.pitch_controller.write(str_stop)  
        self.pos = 0

    def commandCallback(self,msg):
        #move relative coarse mode
        if msg.ox > self.step_bt:
            self.pos = self.pos + self.step_b
            self.gotoPos(self.pos)
        elif msg.ox < self.step_bt:
            self.pos = self.pos - self.step_b
            self.gotoPos(self.pos)

        self.step_bt = msg.ox
        #move relative precise mode
        if msg.oy > self.step_st:
            self.pos = self.pos + self.step_s
            self.gotoPos(self.pos)
        elif msg.oy < self.step_st:
            self.pos = self.pos - self.step_s
            self.gotoPos(self.pos)

        self.step_st = msg.oy
        #change speed
        if msg.oz > self.speed_t:
            self.speed = self.speed + self.speed_step
            self.adjustSpeed(self.speed)
        elif msg.oz < self.speed_t:
            self.speed = self.speed - self.speed_step
            self.adjustSpeed(self.speed)
        if self.speed<0:
            self.speed = 0

        self.speed_t = msg.oz

        #run a large angle
        if msg.px > self.continuous_pos_t:
            self.pos = self.pos + self.continuous_step
            self.gotoPos(self.pos)
        elif msg.px < self.continuous_pos_t:
            self.pos = self.pos - self.continuous_step
            self.gotoPos(self.pos)

        self.continuous_pos_t = msg.px

        # if stop running
        if msg.py != self.py_t:
            self.Stop()
        self.py_t = msg.py
        
        

if __name__=="__main__":
    #initialize the object
    Arduino = ArduinoStepper()

    ###########Callback#################
    def create_folder():
        t = time.localtime()
        current_day = time.strftime("%Y_%m_%d", t)
        homepath = '/home/chunxiang/Documents/'+current_day+'/'

        if not os.path.exists(homepath):
            os.makedirs(homepath)
        return homepath

    def saveCallback(msg): #pz: qz
        if msg.pz>0:
            Arduino.b_flag = True
            if Arduino.b_first == True:
                print('start saving')
        else:
            Arduino.b_flag = False
            print('not save')
        
        if Arduino.b_flag:
            if Arduino.b_first:
                t = time.localtime()
                current_time = time.strftime("%m_%d_%H_%M_%S", t)
                print(current_time)

                Arduino.start_time = rospy.get_rostime().secs
                t_save = (rospy.get_rostime().secs-Arduino.start_time)
                
                Arduino.save_txt = np.array([[t_save, Arduino.pos, Arduino.speed]])
                homepath = create_folder()
                Arduino.name = homepath + current_time
                np.savetxt((Arduino.name+'.txt'),Arduino.save_txt,fmt='%d %.3f %.3f',header='Time  Ang   Speed')

                Arduino.b_first = False
            else:
                if Arduino.b_flag and (Arduino.save_txt is not None):
                    print("next")

                    t_save = (rospy.get_rostime().secs-Arduino.start_time)
                    ang_np = np.array([[t_save, Arduino.pos, Arduino.speed]])
                    Arduino.save_txt = np.r_[Arduino.save_txt,ang_np]
                    np.savetxt((Arduino.name+'.txt'),Arduino.save_txt,fmt='%d %.3f %.3f',header='Time  Ang   Speed')


                Arduino.b_first = False
        else:
            #pass
            Arduino.b_first = True
            Arduino.start_time = None
            Arduino.save_txt = None
            Arduino.name=None
    
    #start
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('arduino_motor', anonymous=True)
    #save doc!!!!!!!!!!!!!!!!!!!!!1
    save_sub = rospy.Subscriber("/stageCommand", stage_parameter, saveCallback)


    Arduino.zero_arduino()
    time.sleep(1)

    Arduino.initialize_stepper()
    time.sleep(1)
    # read in waypoints data: px, py

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

    # home the stage
    Arduino.zero_arduino()
