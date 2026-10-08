## for arduino
from socket import timeout
import serial
import os
import cv2 as cv
import numpy as np
import time
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw 


class Stepper_servo():
    def __init__(self, serial_no='/dev/ttyACM0',motor_step=2,speed_step=2,servo_vel=8, servo_vel_step=2, b_rotate_start = False) -> None:
        self.servo_vel = servo_vel
        self.servo_vel_step = servo_vel_step
        self.servo_direct = 1000

        self.stepper_angle = 0
        self.motor_step = motor_step
        self.speed_step = speed_step

        #initialize the serial
        self.serialcom = serial.Serial(serial_no,9600)
        self.serialcom.timeout = 1
        print('If serial is opened:',end=' ')
        print(self.serialcom.isOpen())
        for _ in range(2):
            if b_rotate_start:
                self.serialcom.write((bytes('w,',encoding='utf-8')))
            else:
                self.serialcom.write((bytes('s,',encoding='utf-8')))
            time.sleep(0.5)
        #rotate the servo to show the serial is enabled
        if b_rotate_start:
            self.servo_rotate(self.servo_direct)
            time.sleep(1)
            self.servo_stop()
    
    def stepper_move(self, b_direct, step = None):
        if step is None:
            step = self.motor_step
        if b_direct > 0:
            self.stepper_angle +=step
            string_w = 'p{},'.format(self.stepper_angle)
        else:
            self.stepper_angle -=step
            string_w = 'p{},'.format(self.stepper_angle)          
        
        self.serialcom.write(bytes(string_w,encoding='utf-8'))

    def servo_rotate(self, b_direct):
        if b_direct >0:
            self.servo_direct = 1000
            string_w = 'r{},'.format(self.servo_direct)
        else:
            self.servo_direct = -1000
            string_w = 'r{},'.format(self.servo_direct)
        
        self.serialcom.write(bytes(string_w,encoding='utf-8'))

    def servo_vel_adjust(self, b_acc):
        if b_acc >0:
            self.servo_vel +=self.servo_vel_step
        else:
            self.servo_vel -=self.servo_vel_step
        
        string_speed = 'v{},'.format(self.servo_vel)
        self.serialcom.write(bytes(string_speed,encoding='utf-8'))
        time.sleep(0.2)
        self.servo_rotate(self.servo_direct)
    
    def servo_stop(self):
        string_w = 's{},'.format(1000)
        self.serialcom.write(bytes(string_w,encoding='utf-8'))

    def stepper_vel_adjust(self, b_acc):
        if b_acc >0:
            string_speed = 'v{},'.format(self.servo_vel_step)
        else:
            string_speed = 'v{},'.format(-1*self.servo_vel_step)
        
        self.serialcom.write(bytes(string_speed,encoding='utf-8'))
   
if __name__=="__main__":
    stepper_servo = Stepper_servo(servo_vel_step=400)
    stepper_servo.servo_rotate(b_direct=1000)
    # time.sleep(2)
    # stepper_servo.servo_vel_adjust(1)
    # stepper_servo.stepper_move(1, step = 10)
    time.sleep(2)
    stepper_servo.servo_stop()
    time.sleep(2)
    stepper_servo.servo_rotate(b_direct=-1000)
    time.sleep(2)
    stepper_servo.stepper_vel_adjust(b_acc=1)
    time.sleep(2)
    stepper_servo.servo_stop()