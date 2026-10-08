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

servo_time = 60000#ms
motor_step =2 #deg.
servo_speed = 8
speed_step =2
def main():
    global servo_speed
    #initialize the serial
    serialcom = serial.Serial('COM3',9600)
    serialcom.timeout = 1
    print('If serial is opened:',end=' ')
    print(serialcom.isOpen())
    for _ in range(2):
        serialcom.write((bytes('w,',encoding='utf-8')))
        time.sleep(0.5)
    #rotate the servo to show the serial is enabled
    serialcom.write((bytes('r100,',encoding='utf-8')))
    serialcom.write((bytes('r-100,',encoding='utf-8')))
    #initialize the image
    img_size = 500
    img_origin = np.zeros((img_size,img_size,3),np.uint8)
    
    font = ImageFont.truetype("arial.ttf", 30)
    img = Image.fromarray(img_origin)
    draw = ImageDraw.Draw(img)
    # font = ImageFont.truetype(<font-file>, <font-size>)
    
    # draw.text((x, y),"Sample Text",(r,g,b))
    draw.text((20, 0),"MOTOR CONTROL------ \n<w-s: pitch> \n<j-k: rotation>",(255,255,255),font=font)
    img_origin = np.array(img)

    cv.namedWindow('Command',cv.WINDOW_AUTOSIZE)
    stepper_angle = 0
    servo_rot_pos = True
    
    while True:
        img_show = img_origin.copy()
        img = Image.fromarray(img_show)
        draw = ImageDraw.Draw(img)
        draw.text((20, 140),"Stepper angle: {}".format(stepper_angle),(0,255,255),font=font)
        if servo_rot_pos:
            draw.text((20, 180),"Servo rotation: {} ms".format(servo_time),(0,255,0),font=font)
        else:
            draw.text((20, 180),"Servo rotation: -{} ms".format(servo_time),(0,255,0),font=font)
        img_show = np.array(img)
        cv.imshow('Command',img_show)
        key = cv.waitKey()
        
        b_run = False
        if key == ord('w'):
            stepper_angle +=motor_step
            string_w = 'p{},'.format(stepper_angle)
            b_run = True
        if key == ord('s'):
            stepper_angle -=motor_step
            string_w = 'p{},'.format(stepper_angle)
            b_run = True

        if key == ord('j'):
            string_w = 'r{},'.format(servo_time)
            servo_rot_pos = True
            b_run = True    
        if key == ord('k'):
            time_temp = -1*servo_time
            string_w = 'r{},'.format(time_temp)
            servo_rot_pos = False
            b_run = True  
        if key == ord('a'):
            servo_speed -= speed_step
            b_run = True 
        if key == ord('d'):
            servo_speed += speed_step
            b_run = True         
        if key == ord('x'):
            string_w = 's{},'.format(stepper_angle)
            b_run = True          
        if b_run:
            string_speed = 'v{},'.format(servo_speed)
            serialcom.write(bytes(string_speed,encoding='utf-8'))
            #print(string_w)
            # send serial data
            time.sleep(0.2)
            serialcom.write(bytes(string_w,encoding='utf-8'))
            # display the information
def test():
    serialcom = serial.Serial('COM3',9600)
    print(serialcom.isOpen())
    serialcom.timeout = 1
    #time.sleep(0.5)
    #serialcom.open()
    serialcom.write((bytes('w,',encoding='utf-8')))
    time.sleep(0.5)
    #serialcom.write('r2000,'.encode('ascii'))
    serialcom.write(bytes('r2000,',encoding='utf-8'))
    line = serialcom.readline().decode('ascii')
    print(line)
    time.sleep(1)
    serialcom.write(bytes('r-2000,',encoding='utf-8'))
    line = serialcom.readline().decode('ascii')
    print(line)
        #print(serialcom.readline().decode('unicode'))
   
if __name__=="__main__":
    main()