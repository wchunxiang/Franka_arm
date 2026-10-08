import cv2 as cv
import numpy as np
import time
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw 

class Show(object):
    def __init__(self, robot_xyz_init = None, xyz_ang_init=(0,0,0,0), img_size = 300) -> None:
        # robot_xyz: robot position relative to the magnet
        # xyz_ang_init: tuple(x:mm,y:mm,z:mm,deg)
        self.img_origin = np.zeros((int(img_size),int(img_size),3),np.uint8)
        self.font = ImageFont.truetype("~/usr/share/fonts/truetype/freefont/FreeMono.ttf", int(img_size*40/300))
        self.font_s = ImageFont.truetype("~/usr/share/fonts/truetype/freefont/FreeMono.ttf", int(img_size*25/300))
        self.xyz_ang_init = np.array(xyz_ang_init)
        self.robot_xyz_init = robot_xyz_init
        cv.namedWindow('Command',cv.WINDOW_AUTOSIZE)
        self.draw(self.xyz_ang_init)
        cv.waitKey(1000)
        print("--show text--")

    def draw(self,xyz_ang_relative,color=(255,255,255)):
        
        # xyz_ang_relative: tuple(x,y,z,deg)
        xyz_ang = self.xyz_ang_init+np.array(xyz_ang_relative)
        #draw text
        img_show = self.img_origin.copy()
        img = Image.fromarray(img_show)
        draw = ImageDraw.Draw(img)

        
        string = "x(mm):{:d} \ny(mm):{:d} \nz(mm):{:d} \nθ(°):{:d}".format(xyz_ang[0],xyz_ang[1],xyz_ang[2],xyz_ang[3])

        

        draw.text((20, 20),string,font=self.font,fill=color)
        if self.robot_xyz_init is not None:
            text_size = draw.textbbox((20, 20),string,font=self.font)
            
            string1 = 'robot_xyz(mm):\n({:d}, {:d}, {:d})'.format(self.robot_xyz_init[0],self.robot_xyz_init[1],self.robot_xyz_init[2])
            
            draw.text((20, text_size[-1]),string1,font=self.font_s,fill=(255,255,255))
        
        img_show = np.array(img)
        cv.imshow('Command',img_show)
        key = cv.waitKey(100)

    def draw_3D(self,xyz_ang_relative,color=(255,255,255)):
        
        # xyz_ang_relative: tuple(x,y,z,deg)
        xyz_ang = self.xyz_ang_init+np.array(xyz_ang_relative)
        #draw text
        img_show = self.img_origin.copy()
        img = Image.fromarray(img_show)
        draw = ImageDraw.Draw(img)

        
        string = "x(mm):{:d} \ny(mm):{:d} \nz(mm):{:d} \nα(°):{:d} \nθ(°):{:d}".format(xyz_ang[0],xyz_ang[1],xyz_ang[2],xyz_ang[3],xyz_ang[4])

        

        draw.text((20, 20),string,font=self.font,fill=color)
        if self.robot_xyz_init is not None:
            text_size = draw.textbbox((20, 20),string,font=self.font)
            
            string1 = 'robot_xyz(mm):\n({:d}, {:d}, {:d})'.format(self.robot_xyz_init[0],self.robot_xyz_init[1],self.robot_xyz_init[2])
            
            draw.text((20, text_size[-1]),string1,font=self.font_s,fill=(255,255,255))
        
        img_show = np.array(img)
        cv.imshow('Command',img_show)
        key = cv.waitKey(100)

class Show_3D(object):
    def __init__(self, robot_xyz_init = None, xyz_ang_init=(0,0,0,0,0), img_size = 240) -> None:
        # robot_xyz: robot position relative to the magnet
        # xyz_ang_init: tuple(x:mm,y:mm,z:mm,deg)
        self.img_origin = np.zeros((int(img_size),int(img_size),3),np.uint8)
        self.font = ImageFont.truetype("~/usr/share/fonts/truetype/freefont/FreeMono.ttf", int(img_size*40/300))
        self.font_s = ImageFont.truetype("~/usr/share/fonts/truetype/freefont/FreeMono.ttf", int(img_size*25/300))
        self.xyz_ang_init = np.array(xyz_ang_init)
        self.robot_xyz_init = robot_xyz_init
        cv.namedWindow('Command',cv.WINDOW_AUTOSIZE)
        self.draw(self.xyz_ang_init)
        cv.waitKey(1000)
        print("--show text--")

    def draw(self,xyz_ang_relative,color=(255,255,255)):
        
        # xyz_ang_relative: tuple(x,y,z,deg)
        xyz_ang = self.xyz_ang_init+np.array(xyz_ang_relative)
        #draw text
        img_show = self.img_origin.copy()
        img = Image.fromarray(img_show)
        draw = ImageDraw.Draw(img)

        
        string = "x(mm):{:d} \ny(mm):{:d} \nz(mm):{:d} \nα(°):{:d} \nθ(°):{:d}".format(xyz_ang[0],xyz_ang[1],xyz_ang[2],xyz_ang[3],xyz_ang[4])

        

        draw.text((20, 20),string,font=self.font,fill=color)
        if self.robot_xyz_init is not None:
            text_size = draw.textbbox((20, 20),string,font=self.font)
            
            string1 = 'robot_xyz(mm):\n({:d}, {:d}, {:d})'.format(self.robot_xyz_init[0],self.robot_xyz_init[1],self.robot_xyz_init[2])
            
            draw.text((20, text_size[-1]),string1,font=self.font_s,fill=(255,255,255))
        
        img_show = np.array(img)
        cv.imshow('Command',img_show)
        key = cv.waitKey(100)
    
if __name__ == "__main__":
    info = Show((120,120,120,0))
