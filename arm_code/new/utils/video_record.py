import cv2 as cv
import numpy as np
from pathlib import Path
import os

class Videowriter(object):
    def __init__(self, im0, file_path):
        output_folder = str(Path(file_path).parents[0])
        os.makedirs(output_folder, exist_ok=True)
        print('-----'+"Start recording video: "+ output_folder +'-----')
        fps, w, h = 30, im0.shape[1], im0.shape[0]
        self.vid_writer = cv.VideoWriter(file_path, cv.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        self.vid_writer.write(im0)
    
    def write(self,frame):
        self.vid_writer.write(frame)

    def release(self):
        print('-----Finish recording video-----')
        self.vid_writer.release()