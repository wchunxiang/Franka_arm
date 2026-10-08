import cv2 as cv
import numpy as np
import os
from tqdm import tqdm

# enter->the next image
# space->select the next point
# esc->delete the former point

# the coordinate of x,y is relative to the whole image
# | y
# |
# |
# --------------> x


class Select(object):
    def __init__(self, img_path,b_select_ROI = True):
        self.img = cv.imread(img_path)
        self.roi = None
        self.roi_height = None
        self.roi_width = None
        self.roi_x = None
        self.roi_y = None

        self.roi_enlarge_ratio = 1#enlarge the roi

        self.b_select_ROI = b_select_ROI

        self.pos = None
        self.pos_save = None
        self.counter = 0

        self.b_complete = False

        self.linewidth = 3
        self.color=(0,255,0)
        self.radius = 10
        self.circle_color=(0,255,0)
        self.font = cv.FONT_HERSHEY_COMPLEX
        self.font_size = 1

        cv.namedWindow("img",cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback('img', self.get_pos)

    def select_roi(self):
        if self.roi is None:
            cv.namedWindow("ROI",cv.WINDOW_NORMAL)
            x1, y1, w1, h1 = cv.selectROI("ROI", self.img, True, False)
            self.roi = self.img[y1:y1+h1,x1:x1+w1,:]
            self.roi = cv.resize(self.roi,[w1*self.roi_enlarge_ratio,h1*self.roi_enlarge_ratio])
            self.roi_height = h1 * self.roi_enlarge_ratio
            self.roi_width = w1 * self.roi_enlarge_ratio
            self.roi_x = x1
            self.roi_y = y1
            
            cv.destroyWindow("ROI")

    def select_point(self):
        if self.roi is None:
            self.select_roi()

        while(1):  
            img_show = self.roi.copy() 

            if self.pos is not None:
                img_show = self.roi.copy()
                cv.circle(img_show, (self.pos[0], self.pos[1]), self.radius, self.circle_color, 3)
                cv.circle(img_show, (self.pos[0], self.pos[1]), self.radius//3, self.circle_color, -1)

            else:
                img_show = cv.putText(img_show,'Please click left button',(22,22),self.font,self.font_size,(0,0,255),2)
            
            cv.imshow('img', img_show)
            cv.setMouseCallback('img', self.get_pos)
            key = cv.waitKey(20)

            if key == 13 or key == 32:##press enter,press space
                if self.pos is not None:
                    self.pos_save = self.pos #(x,y)
                    self.pos_save[0] = self.roi_x + self.pos[0]/self.roi_enlarge_ratio
                    self.pos_save[1] = self.roi_y + self.pos[1]/self.roi_enlarge_ratio
                    self.b_complete = True
                    self.pos = None
                    cv.destroyAllWindows()
                    break
        return self.pos_save #(x,y)

    def get_pos(self,event,x,y,flags,param):
        if event == cv.EVENT_LBUTTONDBLCLK:
            self.pos = np.array([x, y])
            #print(self.pos)
    
        
def main():
    img_path = '../segmentation.png'

    curve = Select(img_path)
    #while(not curve.b_complete):
    pos = curve.select_point()
    print(pos)

if __name__=='__main__':
    main()