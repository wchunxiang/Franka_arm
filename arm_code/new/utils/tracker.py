import numpy as np
from utils.adaptive_filter import AdaptiveFilter


class Tracker(object):
	def __init__(self, pos_x, R_s = [0.1, 0.1, 0.1], Q_s = [0.1, 0.5, 5], p_s=[0.7, 0.2, 0.1]):
    	#pos_x: [x_left, x_right]
		self.pos_x = np.array(pos_x)
		self.center_x = np.sum(self.pos_x)/2
		self.width = abs(self.pos_x[1]-self.pos_x[0])

		x_left, x_right = pos_x
		window_width = x_right - x_left
		self.R_s = np.array(R_s)*window_width
		self.Q_s = np.array(Q_s)*self.R_s
		self.AKF_left = AdaptiveFilter([x_left,0],R_s=self.R_s, Q_s=self.Q_s, p_s=p_s)
		self.AKF_right = AdaptiveFilter([x_right,0],R_s=self.R_s, Q_s=self.Q_s, p_s=p_s)

	def predict(self):
		self.AKF_left.predict()
		self.AKF_right.predict()

		self.pos_x = np.array([self.AKF_left.pos[0],self.AKF_right.pos[0]])
		self.center_x = np.sum(self.pos_x)/2
		self.width = self.pos_x[1]-self.pos_x[0]

	def update(self,pos_x):
		self.AKF_left.update([pos_x[0],0])
		self.AKF_right.update([pos_x[1],0])

		self.pos_x = np.array([self.AKF_left.pos[0],self.AKF_right.pos[0]])
		self.center_x = np.sum(self.pos_x)/2
		self.width = self.pos_x[1]-self.pos_x[0]
