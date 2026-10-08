#!/usr/bin/env python3

import sys, select, termios, tty, time, os
import time
import sys
import pyAPT
import numpy as np
from threading import Thread

#--------------config info.-------------------

X_AXIS_SN = 45155164
Y_AXIS_SN = 45174164
Z_AXIS_SN = 45155234
MAX_DIST =  150 # mm
Max_vel = 50 #mm/s
Max_acc = 50
Delay = 0.001
#Axis = [True,True,True]

#-------------initialize the class--------------
class LinearStage(object):
	def __init__(self, Axis_avail=(None,None,None)): #[X_true, Y_true, Z_True]
		if Axis_avail[0] is None:
			self.Axis = (False,False,False)
		else:
			self.Axis = Axis_avail
		cwd = os.path.abspath(os.getcwd())
		print(cwd)
		# Reading linear stage serial number 

		self.X_AXIS_SN = X_AXIS_SN
		self.Y_AXIS_SN = Y_AXIS_SN
		self.Z_AXIS_SN = Z_AXIS_SN
		
		# Reading distance range and scaling 
		self.MAX_DIST = MAX_DIST

		self.max_velocity = Max_vel  #mm/s
		self.max_acceleration = Max_acc #mm^2/s
		self.delay = Delay
		# flag
		self.b_wait = False # if wait until the stage arrives
		# print(Axis_avail)
		#print(self.Axis)
        # Initialize 3 stages
		if self.Axis[0]:
			print('enable stage x')
			self.x_stage = pyAPT.LTS150(serial_number = X_AXIS_SN)
		if self.Axis[1]:
			print('enable stage y')
			self.y_stage = pyAPT.LTS150(serial_number = Y_AXIS_SN)
		if self.Axis[2]:
			print('enable stage z')
			self.z_stage = pyAPT.LTS150(serial_number = Z_AXIS_SN)  
		self.set_Velocity()


	def getInfoAxis(self, stage):
		ret = stage.info()
		return ret

	'''
	@brief Prints the serial number, model, type, firmware version and servo of all the connected stages.
	'''
	def getInfo(self):
		labels = ['S/N','Model','Type','Firmware Ver', 'Notes', 'H/W Ver', 'Mod State', 'Channels']
		
		if self.Axis[0]:
			xInfo = self.getInfoAxis(self.x_stage)
			print("\nInformation of the X axis:\n")
			print('--------------------------\n')
			for idx, ainfo in enumerate(xInfo):
				print(("\t%12s: %s" % (labels[idx], ainfo)))

		if self.Axis[1]:
			yInfo = self.getInfoAxis(self.y_stage)
			print("\nInformation of the Y axis:\n")
			print('--------------------------\n')
			for idx, ainfo in enumerate(yInfo):
				print(("\t%12s: %s" % (labels[idx], ainfo)))

		if self.Axis[2]:
			zInfo = self.getInfoAxis(self.z_stage)
			print("\nInformation of the Z axis:\n")
			print('--------------------------\n')
			for idx, ainfo in enumerate(zInfo):
				print(("\t%12s: %s" % (labels[idx], ainfo)))
		print("\n")

	'''
	@brief Obtains the current position, velocity and status of a linear stage connected through USB.
	@param[in] axis Serial number of the target linear stage.
	@returns Status for the stage with the serial number provided.
	'''
	def getStatusAxis(self, stage):
		ret = stage.status()
		return ret

	'''
	@brief Prints the axis, position and velocity of the connected stages.
	'''
	def getStatus(self):
		if self.Axis[0]:
			xStatus = self.getStatusAxis(self.x_stage)
		if self.Axis[1]:
			yStatus = self.getStatusAxis(self.y_stage)
		if self.Axis[2]:
			zStatus = self.getStatusAxis(self.z_stage)
		print("Axis:   Position [mm]:   Velocity [mm/s]:\n")
		if self.Axis[0]:
			print(("X %6.3f %6.3f\n" % (xStatus.position, xStatus.velocity)))
		if self.Axis[1]:
			print(("Y %6.3f %6.3f\n" % (yStatus.position, yStatus.velocity)))
		if self.Axis[2]:
			print(("Z %6.3f %6.3f\n" % (zStatus.position, zStatus.velocity)))

	'''
	@brief Provides the position of one or all axes.
	@param[in] axis String with the name of the axis we want to retrieve.
	@returns position[s] [X, Y, Z] of the stage.
	'''
	def getPos(self):
		if self.Axis[0]:
			status = self.x_stage.status()
			posX = status.position
		else:
			posX = -1

		if self.Axis[1]:
			status = self.y_stage.status()
			posY = status.position
		else:
			posY = -1

		if self.Axis[2]:
			status = self.z_stage.status()
			posZ = status.position
		else:
			posZ = -1		
		return [posX, posY, posZ]

	'''
	@brief Sends the 3D linear stage to the position (0, 0, 0).
	'''
	def goHome(self, b_wait = True):
		# Move to home position of the stage
		#self.moveAbsolute(self.MAX_DIST, 0, self.MAX_DIST)
		threads = []
		if self.Axis[0]:
			thread = Thread(target=self.x_stage.home,kwargs={'velocity': self.max_velocity/2})
			thread.start()
			threads.append(thread)

		if self.Axis[1]:
			thread = Thread(target=self.y_stage.home,kwargs={'velocity': self.max_velocity/2})
			thread.start()
			threads.append(thread)

		if self.Axis[2]:
			thread = Thread(target=self.z_stage.home,kwargs={'velocity': self.max_velocity/2})
			thread.start()
			threads.append(thread)
		
		for thread in threads:
			thread.join()

	'''
	@brief Move the stage to the position x, y, z.
	@param[in] x     Position of the x axis in mm.
	@param[in] y     Position of the y axis in mm.
	@param[in] z     Position of the z axis in mm.
	@param[in] delay Delay (in seconds) after each position has been reached.
	'''
	def moveAbsolute(self, pos):
		"pos: [x,y,z]"
		#tx = threading.Thread(target = self.moveAbsoluteX(x))
		#ty = threading.Thread(target = self.moveAbsoluteY(y))
		#tz = threading.Thread(target = self.moveAbsoluteZ(z))
		#tx.daemon = True
		#ty.daemon = True
		#tz.daemon = True
		#tx.start()
		#ty.start()
		#tz.start()
		names = ('X', 'Y', 'Z')
		for indice, point in enumerate(pos):
			if self.Axis[indice]:
				if point<=0:
					print('!!!----'+names[indice]+':'+'reach 0 limit----!!!')
				if point>=MAX_DIST:
					print('!!!----'+names[indice]+':'+'reach max limit----!!!')
		
		pos = np.array(pos)
		pos[pos<0] = 0
		pos[pos>MAX_DIST] = MAX_DIST
		x,y,z = pos

		

		threads = []
		if self.Axis[0]:
			thread = Thread(target=self.x_stage.goto,args=(x,))
			thread.start()
			threads.append(thread)
		if self.Axis[1]:
			thread = Thread(target=self.y_stage.goto,args=(y,))
			thread.start()
			threads.append(thread)
		if self.Axis[2]:
			thread = Thread(target=self.z_stage.goto,args=(z,))
			thread.start()
			threads.append(thread)
		# wait until all axises arrive
		for thread in threads:
			thread.join()
		
		return True

	'''
	@brief Move the stage to the relative position dx, dy, dz.
	@param[in] dx     relative Position of the x axis in mm.
	@param[in] dy     relative Position of the y axis in mm.
	@param[in] dz     relative Position of the z axis in mm.
	@param[in] delay Delay (in seconds) after each position has been reached.
	'''
	def moveRelative(self, dpos):
		"dpos: list [dx,dy,dz]"
		pos_ = self.getPos()
		pos = np.array(pos_)+np.array(dpos)
		self.moveAbsolute(pos)
		return True

	
	def set_Velocity(self, vel=[Max_vel,Max_vel,Max_vel], acc=[Max_acc,Max_acc,Max_acc]):
		#vel = [vel_x, vel_y, vel_z]
		#acc = [acc_x, acc_y, acc_z]
		if self.Axis[0]:
			self.x_stage.set_velocity_parameters(vel[0], acc[0])
		if self.Axis[1]:
			self.y_stage.set_velocity_parameters(vel[1], acc[1])
		if self.Axis[2]:
			self.z_stage.set_velocity_parameters(vel[2], acc[2])	



# def zero_stage():
	
# 	Stage.goHome()

# def initialize_stage():
# 	px = 0
# 	py = 0
# 	pz = 0
# 	Stage.set_Velocity([5,5,5],[5,5,5])
# 	Stage.moveAbsolute(px,py,pz)

# def buttonCallback(msg):
# 	px = msg.px*10
# 	py = msg.py*10
# 	pz = msg.pz*10
	
# 	Stage.moveAbsolute(px, py, pz, False)
# 	Stage.getStatus()

    #stage_x = pyAPT.lts150.LTS150(serial_number=serial_x) # Initialize class
  #con.moveAbsolute(px,py,pz)
    #stage_y = pyAPT.lts150.LTS150(serial_number=serial_y) # Initialize class
    #goto(serial_y,py)
  

if __name__=="__main__":
	pass
	# import rospy
	# #--------------initialize the object-----------------
	# Stage = LinearStage([True,False,False])
    	
	
	# zero_stage()
	# initialize_stage()

	# settings = termios.tcgetattr(sys.stdin)
	# rospy.init_node('xy_stage', anonymous=True)
	# stage_sub = rospy.Subscriber("/stageCommand", stage_parameter, buttonCallback)	

	# rate = rospy.Rate(10) # 10hz
	# while not rospy.is_shutdown():
	# 	rate.sleep()

	# # home the stage
	# zero_stage()
