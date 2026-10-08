#!/usr/bin/env python
import roslib; roslib.load_manifest('mag_motionstage')
import rospy
from mag_msgs.msg._stage_parameter import stage_parameter
import sys, select, termios, tty, time, os
import time
import sys
import pylibftdi
import pyAPT

#--------------config info.-------------------

X_AXIS_SN = 45174164
Y_AXIS_SN = 45155234
Z_AXIS_SN = 45155164
MAX_DIST =  150 # mm
ENCODER_SCALE = 24576
MAX_DIST_ENCODER = MAX_DIST*ENCODER_SCALE
Max_vel = 20 #mm/s
Max_acc = 20
Delay = 0.001
Axis = [True,True,True]

#-------------initialize the class--------------
class LinearStage(object):
	def __init__(self, Axis_avail=[None,None,None]): #[X_true, Y_true, Z_True]
		if Axis_avail[0] is None:
			self.Axis = [False,False,False]
		else:
			self.Axis = Axis_avail
		cwd = os.path.abspath(os.getcwd())
		print(cwd)
		# Reading linear stage serial number 

		self.X_AXIS_SN = 45155164
		self.Y_AXIS_SN = 45155234
		self.Z_AXIS_SN = 45174164
		
		# Reading distance range and scaling 
		self.MAX_DIST = 150
		self.ENCODER_SCALE = 24576
		self.MAX_DIST_ENCODER = self.MAX_DIST * self.ENCODER_SCALE

		self.max_velocity = Max_vel  #mm/s
		self.max_acceleration = Max_acc #mm^2/s
		self.delay = Delay
		# flag
		self.b_wait = False # if wait until the stage arrives

        # Initialize 3 stages
		if self.Axis[0]:
			self.x_stage = pyAPT.LTS150(serial_number = X_AXIS_SN)
		if self.Axis[1]:
			self.y_stage = pyAPT.LTS150(serial_number = Y_AXIS_SN)
		if self.Axis[2]:
			self.z_stage = pyAPT.LTS150(serial_number = Z_AXIS_SN)  


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
				print(("\t%12s: %s" % (labels[idx], bytes(ainfo))))

		if self.Axis[1]:
			yInfo = self.getInfoAxis(self.y_stage)
			print("\nInformation of the Y axis:\n")
			print('--------------------------\n')
			for idx, ainfo in enumerate(yInfo):
				print(("\t%12s: %s" % (labels[idx], bytes(ainfo))))

		if self.Axis[2]:
			zInfo = self.getInfoAxis(self.z_stage)
			print("\nInformation of the Z axis:\n")
			print('--------------------------\n')
			for idx, ainfo in enumerate(zInfo):
				print(("\t%12s: %s" % (labels[idx], bytes(ainfo))))
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
			print(("X %6.3f %6.3f\n" % (xStatus.position, xStatus.velocity)))#float(self.MAX_DIST) - 
		if self.Axis[1]:
			print(("Y %6.3f %6.3f\n" % (yStatus.position, yStatus.velocity)))
		if self.Axis[2]:
			print(("Z %6.3f %6.3f\n" % (zStatus.position, zStatus.velocity)))#float(self.MAX_DIST) - 

	'''
	@brief Provides the position of one or all axes.
	@param[in] axis String with the name of the axis we want to retrieve.
	@returns position[s] [X, Y, Z] of the stage.
	'''
	def getPos(self, axis = None):
		if ((axis == 'X' or axis == 'x') and self.Axis[0]):
			status = self.x_stage.status()
			posX = float(self.MAX_DIST_ENCODER - status.position_apt) / self.ENCODER_SCALE
		else:
			posX = -1

		if ((axis == 'Y' or axis == 'y') and self.Axis[1]):
			status = self.y_stage.status()
			posY = float(status.position_apt) / self.ENCODER_SCALE
		else:
			posY = -1

		if ((axis == 'Z' or axis == 'z') and self.Axis[2]):
			status = self.z_stage.status()
			posZ = float(self.MAX_DIST_ENCODER - status.position_apt) / self.ENCODER_SCALE
		else:
			posZ = -1		
		return [posX, posY, posZ]

	'''
	@brief Sends the 3D linear stage to the position (0, 0, 0).
	'''
	def goHome(self):
		# Move to home position of the stage
		#self.moveAbsolute(self.MAX_DIST, 0, self.MAX_DIST)
		if self.Axis[0]:
			self.x_stage.home(velocity = self.max_velocity)
			self.x_stage.goto(0, wait = False)

		if self.Axis[1]:
			self.y_stage.home(velocity = self.max_velocity)
			self.y_stage.goto(0, wait = False)

		if self.Axis[2]:
			self.z_stage.home(velocity = self.max_velocity)
			self.z_stage.goto(0, wait = False)

    #'''Move absolute'''
	def moveAbsoluteX(self, x, b_wait = None):#abs_pos_mm
		if b_wait is None:
			b_wait = self.b_wait
		#x = float(self.MAX_DIST) - x
		if x < 0:
			x = float(0)
		if x> self.MAX_DIST:
			x = float(self.MAX_DIST)
		self.x_stage.goto(x, wait = False)
		# wait until arriving 
		if b_wait:
			stat = self.x_stage.status()
			while stat.moving:
				time.sleep(self.delay)
				stat = self.x_stage.status()

	'''
	@brief Moving Y axis of the stage to the position y (mm)
	@param[in] y Goal position in mm.
	'''
	def moveAbsoluteY(self, y, b_wait = None):#abs_pos_mm
		if b_wait is None:
			b_wait = self.b_wait
		if y < 0:
			y = float(0)
		if y > self.MAX_DIST:
			y = float(self.MAX_DIST)	
		self.y_stage.goto(y, wait = False)

		if b_wait:
			stat = self.y_stage.status()
			while stat.moving:
				time.sleep(self.delay)
				stat = self.y_stage.status()		

	'''
	@brief Moving Z axis of the stage to the position z (mm)
	@param[in] z Goal position in mm.
	'''
	def moveAbsoluteZ(self, z, b_wait = None):
		if b_wait is None:
			b_wait = self.b_wait
		#z = float(self.MAX_DIST) - z
		if z < 0:
			z = float(0)
		if z > self.MAX_DIST:
			z = float(self.MAX_DIST)	
		self.z_stage.goto(z, wait = False)

		if b_wait:
			stat = self.z_stage.status()
			while stat.moving:
				time.sleep(self.delay)
				stat = self.z_stage.status()	

	'''
	@brief Move the stage to the position x, y, z.
	@param[in] x     Position of the x axis in mm.
	@param[in] y     Position of the y axis in mm.
	@param[in] z     Position of the z axis in mm.
	@param[in] delay Delay (in seconds) after each position has been reached.
	'''
	def moveAbsolute(self, x, y, z, b_wait = None):
		#tx = threading.Thread(target = self.moveAbsoluteX(x))
		#ty = threading.Thread(target = self.moveAbsoluteY(y))
		#tz = threading.Thread(target = self.moveAbsoluteZ(z))
		#tx.daemon = True
		#ty.daemon = True
		#tz.daemon = True
		#tx.start()
		#ty.start()
		#tz.start()
		if b_wait is None:
			b_wait = self.b_wait

		if self.Axis[0]:
			self.moveAbsoluteX(x, b_wait= False)
		if self.Axis[1]:
			self.moveAbsoluteY(y, b_wait= False)
		if self.Axis[2]:
			self.moveAbsoluteZ(z, b_wait= False)
		# wait until all axises arrive
		if b_wait:
			x_moving = b_wait
			y_moving = b_wait
			z_moving = b_wait

			while x_moving or y_moving or z_moving:
				time.sleep(self.delay)
				if self.Axis[0]:	
					x_moving = self.x_stage.status().moving
				else:
					x_moving = False
				if self.Axis[1]:	
					y_moving = self.y_stage.status().moving
				else:
					y_moving = False			
				if self.Axis[2]:	
					z_moving = self.z_stage.status().moving
				else:
					z_moving = False

	'''Move relative'''
	def moveRelativeX(self, dx, b_wait = None):#abs_pos_mm
		if b_wait is None:
			b_wait = self.b_wait
		status = self.x_stage.status()
		posX = float(self.MAX_DIST_ENCODER - status.position_apt) / self.ENCODER_SCALE
		#x = float(self.MAX_DIST) - (posX+dx)
		x = (posX+dx)
		if x < 0:
			x = float(0)
		if x> self.MAX_DIST:
			x = float(self.MAX_DIST)
		self.x_stage.goto(x, wait = False)
		# wait until arriving 
		if b_wait:
			stat = self.x_stage.status()
			while stat.moving:
				time.sleep(self.delay)
				stat = self.x_stage.status()

	def moveRelativeY(self, dy, b_wait = None):#abs_pos_mm
		if b_wait is None:
			b_wait = self.b_wait
		status = self.y_stage.status()
		posY = float(self.MAX_DIST_ENCODER - status.position_apt) / self.ENCODER_SCALE
		y = float(posY+dy)
		if y < 0:
			y = float(0)
		if y > self.MAX_DIST:
			y = float(self.MAX_DIST)	
		self.y_stage.goto(y, wait = False)

		if b_wait:
			stat = self.y_stage.status()
			while stat.moving:
				time.sleep(self.delay)
				stat = self.y_stage.status()

	def moveRelativeZ(self, dz, b_wait = None):#abs_pos_mm
		if b_wait is None:
			b_wait = self.b_wait		
		status = self.z_stage.status()
		posZ = float(self.MAX_DIST_ENCODER - status.position_apt) / self.ENCODER_SCALE
		#z = float(self.MAX_DIST) - (posZ+dz)
		z = posZ + dz
		if z < 0:
			z = float(0)
		if z > self.MAX_DIST:
			z = float(self.MAX_DIST)	
		self.z_stage.goto(z, wait = False)

		if b_wait:
			stat = self.z_stage.status()
			while stat.moving:
				time.sleep(self.delay)
				stat = self.z_stage.status()


	'''
	@brief Move the stage to the relative position dx, dy, dz.
	@param[in] dx     relative Position of the x axis in mm.
	@param[in] dy     relative Position of the y axis in mm.
	@param[in] dz     relative Position of the z axis in mm.
	@param[in] delay Delay (in seconds) after each position has been reached.
	'''
	def moveRelative(self, x, y, z, b_wait = None):
		if b_wait is None:
			b_wait = self.b_wait

		if self.Axis[0]:
			self.moveRelativeX(x, b_wait= False)
		if self.Axis[1]:
			self.moveRelativeY(y, b_wait= False)
		if self.Axis[2]:
			self.moveRelativeZ(z, b_wait= False)
		# wait until all axises arrive

		x_moving = b_wait
		y_moving = b_wait
		z_moving = b_wait

		while x_moving or y_moving or z_moving:
			time.sleep(self.delay)
			if self.Axis[0]:	
				x_moving = self.x_stage.status().moving
			else:
				x_moving = False
			if self.Axis[1]:	
				y_moving = self.y_stage.status().moving
			else:
				y_moving = False			
			if self.Axis[2]:	
				z_moving = self.z_stage.status().moving
			else:
				z_moving = False	
	
	def set_Velocity(self, vel=[Max_vel,Max_vel,Max_vel], acc=[Max_acc,Max_acc,Max_acc]):
		#vel = [vel_x, vel_y, vel_z]
		#acc = [acc_x, acc_y, acc_z]
		if self.Axis[0]:
			self.x_stage.set_velocity_parameters(vel[0], acc[0])
		if self.Axis[1]:
			self.y_stage.set_velocity_parameters(vel[1], acc[1])
		if self.Axis[2]:
			self.z_stage.set_velocity_parameters(vel[2], acc[2])	







def zero_stage():
	
	Stage.goHome()

def initialize_stage():
	px = 0
	py = 0
	pz = 0
	Stage.set_Velocity([5,5,5],[5,5,5])
	Stage.moveAbsolute(px,py,pz)

def buttonCallback(msg):
	px = msg.px*10
	py = msg.py*10
	pz = msg.pz*10
	
	Stage.moveAbsolute(px, py, pz, False)
	Stage.getStatus()

    #stage_x = pyAPT.lts150.LTS150(serial_number=serial_x) # Initialize class
  #con.moveAbsolute(px,py,pz)
    #stage_y = pyAPT.lts150.LTS150(serial_number=serial_y) # Initialize class
    #goto(serial_y,py)
  

if __name__=="__main__":
	#--------------initialize the object-----------------
	Stage = LinearStage([True,False,False])
    	
	
	zero_stage()
	initialize_stage()

	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('xy_stage', anonymous=True)
	stage_sub = rospy.Subscriber("/stageCommand", stage_parameter, buttonCallback)	

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		rate.sleep()

	# home the stage
	zero_stage()
