#!/usr/bin/env python

'''
This is a python file that starts a ROS node called drone_control which holds the position of the drone at a given point (specified in whycon frame).
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/zero_line
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [5,5,18] 
		
		
		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [345*0.05,141*0.05,349*0.05]
		self.Ki = [0,0,11*0.009]
		self.Kd = [484*0.4,419*0.4,413*0.4]

		#-----------------------Adding other required variables for pid here ----------------------------------------------

		self.prev_error = [0,0,0]
		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]
		self.error_sum = [0,0,0]
		self.result = [0,0,0]
		
		self.out_alt = 0
		self.out_pitch = 0
		self.out_roll = 0

		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which the PID runs. 
		self.sample_time = 0.060 # in seconds
		self.last_time = 0
		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=10)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=10)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=10)
		self.zero_line_pub = rospy.Publisher('/zero_line',Float64, queue_size = 10)

		#-----------------------------------------------------------------------------------------------------------

		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
	
	# Disarming the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	# Arming the drone : first disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
		
	# Whycon callback function
	
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		

		#--------------------Set the remaining co-ordinates of the drone from msg as shown in the example line above ----------------------------------------------

		#YOUR CODE HERE
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		
		#---------------------------------------------------------------------------------------------------------------

	# Callback function for /pid_tuning_altitude
	
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.05 # This is an example and you need to change the ratio/fraction value accordingly for tuning
		self.Ki[2] = alt.Ki * 0.009
		self.Kd[2] = alt.Kd * 0.4

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.05 # This is an example and you need to change the ratio/fraction value accordingly for tuning
		self.Ki[1] = pitch.Ki * 0.009
		self.Kd[1] = pitch.Kd * 0.4
		
	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.05 # This is an example and you need to change the ratio/fraction value accordingly for tuning
		self.Ki[0] = roll.Ki * 0.009
		self.Kd[0] = roll.Kd * 0.4

	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#----------------------------- PID algorithm --------------------------------------------------------------

		self.zero_line = 0.00
		self.now = time.time()
		#self.last_time = 0
		self.time_change = self.now - self.last_time
		

		if(self.time_change>0.050):
		
			#Step 1 - Calculate the error 
			#We are calculating difference between the error and the set point.
			#x-roll-movement along x
			#y-pitch-movement along y 
			#z- throttle - along z axis
			#We are not going to control yaw as it is not needed in this application
			#WHAT IS ZERO LINE
			self.error_roll = self.drone_position[0] - self.setpoint[0]
			self.error_pitch = self.drone_position[1] - self.setpoint[1]
			self.error_alt = self.drone_position[2] - self.setpoint[2]
			
			#YOUR CODE HERE
			
			#Publish the errors
			#YOUR CODE HERE
			#Figure out which axis denotes what.
			self.roll_error_pub.publish(self.error_roll)
			self.pitch_error_pub.publish(self.error_pitch)
			self.alt_error_pub.publish(self.error_alt)
		    
	
			#Step 2- Compute error, change_in_error
			
			#YOUR CODE HERE
			#Let us create a list so that errors can be calculated easily.
			self.error = [self.error_roll,self.error_pitch,self.error_alt]
			#Change from what value?? Change from the prev value, prev error array has been defined earlier, use that.
			self.change_in_error = [self.error[0]-self.prev_error[0], self.error[1]-self.prev_error[1], self.error[2]-self.prev_error[2]]
			
			#Step 3 - Calculating PID output, using the equation 
			# output = (kp * error) + Iterm + (Kd * errro-prev_error)
			#We need to calculate three outputs, as we are controlling three parameters, roll, pitch and alt.
			#Intuitively you can understand that you will have three sets of kp,ki,ki values so use the corresponding gain parameters to calculate the PID output.
			#We have error
			# We have change in error
			#Error sum - defined in line 61 - I was pondering over it for hours,but it was right infront of my eyes!!!!!
			self.out_roll = (self.Kp[0]*self.error[0]) + (self.Ki[0]*self.error_sum[0]*self.sample_time) + (self.Kd[0]*self.change_in_error[0]/self.sample_time)
			self.out_pitch = (self.Kp[1]*self.error[1]) + (self.Ki[1]*self.error_sum[1]*self.sample_time) + (self.Kd[1]*self.change_in_error[1]/self.sample_time)
			self.out_alt =  (self.Kp[2]*self.error[2]) + (self.Ki[2]*self.error_sum[2]*self.sample_time) + (self.Kd[2]*self.change_in_error[2]/self.sample_time)
			
			#YOUR CODE HERE
	
			#Step 4 - Apply the output obtained for roll, pitch and throttle and add/subtract to the respective topics. Uncomment and complete 
			#Refer pdf for pointers
			self.cmd.rcPitch = 1500 + self.out_pitch #YOUR CODE HERE
			self.cmd.rcRoll =  1500 - self.out_roll#YOUR CODE HERE - Confirmed that it is - 
			self.cmd.rcThrottle =  1500 + self.out_alt#YOUR CODE HERE #- Confirmed that it is +
			#Step 5 - Limiting the output to ensure it stays within the specified limits 
			#Limiting Roll
			#Like capping that you saw on the video
			if self.cmd.rcRoll > self.max_values[0]:
				self.cmd.rcRoll = self.max_values[0]
				
			elif self.cmd.rcRoll < self.min_values[0]:
				self.cmd.rcRoll = self.min_values[0]

			#Limiting Pitch
			if self.cmd.rcPitch > self.max_values[1]:
				self.cmd.rcPitch = self.max_values[1]
				
			elif self.cmd.rcPitch < self.min_values[1]:
				self.cmd.rcPitch = self.min_values[1]
		
			#Limiting Throttle
			if self.cmd.rcThrottle > self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]
				
			elif self.cmd.rcThrottle < self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]

			self.command_pub.publish(self.cmd)

		
			#Step 8 - Calculate sum of errors here

			#YOUR CODE HERE
			for i in range(len(self.error)):
				self.error_sum[i] += self.error[i]


			#Step 7 - Update previous error 
			#Current error will be the previous error in the next iteration, which is pretty intuitive 
			#YOUR CODE HERE
			self.prev_error = self.error
			self.last_time = self.now
			#------------------------------------------------------------------------------------------------------------------------
			
			
if __name__ == '__main__':

	e_drone = Edrone()
	while not rospy.is_shutdown():
		e_drone.pid()