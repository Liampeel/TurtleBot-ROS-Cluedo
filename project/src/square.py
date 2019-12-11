#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
import math
from kobuki_msgs.msg import BumperEvent
from cluedofinder import CluedoFinder

bumper = False

class squares():

	def stopRobot(self, data):
		if(data.state == 1):
			self.bumper = True

	def __init__(self):
		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 1)
		self.bumper_subscriber = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.stopRobot)
		self.rate = rospy.Rate(10) #10hz
		self.desired_velocity = Twist()
		self.image_found = False
		#self.image = None
		self.rotation = 0
		self.bumper = False


	def rotate(self, x, z):
		self.desired_velocity.linear.x = x

		self.desired_velocity.angular.z = z
		current_angle = 0
		time_start = time.time()

		cF = CluedoFinder()
		if self.rotation >= 0:
			while(current_angle <= (self.rotation) and not self.bumper):
				self.pub.publish(self.desired_velocity)
				current_angle = (self.desired_velocity.angular.z)*(time.time() - time_start)
				if cF.image_close_enough or cF.image_detected:
					rospy.loginfo("FOUND IMAGE")
					self.image_found = True
				#	self.image = cF.cv_image
					break
		else:
			while(current_angle >= (self.rotation) and not self.bumper):
				self.pub.publish(self.desired_velocity)
				current_angle = (self.desired_velocity.angular.z)*(time.time() - time_start)
				if cF.image_close_enough or cF.image_detected:
					rospy.loginfo("FOUND IMAGE")
					self.desired_velocity.linear.x = 0
					self.desired_velocity.angular.z = 0
					self.pub.publish(self.desired_velocity)
					self.image_found = True
					#self.image = cF.cv_image
					break

	def publishCircle(self):
		angular = 0.2*math.pi

		self.desired_velocity.linear.x=0.2
		self.desired_velocity.angular.z=angular

		self.rotation = 2*math.pi
		self.rotate(0,angular)
		while not self.image_found:
			if self.bumper:
				for i in range(30):
					self.desired_velocity.linear.x=-0.25
					self.desired_velocity.angular.z=0
					self.pub.publish(self.desired_velocity)
				self.bumper = False

			angular = 0.75*angular
			self.rotation = math.pi/2
			self.rotate(0.1,angular)
			self.rotation = -2*math.pi
			self.rotate(0,-0.2*math.pi)

		return self.image_found


	def publish(self):
		self.rotation = 2*math.pi
		self.rotate(0,0.1*math.pi)
		distance = 30
		count = 0

		while not self.image_found:
			if count == 4:
				distance = distance + 5
				count = 0
			self.desired_velocity.angular.z = 0
			self.desired_velocity.linear.x = 0.2
			rospy.loginfo(bumper)

			for i in range (distance):
				if(bumper == False):
					self.pub.publish(self.desired_velocity)
					self.rate.sleep()
				else:
					self.desired_velocity.linear.x = -0.2
					self.pub.publish(self.desired_velocity)
					self.rate.sleep()
			self.rotation = 2*math.pi
			self.rotate(0,0.1*math.pi)
			self.rotation = (math.pi/2)
			self.rotate(0,0.1*math.pi)

			count = count + 1

		return self.image_found

if __name__ == "__main__":
	rospy.init_node('publisher', anonymous=True)
	pB = publisher()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()
		raise
