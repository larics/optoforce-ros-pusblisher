#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import WrenchStamped



class sensorFilter():
	def __init__(self):
		# Subscribers
		rospy.Subscriber("/optoforce_node/OptoForceWrench", WrenchStamped, self.OptoforceSensorCallback)

		# Publishers
		self.sensorFilteredPub = rospy.Publisher('/optoforce_node/OptoForceWrench_filtered', WrenchStamped, queue_size = 1)

		self.sensorReadings = []
		self.readingCount = 0

		self.sensorReadingAvaraged = WrenchStamped()
		self.sensorReadingAvaraged_old = WrenchStamped()
		self.filterCoef = 1

	def OptoforceSensorCallback(self, msg):

		if (self.readingCount < 9):
			self.sensorReadings.append(msg)
			self.readingCount = self.readingCount + 1
		else:
			#perform calculation
			Fx = 0
			Fy = 0
			Fz = 0
			Tx = 0
			Ty = 0
			Tz = 0
			
			for reading in self.sensorReadings:

				Fx = Fx + reading.wrench.force.x
				Fy = Fy + reading.wrench.force.y
				Fz = Fz + reading.wrench.force.z
				Tx = Tx + reading.wrench.torque.x
				Ty = Ty + reading.wrench.torque.y
				Tz = Tz + reading.wrench.torque.z

			self.sensorReadingAvaraged.header.stamp = rospy.Time.now()
			self.sensorReadingAvaraged.wrench.force.x = Fx / 9.0
			self.sensorReadingAvaraged.wrench.force.y = Fy / 9.0
			self.sensorReadingAvaraged.wrench.force.z = Fz / 9.0
			self.sensorReadingAvaraged.wrench.torque.x = Tx / 9.0
			self.sensorReadingAvaraged.wrench.torque.y = Ty / 9.0
			self.sensorReadingAvaraged.wrench.torque.z = Tz / 9.0
			

			# PT1
			self.sensorReadingAvaraged.wrench.force.x = self.filterCoef*self.sensorReadingAvaraged.wrench.force.x + (1-self.filterCoef)*self.sensorReadingAvaraged_old.wrench.force.x
			self.sensorReadingAvaraged.wrench.force.y = self.filterCoef*self.sensorReadingAvaraged.wrench.force.y + (1-self.filterCoef)*self.sensorReadingAvaraged_old.wrench.force.y
			self.sensorReadingAvaraged.wrench.force.z = self.filterCoef*self.sensorReadingAvaraged.wrench.force.z + (1-self.filterCoef)*self.sensorReadingAvaraged_old.wrench.force.z
			self.sensorReadingAvaraged.wrench.torque.x = self.filterCoef*self.sensorReadingAvaraged.wrench.torque.x + (1-self.filterCoef)*self.sensorReadingAvaraged_old.wrench.torque.x
			self.sensorReadingAvaraged.wrench.torque.y = self.filterCoef*self.sensorReadingAvaraged.wrench.torque.y + (1-self.filterCoef)*self.sensorReadingAvaraged_old.wrench.torque.y
			self.sensorReadingAvaraged.wrench.torque.z = self.filterCoef*self.sensorReadingAvaraged.wrench.torque.z + (1-self.filterCoef)*self.sensorReadingAvaraged_old.wrench.torque.z
			
			self.sensorReadingAvaraged_old = self.sensorReadingAvaraged

			self.sensorFilteredPub.publish(self.sensorReadingAvaraged)

			self.readingCount = 0
			self.sensorReadings = []

	def run(self):
		
		while not rospy.is_shutdown():

			#print "running"
			rospy.sleep(0.01)


if __name__ == '__main__':

	rospy.init_node('SensorFilter')
	node = sensorFilter()

	node.run()
