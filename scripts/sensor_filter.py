#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import WrenchStamped



class sensorFilter():
	def __init__(self):
		# Subscribers
		rospy.Subscriber("/optoforce_node/OptoForceWrench", WrenchStamped, self.OptoforceSensorCallback)

		# Publishers
		self.sensorFilteredPub = rospy.Publisher('/optoforce_node/OptoForceWrench_filtered2', WrenchStamped, queue_size = 1)

		self.sensorReadings = []
		self.readingCount = 0

		self.sensorReadingAvaraged = WrenchStamped()
		self.sensorReadingAvaraged_old = WrenchStamped()
		self.filterCoef = 1


		#filter coefs
		self.a0 = 0.00385
		self.a1 = 0.007699
		self.a2 = 0.00385
		self.b0 = 0.9254
		self.b1 = -1.91
		self.b2 = 1.0

		self.filter_u = [0, 0, 0, 0, 0, 0]		# input u(k)
		self.filter_u_k = [0, 0, 0, 0, 0, 0]	# input u(k-1)
		self.filter_u_kk = [0, 0, 0, 0, 0, 0]	# input u(k-2)

		self.filter_y = [0, 0, 0, 0, 0, 0]		# input y(k)
		self.filter_y_k = [0, 0, 0, 0, 0, 0]	# input y(k-1)
		self.filter_y_kk = [0, 0, 0, 0, 0, 0]	# input y(k-2)

	def OptoforceSensorCallback(self, msg):


		self.filter_u = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

		for i in range(0, 6):
			self.filter_y[i] = (self.a2/self.b2)*self.filter_u[i] + (self.a1/self.b2)*self.filter_u_k[i] + (self.a0/self.b2)*self.filter_u_kk[i] - (self.b1/self.b2)*self.filter_y_k[i] - (self.b0/self.b2)*self.filter_y_kk[i]  


		self.sensorReadingAvaraged.header.stamp = msg.header.stamp #.Time.now()
		self.sensorReadingAvaraged.wrench.force.x = self.filter_y[0]
		self.sensorReadingAvaraged.wrench.force.y = self.filter_y[1]
		self.sensorReadingAvaraged.wrench.force.z = self.filter_y[2]
		self.sensorReadingAvaraged.wrench.torque.x = self.filter_y[3]
		self.sensorReadingAvaraged.wrench.torque.y = self.filter_y[4]
		self.sensorReadingAvaraged.wrench.torque.z = self.filter_y[5]
			
		self.sensorFilteredPub.publish(self.sensorReadingAvaraged)

		# update old states

		for i in range(0, 6):
			self.filter_y_kk[i] = self.filter_y_k[i]
			self.filter_y_k[i] = self.filter_y[i]
			self.filter_u_kk[i] = self.filter_u_k[i]
			self.filter_u_k[i] = self.filter_u[i]

	def run(self):
		
		while not rospy.is_shutdown():

			#print "running"
			rospy.sleep(0.001)


if __name__ == '__main__':

	rospy.init_node('SensorFilter')
	node = sensorFilter()

	node.run()
