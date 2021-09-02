#!/usr/bin/env python

import rospy
import csv
from geometry_msgs.msg import WrenchStamped
import copy
import numpy as np

class sensorMedianFilter():
    def __init__(self):
        # Subscribers
        rospy.Subscriber("/optoforce_node/OptoForceWrench_raw", WrenchStamped, self.OptoforceSensorCallback)

        # Publishers
        self.sensorFilteredPub = rospy.Publisher('/optoforce_node/OptoForceWrench', WrenchStamped, queue_size = 1)

        self.sensorReadings = []
        self.readingCount = 0

        self.sensorReadingAvaraged = WrenchStamped()
        self.sensorReadingAvaraged_old = WrenchStamped()
        self.filterCoef = 1

        # median filter
        self.med_window = 6
        self.med_Msg = WrenchStamped()
        self.med_Fx = []
        self.med_Fy = []
        self.med_Fz = []
        self.med_Tx = []
        self.med_Ty = []
        self.med_Tz = []




    def medianFilter(self):
        
        
        if (len(self.med_Fx) >= 10):

            temp_Fx = np.array(self.med_Fx)
            temp_Fx = np.sort(temp_Fx)
            temp_Fy = np.array(self.med_Fy)
            temp_Fy = np.sort(temp_Fy)
            temp_Fz = np.array(self.med_Fz)
            temp_Fz = np.sort(temp_Fz)
            temp_Tx = np.array(self.med_Tx)
            temp_Tx = np.sort(temp_Tx)
            temp_Ty = np.array(self.med_Ty)
            temp_Ty = np.sort(temp_Ty)
            temp_Tz = np.array(self.med_Tz)
            temp_Tz = np.sort(temp_Tz)

            retMsg = WrenchStamped()

            retMsg.header.stamp = rospy.Time.now()
            retMsg.wrench.force.x = temp_Fx[2]
            retMsg.wrench.force.y = temp_Fy[2]
            retMsg.wrench.force.z = temp_Fz[2]
            retMsg.wrench.torque.x = temp_Tx[2]
            retMsg.wrench.torque.y = temp_Ty[2]
            retMsg.wrench.torque.z = temp_Tz[2]


            self.med_Fx.pop(0)
            self.med_Fy.pop(0)
            self.med_Fz.pop(0)
            self.med_Tx.pop(0)
            self.med_Ty.pop(0)
            self.med_Tz.pop(0)

            return retMsg

        else:        
            return self.med_Msg

    def OptoforceSensorCallback(self, msg):
        
        self.med_Msg = msg

        self.med_Fx.append(msg.wrench.force.x)
        self.med_Fy.append(msg.wrench.force.y)
        self.med_Fz.append(msg.wrench.force.z)
        self.med_Tx.append(msg.wrench.torque.x)
        self.med_Ty.append(msg.wrench.torque.y)
        self.med_Tz.append(msg.wrench.torque.z)

        returnValue = self.medianFilter()


        self.sensorFilteredPub.publish(returnValue)


        


    def run(self):
        
        while not rospy.is_shutdown():

            #print "running"
            rospy.sleep(0.001)


if __name__ == '__main__':

    rospy.init_node('SensorMedianFilter')
    node = sensorMedianFilter()

    node.run()
