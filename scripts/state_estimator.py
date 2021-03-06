#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from z_state_estimator.msg import ZStateEst
from rosflight_msgs.msg import Attitude
from std_msgs.msg import Time
from geometry_msgs.msg import Vector3Stamped
import math
import numpy as np

class zStateEstimator:

    def __init__(self):
        # Define subscribers
        self.range_sub = rospy.Subscriber('/teraranger_evo', Range, self.range_cb, queue_size=10)
        self.attitude_sub = rospy.Subscriber('/attitude/euler', Vector3Stamped, self.attitude_cb, queue_size=10)

        # Define publishers
        self.z_state_pub = rospy.Publisher('/z_state_estimator/z_state_estimate', ZStateEst, queue_size=10)
        
        # Import Parameters
        self.h_filt_alpha = rospy.get_param('h_filt_alpha', .1)
        self.v_filt_alpha = rospy.get_param('v_filt_alpha', .1)
        self.range_variance = rospy.get_param('range_variance', .1)
       
        # Initialize atittude
        self.roll = 0.0
        self.pitch = 0.0
        self.tworot = np.identity(3)
        self.onerot = np.identity(3)

        # Initialize estimator variables
        self.h_init = False
        self.h_filt_last = 0.0
        self.h_filt = 0.0
        self.range_time_s_last = 0.0

        self.v_filt = 0.0
        self.v_filt_last = 0.0

        self.est_msg = ZStateEst()

        # Kalman Filter Matrices / Vectors
        self.P = np.full((3,3),0)
        self.xk = np.matrix([[0], [0], [0]])
        self.Q = np.matrix([[.001, 0, 0],[0, .001, 0],[0, 0, .01]])*.01
        self.R = np.matrix([[pow(self.range_variance,2)]])

        while not rospy.is_shutdown():
            rospy.spin()

    def range_cb(self, msg):
        range_val = msg.range
        if (range_val < 0):
            rospy.loginfo_throttle(5,'invalid range value, setting to min')
            range_val = .1
        elif (range_val > 3):
            rospy.loginfo_throttle(5,'invalid range value, setting to max')
            range_val = 3
        range_time = msg.header.stamp
        range_time_s = range_time.to_sec()
        dt = range_time_s - self.range_time_s_last
        self.range_time_s_last = range_time_s
        
        # Convert range_val to h_agl using roll / pitch values
        h_agl_vec = self.onerot*self.tworot*np.matrix([[0],[0],[range_val]])
        #h_agl = h_agl_vec.item(2)
        h_agl = range_val*math.cos(self.roll)*math.cos(self.pitch)
        print(h_agl)

        # Low pass filter the h_agl value
        # to remove some of the sensor noise.
        # This improves the velocity estimates greatly.
        self.h_filt = (1-self.h_filt_alpha)*self.h_filt_last + self.h_filt_alpha*h_agl
        self.h_filt_last = self.h_filt
 
        A = np.matrix([[1, 0, dt],[1, 0, 0],[1/dt, -1/dt, 0]])
        H = np.matrix([1, 0, dt])
        
        if(not self.h_init):
            self.h_init = True
            self.xk = np.matrix([[self.h_filt], [self.h_filt], [0.0]])
            self.P = np.full((3,3),0)
        
        # Predict Ahead
        xkp1 = A*self.xk
        Pkp1 = A*self.P*A.transpose() + self.Q; 
        
        # Compute Kalman Gain
        K = Pkp1*H.transpose()*np.linalg.inv(H*Pkp1*H.transpose() + self.R)
        
        # Update State Estimate
        self.xk = xkp1 + K*(self.h_filt - H*xkp1)
        
        # Update Error Covariance
        self.P = (np.identity(3) - K*H)*Pkp1

        # Filter the vertical velocity estimate
        self.v_filt = (1-self.v_filt_alpha)*self.v_filt_last + self.v_filt_alpha*self.xk.item(2)
        self.v_filt_last = self.v_filt
 
        # Publish the estimate
        self.est_msg.header.stamp = rospy.Time.now()
        self.est_msg.height_agl.data = self.xk.item(0)
        self.est_msg.z_velocity.data = self.v_filt
        self.z_state_pub.publish(self.est_msg)


    def attitude_cb(self, msg):
        self.roll = msg.vector.x
        self.pitch = msg.vector.y
        #self.tworot = np.matrix([[math.cos(pitch), 0, -sin(pitch)],[0, 1, 0],[sin(pitch), 0, cos(pitch)]])
        #self.onerot = np.matrix([[1, 0, 0],[0, cos(roll), sin(roll)],[0, -sin(roll), cos(roll)]])

if __name__ == '__main__':
    rospy.init_node('z_state_estimator')
    try:
        estimator = zStateEstimator()
    except:
        rospy.ROSInterruptException
    pass




