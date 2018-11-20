#!/usr/bin/env python
import rospy
import allantools
import allan_variance
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import nnls
from collections import OrderedDict
import pandas as pd
import csv
#import lowpass
from scipy.signal import butter, lfilter, freqz
import pywt
from statsmodels.robust import mad
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from scipy import integrate


class Imu_Filter():
    def __init__(self,topic = '/imu'):
        self.linear_accs = [[],[],[]]
        self.gyro = [[],[],[]]
        self.t = []
        imu_sub = rospy.Subscriber(topic, Imu, self.imu_callback)

        self.imu_pub = rospy.Publisher('/imu/filtered', Imu, queue_size=10)
        self.imu_pub_pose = rospy.Publisher('/imu/filtered/pose', Imu, queue_size=10)
        self.pose_pub = rospy.Publisher('/imu_pose', Pose, queue_size=10)
        self.pose_pub_unfilter = rospy.Publisher('/unfiltered_imu_pose', Pose, queue_size=10)

        rospy.sleep(10)

        self.xavg = np.average(self.linear_accs[0])
        self.yavg = np.average(self.linear_accs[1])
        self.zavg = np.average(self.linear_accs[2])

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.x_unfiltered = 0.0
        self.y_unfiltered = 0.0
        self.z_unfiltered = 0.0


    def imu_callback(self, data):
        # Add linear accelerations to array for later processing
        self.linear_accs[0].append(data.linear_acceleration.x)
        self.linear_accs[1].append(data.linear_acceleration.y)
        self.linear_accs[2].append(data.linear_acceleration.z)

        self.t.append(rospy.get_rostime().secs)

        # self.filter()


    def filter(self):

        len_acc = len(self.linear_accs[0])
        # Filter x coordinate
        filtered_x = self.wavelet_filter(self.linear_accs[0][-200:len_acc], 'db38',6)
        filtered_x[:] = [h - self.xavg for h in filtered_x]
        # Filter y coordinate
        filtered_y = self.wavelet_filter(self.linear_accs[1][-200:len_acc], 'db38',6)
        filtered_y[:] = [i - self.yavg for i in filtered_y]
        # Filter z coordinate
        filtered_z = self.wavelet_filter(self.linear_accs[2][-200:len_acc], 'db38',6)
        filtered_z[:] = [j - self.zavg for j in filtered_z]

        dx = self.trapz(filtered_x[-6:len(filtered_x)],self.t[-6:len(self.t)])
        dy = self.trapz(filtered_y[-6:len(filtered_x)],self.t[-6:len(self.t)])
        dz = self.trapz(filtered_z[-6:len(filtered_x)],self.t[-6:len(self.t)])

        self.x += dx[len(dx)-1]
        self.y += dy[len(dy)-1]
        self.z += dz[len(dz)-1]

        #self.imu_pub.Header.header.stamp = rospy.Time.now()
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = filtered_x[len(filtered_x)-1]-self.xavg
        imu_msg.linear_acceleration.y = filtered_y[len(filtered_y)-1]-self.yavg
        imu_msg.linear_acceleration.z = filtered_z[len(filtered_z)-1]-self.zavg

        self.imu_pub.publish(imu_msg)

        #self.imu_pub.Header.header.stamp = rospy.Time.now()
        imu_msg_pose = Imu()
        imu_msg_pose.header.stamp = rospy.Time.now()
        imu_msg_pose.linear_acceleration.x = dx[len(dx)-1]
        imu_msg_pose.linear_acceleration.y = dy[len(dy)-1]
        imu_msg_pose.linear_acceleration.z = dz[len(dz)-1]

        self.imu_pub_pose.publish(imu_msg_pose)

        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = self.z

        self.pose_pub.publish(pose_msg)

        # Same with unfiltered to check
        dxuf = self.trapz(self.linear_accs[0][-6:len_acc],self.t[-6:len(self.t)])
        dyuf = self.trapz(self.linear_accs[1][-6:len_acc],self.t[-6:len(self.t)])
        dzuf = self.trapz(self.linear_accs[2][-6:len_acc],self.t[-6:len(self.t)])

        self.x_unfiltered += dxuf[len(dxuf)-1]
        self.y_unfiltered += dyuf[len(dyuf)-1]
        self.z_unfiltered += dzuf[len(dzuf)-1]

        pose_msg_uf = Pose()
        pose_msg_uf.position.x = self.x_unfiltered
        pose_msg_uf.position.y = self.y_unfiltered
        pose_msg_uf.position.z = self.z_unfiltered

        self.pose_pub_unfilter.publish(pose_msg_uf)



    def wavelet_filter(self, x, wave, level):
        # Calculate wavelet coefficients
        x_coeffs = pywt.wavedec(x, wave, level=level , mode = 'per')
        # Calculate a threshold
        sigma = mad(x_coeffs[-level])
        # changing this threshold also changes the behavior,
        uthresh = sigma * np.sqrt( 2*np.log( len( x ) ) )
        x_coeffs[1:] = ( pywt.threshold( i, value=uthresh, mode="soft" )
                        for i in x_coeffs[1:] )
        # reconstruct the signal using the thresholded coefficients
        w = pywt.waverec( x_coeffs, wave, mode="per" )

        # return filtered signal
        return w
    

    def trapz(self,x,t):
        '''integrate twice'''
        if len(x)!=len(t):
            if len(x)<len(t):
                l = len(x)
            else:
                l = len(t)
        else:
            l = len(t)

        print 'x=', x[-l:len(x)]
        print 't=', t[-l:len(t)]
        dt = integrate.cumtrapz(x[-l:len(x)],t[-l:len(t)])
        print 'dt = ', dt
        ddt = integrate.cumtrapz(dt,t[-l+1:len(t)])
        print 'ddt = ', ddt

        return ddt



if __name__=='__main__':
    rospy.init_node('imu_filter_wav',anonymous = True)
    iwf = Imu_Filter(topic='/mavros/imu/data')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep() # sleeps for 1 sec
        iwf.filter()
