#!/usr/bin/env python
import rospy
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
import tf2_ros # https://gist.github.com/yukke42/b6f409930240f424f69b16eb6bc374b2
from statsmodels.robust import mad
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, TransformStamped
from scipy import integrate


class Imu_Filter():
    def __init__(self,topic = '/imu', base_frame = 'world', imu_frame = 'base_link'):
        # Unfiltered accelerations
        self.linear_accs = [[],[],[]]
        # Unfiltered rotations
        self.gyro = [[],[],[],[]]
        # Time array
        self.t = []
        # Frames
        self.base_frame = base_frame
        self.imu_frame = imu_frame
        # Filtered accelerations
        self.filtered_accs = [[],[],[]]

        imu_sub = rospy.Subscriber(topic, Imu, self.imu_callback)

        rospy.sleep(10)

        self.accs_avg = [[np.average(self.linear_accs[0])],
                         [np.average(self.linear_accs[1])],
                         [np.average(self.linear_accs[2])]]

        self.position = [[0.0],[0.0],[0.0]]

        self.position_unfiltered = [[0.0],[0.0],[0.0]]


    def imu_callback(self, data):
        # Add linear accelerations to array for later processing
        # self.linear_accs[0].append(data.linear_acceleration.x)
        # self.linear_accs[1].append(data.linear_acceleration.y)
        # self.linear_accs[2].append(data.linear_acceleration.z)

        self.linear_accs[0].append(data.linear_acceleration.y)
        self.linear_accs[1].append(-data.linear_acceleration.x)
        self.linear_accs[2].append(data.linear_acceleration.z)

        # Add quaternion orientation (directly)
        self.gyro[0].append(data.orientation.x)
        self.gyro[1].append(data.orientation.y)
        self.gyro[2].append(data.orientation.z)
        self.gyro[3].append(data.orientation.w)


        self.t.append(rospy.get_rostime().secs)

        # self.filter()


    def filter(self, wave, level):

        # Filter each dimension with the wavelet filter
        self.filtered_accs = self.wavelet_filter(self.linear_accs, wave, level)


    def integrate(self, accs, t, position):
        """ This function takes x,y and z coordinates, as well as time t,
        and calls the desired integration method"""

        dt = t[-6:len(t)]
        dposition = [[0.0],[0.0],[0.0]]

        for i in (0,1,len(accs)-1):
            x_array = self.double_trapz(accs[i][-6:len(accs[i])],dt)
            dposition[i][0] = x_array[len(x_array)-1]

        rospy.logdebug("Differential of position x: %s", dposition[0])
        rospy.logdebug("Position to add: %s", position[0])

        # Sum the new differential of position with absolute position
        position = np.add(position,dposition)
        return position



    def publish(self, pub, type, data):

        msg = type

        x = data[0][len(data[0])-1]
        y = data[1][len(data[1])-1]
        z = data[2][len(data[2])-1]

        ox = self.gyro[0][len(self.gyro[0])-1]
        oy = self.gyro[1][len(self.gyro[1])-1]
        oz = self.gyro[2][len(self.gyro[2])-1]
        ow = self.gyro[3][len(self.gyro[3])-1]

        if msg == Imu():
            msg.header.stamp = rospy.Time.now()
            msg.linear_acceleration.x = x
            msg.linear_acceleration.y = y
            msg.linear_acceleration.z = z
            
            msg.orientation.x = ox
            msg.orientation.y = oy
            msg.orientation.z = oz
            msg.orientation.w = ow

        elif msg == Pose():
            msg.position.x = x
            msg.position.y = y
            msg.position.z = z

            msg.orientation.x = ox
            msg.orientation.y = oy
            msg.orientation.z = oz
            msg.orientation.w = ow

        elif msg == TransformStamped():
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.base_frame
            msg.child_frame_id = self.imu_frame

            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = z

            msg.transform.rotation.x = ox
            msg.transform.rotation.y = oy
            msg.transform.rotation.z = oz
            msg.transform.rotation.w = ow

            pub.sendTransform(msg)

            return # Because tf doesnt use publish

        pub.publish(msg)


    def wavelet_filter(self, accs, wave, level):
        """ filters the three axis and substracts the average previously
        obtained for each axis """

        accs_coeffs = [[],[],[]]
        sigmas      = [[],[],[]]
        uthresh     = [[],[],[]]
        filtered    = [[],[],[]]

        for i in (0,1,len(accs)-1):
            # Calculate wavelet coefficients
            accs_coeffs[i] = pywt.wavedec(accs[i], wave, level=level , mode = 'per')
            # Calculate a threshold
            sigmas[i] = mad(accs_coeffs[i][-level])
            # changing this threshold also changes the behavior,
            uthresh[i] = sigmas[i] * np.sqrt( 2*np.log( len( accs[i] ) ) )
            accs_coeffs[i][1:] = ( pywt.threshold( j, value=uthresh[i], mode="soft" )
            for j in accs_coeffs[i][1:] )
            # reconstruct the signal using the thresholded coefficients
            filtered[i] = pywt.waverec( accs_coeffs[i], wave, mode="per" )
            filtered[i][:] = [h - self.accs_avg[i] for h in filtered[i]]

        return filtered


    def double_trapz(self,x,t):
        '''integrate twice'''
        lx = len(x)
        lt = len(t)

        if lx!=lt:
            if lx<lt:
                l = lx
            else:
                l = lt
        else:
            l = lt

        dt = integrate.cumtrapz(x[-l:lx],t[-l:lt])
        ddt = integrate.cumtrapz(dt,t[-l+1:lt])

        return ddt




if __name__=='__main__':
    rospy.init_node('imu_filter_wav',anonymous = True, log_level=rospy.DEBUG)

    imutopic = rospy.get_param('~imu_topic')
    rospy.logdebug("IMU TOPIC %s", imutopic)
    wav_mode = rospy.get_param('~wavelet_mode')
    rospy.logdebug("WAVELET FILTER MODE %s", wav_mode)
    wav_level = rospy.get_param('~wavelet_level')
    rospy.logdebug("WAVELET FILTER LEVEL %s", wav_level)
    base_frame = rospy.get_param('~base_frame')
    rospy.logdebug("BASE FRAME %s", base_frame)
    imu_frame = rospy.get_param('~imu_frame')
    rospy.logdebug("IMU FRAME %s", imu_frame)

    iwf = Imu_Filter(topic=imutopic, base_frame = base_frame, imu_frame = imu_frame)

    # Data publishers
    imu_filtered_pub    = rospy.Publisher('/imu_filtered', Imu, queue_size=10)
    pose_filtered_pub   = rospy.Publisher('/imu_pose', Pose, queue_size=10)
    pose_unfiltered_pub = rospy.Publisher('/unfiltered_imu_pose', Pose, queue_size=10)
    tf_pub              = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep() # sleeps for 1 sec

        # Filter the read accelerations
        iwf.filter(wav_mode,wav_level)
        # Publish filtered Imu
        iwf.publish(imu_filtered_pub, Imu(), iwf.filtered_accs)
        rospy.logdebug(" Filtered acc x example value: %f", iwf.filtered_accs[0][0])

        # Integrate and sum to obtain position
        iwf.position = iwf.integrate(iwf.filtered_accs,iwf.t, iwf.position)
        rospy.logdebug("Position x after filtering: %f", iwf.position[0][0])
        # Publish filtered pose
        iwf.publish(pose_filtered_pub, Pose(), iwf.position)
        # Publish tf with filtered pose
        iwf.publish(tf_pub, TransformStamped(), iwf.position)

        rospy.logdebug(" UNfiltered acc x example value: %f", iwf.linear_accs[0][0])
        # Integrate unfiltered accs
        iwf.position_unfiltered = iwf.integrate(iwf.linear_accs, iwf.t, iwf.position_unfiltered)
        rospy.logdebug("Unfiltered position x: %f", iwf.position_unfiltered[0][0])
        # Publish unfiltered pose
        iwf.publish(pose_unfiltered_pub, Pose(), iwf.position_unfiltered)
