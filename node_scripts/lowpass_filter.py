#!/usr/bin/env python

import rospy
from spinal.msg import Imu
from std_msgs.msg import Float32
import scipy
from scipy import signal


class LowpassFilterNode(object):

    #
    # See https://qiita.com/yuji0001/items/b0bf121fb8b912c02856
    #

    def __init__(self):

        self.filter_order = rospy.get_param('~filter_order', 3)
        self.freq_cutoff = rospy.get_param('~freq_cutoff', 0.1)
        self.buffer_length = rospy.get_param('~buffer_length', 100)
        self.buffer = list()
        self.buffer_stamp = list()
        self.pre_time = None
        self.pre_acc_z = None
        self.initialized = False
        self.pub = rospy.Publisher('~output', Float32, queue_size=1)
        self.sub = rospy.Subscriber('/spinal/imu', Imu, self.callback)

    def callback_old(self, msg):

        if self.pre_acc_z is None or self.pre_time is None:
            self.pre_acc_z = msg.acc_data[2]
            self.pre_time = msg.stamp
        else:
            dt = (msg.stamp - self.pre_time).to_sec()
            fs = 1.0 / dt
            rospy.loginfo('fs: {}'.format(fs))
            self.pre_acc_z = (1 - (self.freq_cutoff / fs)) * self.pre_acc_z \
                + (self.freq_cutoff / fs) * msg.acc_data[2]
            self.pre_time = msg.stamp

        self.pub.publish(Float32(data=self.pre_acc_z))

    def callback(self, msg):

        self.buffer.append(msg.acc_data[2])
        self.buffer_stamp.append(msg.stamp)
        if len(self.buffer) > self.buffer_length:
            self.buffer.pop(0)
            self.buffer_stamp.pop(0)
            if not self.initialized:
                # initialize filter
                duration = 0
                for i in range(len(self.buffer_stamp)-1):
                    duration += (self.buffer_stamp[i+1] - self.buffer_stamp[i]).to_sec() / (len(self.buffer_stamp)-1)
                self.fs = 1.0 / duration
                b, a = signal.butter(self.filter_order, self.freq_cutoff, btype='lowpass', fs=self.fs)
                self.filter_a = a
                self.filter_b = b
                self.initialized = True
        if self.initialized:
            y = signal.filtfilt(
                    self.filter_b,
                    self.filter_a,
                    self.buffer
                    )
            self.pub.publish(Float32(data=y[-1]))


if __name__ == '__main__':

    rospy.init_node('lowpass_filter')
    node = LowpassFilterNode()
    rospy.spin()
