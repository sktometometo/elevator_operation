#!/usr/bin/env python

import rospy
from spinal.msg import Imu
from std_msgs.msg import Float32


class LowpassFilterNode(object):

    #
    # See https://qiita.com/yuji0001/items/b0bf121fb8b912c02856
    #

    def __init__(self):

        self.freq_cutoff = rospy.get_param('~freq_cutoff')
        self.pre_time = None
        self.pre_acc_z = None
        self.pub = rospy.Publisher('~output', Float32, queue_size=1)
        self.sub = rospy.Subscriber('/spinal/imu', Imu, self.callback)

    def callback(self, msg):

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


if __name__ == '__main__':

    rospy.init_node('lowpass_filter')
    node = LowpassFilterNode()
    rospy.spin()
