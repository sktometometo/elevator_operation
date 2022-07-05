#!/usr/bin/env python

import rospy
from spinal.msg import Barometer
from std_msgs.msg import Float32


class BarometerToAltitude:

    def __init__(self):

        self.pub = rospy.Publisher('~output', Float32, queue_size=1)
        self.sub = rospy.Subscriber('~input', Barometer, self.callback)

    def callback(self, msg):

        output_msg = Float32
        output_msg.data = msg.altitude
        self.pub.publish(output_msg)


if __name__ == '__main__':

    rospy.init_node('barometer_to_altitude')
    node = BarometerToAltitude()
    rospy.spin()
