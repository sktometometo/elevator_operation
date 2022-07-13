#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse
import message_filters


class AltitudeCalculator(object):

    def __init__(self):

        self.pub = rospy.Publisher('~output', Float32, queue_size=1)

        sub_pressure = message_filters.Subscriber('~input_pressure', Float32)
        sub_temparature = message_filters.Subscriber('~input_temperature', Float32)

        msg_pressure = rospy.wait_for_message('~input_pressure', Float32)
        self.p_b = msg_pressure.data
        self.altitude_b = 0

        self.srv = rospy.Service('~reset_altitude', Trigger, self.handler)

        # J K^-1 mol^-1
        self.gas_constant = rospy.get_param('~gas_const', 8.31446261815324)
        # m sec^-2
        self.grav_accel = rospy.get_param('~gravitational_accel', 9.8)

        slop = rospy.get_param('~slop', 0.1)
        self.ts = message_filters.ApproximateTimeSynchronizer(
                [sub_pressure, sub_temparature],
                10,
                slop=slop,
                allow_headerless=True
                )

        rospy.loginfo('initialized')

    def handler(self, req):

        msg_pressure = rospy.wait_for_message('~input_pressure', Float32)
        self.p_b = msg_pressure.data
        self.altitude_b = 0
        return TriggerResponse(success=True)

    def callback(self, msg_pressure, msg_temp):

        p = msg_pressure.data
        T = msg_temp.data + 273
        altitude = altitude_b - ( self.gas_constant * T / self.grav_accel ) * math.log( p / self.p_b )
        msg = Float32()
        msg.data = altitude
        self.pub.publish(msg)


if __name__ == '__main__':

    rospy.init_node('altitude_calculator')
    node = AltitudeCalculator()
    rospy.spin()
