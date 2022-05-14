#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64


class ElevatorDoorOpeningCheckerNode(object):

    def __init__(self):

        self.pub = rospy.Publisher('~output', Int64, queue_size=1)
        self.sub = rospy.Subscriber('~input', PointCloud2, self.callback)

    def callback(self, msg):

        rospy.logdebug('door points: {}'.format(len(msg.data)))
        self.pub.publish(Int64(data=len(msg.data)))


if __name__ == '__main__':

    rospy.init_node('elevator_door_opening_checker_node')
    node = ElevatorDoorOpeningCheckerNode()
    rospy.spin()
