#!/usr/bin/env python

import actionlib
import rospy

import math

from switchbot_ros.msg import SwitchBotCommandAction
from sensor_msgs.msg import PointCloud2
from spinal.msg import Barometer
from spinal.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float32


class ElevatorOperationServer(object):

    def __init__(self):

        # parameters for door opening checker
        self.threshold_door_points = rospy.get_param(
            '~threshold_door_points', 10)
        # variables for door opening checker
        self.door_is_open = False

        # parameters for elevator floor detection
        self.threshold_altitude = rospy.get_param('~threshold_altitude', 2)
        self.initial_elevator_floor = rospy.get_param(
            '~initial_elevator_floor', 7)
        self.altitude_per_elevator_floor = rospy.get_param(
            '~altitude_per_floor', 7)
        self.initial_altitude = rospy.wait_for_message(
            '~input_barometer', Barometer, timeout=rospy.Duration(30)).altitude
        # variables for elevator floor detection
        self.current_elevator_floor = self.initial_elevator_floor

        # parameters for floor moving detection
        self.threshold_accel = rospy.get_param('~threshold_accel', 0.2)
        self.initial_accel = rospy.wait_for_message(
            '~input_accel', Float32, timeout=rospy.Duration(30)).data
        # variables for floor moving detection
        self.elevator_state = 'halt'
        self.rest_elevator = False

        # ROS Action client
        self.action_client_switchbot = actionlib.SimpleActionClient(
            '/switchbot_ros/switch',
            SwitchBotCommandAction
        )

        # ROS Publisher
        self.pub_door_is_open = rospy.Publisher('~door_is_open', Bool, queue_size=1)
        self.pub_current_elevator_floor = rospy.Publisher('~current_elevator_floor', Int16, queue_size=1)
        self.pub_rest_elevator = rospy.Publisher('~rest_elevator', Bool, queue_size=1)

        # ROS Subscribers
        self.subscriber_door_points = rospy.Subscriber(
            '~input_door_point_checker',
            Int64,
            self.callback_door_points)

        self.subscriber_barometer = rospy.Subscriber(
            '~input_barometer',
            Barometer,
            self.callback_barometer)

        self.subscriber_imu = rospy.Subscriber(
            '~input_accel',
            Float32,
            self.callback_imu)

        rospy.loginfo('initialized')

    def callback_door_points(self, msg):

        rospy.logdebug('door points: {}'.format(msg.data))
        if msg.data < self.threshold_door_points:
            self.door_is_open = True
        else:
            self.door_is_open = False
        self.pub_door_is_open.publish(Bool(data=self.door_is_open))

    def callback_barometer(self, msg):

        rospy.logdebug('altitude: {}'.format(msg.altitude))
        current_altitude = msg.altitude
        self.current_elevator_floor = self.initial_elevator_floor + \
            int((current_altitude - self.initial_altitude) /
                self.altitude_per_elevator_floor)
        self.pub_current_elevator_floor.publish(Int16(data=self.current_elevator_floor))

    def callback_imu(self, msg):

        rospy.logdebug('acc z: {}'.format(msg.data))
        if math.fabs(msg.data - self.initial_accel) < self.threshold_accel:
            self.rest_elevator = True
        else:
            self.rest_elevator = False
        self.pub_rest_elevator.publish(Bool(data=self.rest_elevator))


if __name__ == '__main__':

    rospy.init_node('elevator_operation_server')
    node = ElevatorOperationServer()
    rospy.spin()
