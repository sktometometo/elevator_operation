#!/usr/bin/env python

import actionlib
import roslaunch
import rospkg
import rospy

import math
import threading

from switchbot_ros.msg import SwitchBotCommandAction
from sensor_msgs.msg import PointCloud2
from spinal.msg import Barometer
from spinal.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class ElevatorOperationServer(object):

    def __init__(self):

        # variables for roslaunch
        self.roslaunch_parent = None
        self.lock_roslaunch_parent = threading.Lock()
        self.input_topic_points = rospy.get_param('~input_topic_points')
        self.elevator_door_frame_id = rospy.get_param('~elevator_door_frame_id')

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
            '~input_barometer', Barometer, timeout=rospy.Duration(5))
        # variables for elevator floor detection
        self.current_elevator_floor = self.initial_elevator_floor

        # parameters for floor moving detection
        self.threshold_accel = rospy.get_param('~threshold_accel', 0.2)
        self.stable_accel = rospy.get_param('~stable_accel', 0.0)
        # variables for floor moving detection
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

        # ROS Service
        self.service_start_detection = rospy.Service(
            '~start_detection',
            Trigger,
            self.handler_start_detection)
        self.service_stop_detection = rospy.Service(
            '~stop_detection',
            Trigger,
            self.handler_stop_detection)

        # ROS Subscribers
        self.subscriber_door_points = rospy.Subscriber(
            '~input_door_points',
            PointCloud2,
            self.callback_door_points)

        self.subscriber_barometer = rospy.Subscriber(
            '~input_barometer',
            Barometer,
            self.callback_barometer)

        self.subscriber_imu = rospy.Subscriber(
            '~input_imu',
            Imu,
            self.callback_imu)

    def callback_door_points(self, msg):

        rospy.logdebug('door points: {}'.format(len(msg.data)))
        if len(msg.data) < self.threshold_door_points:
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

        rospy.logdebug('acc z: {}'.format(msg.acc_data[2]))
        if math.fabs(msg.acc_data[2] - self.stable_accel) < self.threshold_accel:
            self.rest_elevator = True
        else:
            self.rest_elevator = False
        self.pub_rest_elevator.publish(Bool(data=self.rest_elevator))

    def handler_start_detection(self, req):

        success, message = self._start_detection_launch()
        return TriggerResponse(success, message)

    def handler_stop_detection(self, req):

        success, message = self._stop_detection_launch()
        return TriggerResponse(success, message)

    def _start_detection_launch(self):

        with self.lock_roslaunch_parent:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
            roslaunch_path = \
                rospkg.RosPack().get_path('elevator_operation') +\
                '/launch/elevator_detection.launch'
            roslaunch_cli_args = [
                    roslaunch_path,
                    'input_topic_points:={}'.format(self.input_topic_points),
                    'elevator_door_frame_id:={}'.format(self.elevator_door_frame_id),
                    ]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
                roslaunch_cli_args)
            self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
                uuid,
                roslaunch_file
            )
            self.roslaunch_parent.start()

        return True, 'success'

    def _stop_detection_launch(self):

        with self.lock_roslaunch_parent:
            self.roslaunch_parent.shutdown()
            self.roslaunch_parent = None

        return True, 'success'
