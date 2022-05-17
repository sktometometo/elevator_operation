#!/usr/bin/env python

import math

import actionlib
import rospy
from swtichbot_ros.switchbot_ros_client import SwitchBotROSClient

from spinal.msg import Barometer
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import String
from std_msgs.msg import Float32

from std_srvs.srv import Trigger

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from elevator_operation.msg import MoveElevatorAction
from elevator_operation.msg import MoveElevatorResult


class ElevatorOperationServer(object):

    def __init__(self):

        #######################################################################
        # Elevator Operation State
        #######################################################################
        self.current_floor = rospy.get_param('~initial_floor', 7)

        #######################################################################
        # Elevator Configuration
        #######################################################################
        config = rospy.get_param('~elevator_configuration', [])
        self.elevator_configuration = {
            entry['floor']: entry for entry in config}

        #######################################################################
        # Door Openinig Checker
        #######################################################################
        # variables for door opening checker
        self.door_is_open = False
        # parameters for door opening checker
        self.threshold_door_points = rospy.get_param(
            '~threshold_door_points', 10)

        #######################################################################
        # Elevator Floor Detection
        #######################################################################
        # parameters for elevator floor detection
        self.threshold_altitude = rospy.get_param('~threshold_altitude', 2)
        # variables for floor detection
        self.elevator_floor = self.current_floor
        self.initial_elevator_floor = 0
        self.initial_altitude = 0
        # initialize
        self._reset_initial_altitude(self.current_floor)

        #######################################################################
        # Elevator Moving Detection
        #######################################################################
        # variables for floor moving detection
        self.elevator_state = 'halt'
        self.rest_elevator = False
        # parameters for floor moving detection
        self.threshold_accel = rospy.get_param('~threshold_accel', 0.2)
        self.initial_accel = rospy.wait_for_message(
            '~input_accel', Float32, timeout=rospy.Duration(30)).data

        #######################################################################
        # SwitchBot ROS Client
        #######################################################################
        self.switchbot_ros_client = SwitchBotROSClient()

        #######################################################################
        # Look At server client
        #######################################################################
        self.look_at_client = rospy.ServiceProxy('~look_at', Trigger)

        #######################################################################
        # Move Base client
        #######################################################################
        self.move_base_client = actionlib.SimpleActionClient(
            '/move_base',
            MoveBaseAction
        )

        #######################################################################
        # change floor client
        #######################################################################
        self.pub_change_floor = rospy.Publisher(
            '~change_floor', String, queue_size=1)

        #######################################################################
        # ROS Publisher
        #######################################################################
        self.pub_door_is_open = rospy.Publisher(
            '~door_is_open', Bool, queue_size=1)
        self.pub_current_elevator_floor = rospy.Publisher(
            '~current_elevator_floor', Int16, queue_size=1)
        self.pub_rest_elevator = rospy.Publisher(
            '~rest_elevator', Bool, queue_size=1)

        #######################################################################
        # ROS Subscribers
        #######################################################################
        self.subscriber_door_points = rospy.Subscriber(
            '~input_door_point_checker',
            Int64,
            self._callback_door_points)
        self.subscriber_barometer = rospy.Subscriber(
            '~input_barometer',
            Barometer,
            self._callback_barometer)
        self.subscriber_imu = rospy.Subscriber(
            '~input_accel',
            Float32,
            self._callback_imu)

        #######################################################################
        # ROS Action Server
        #######################################################################
        self.action_server = actionlib.SimpleActionServer(
                '~move_elevator',
                MoveElevatorAction,
                self.execute_cb,
                auto_start=False
                )
        self.action_server.start()

        rospy.loginfo('initialized')

    def execute_cb(self, goal):
        self.move_elevator.move_elevator(goal.target_floor)
        result = MoveElevatorResult()
        result.success = True
        self.action_server.set_succeeded(result)

    def move_elevator(self, target_floor):

        # move robot to the front of elevator
        self._move_to(
            self.elevator_configuration[self.current_floor]['outside_pose']['position'],
            self.elevator_configuration[self.current_floor]['outside_pose']['orientation']
        )

        # look to the door
        self.look_at_client()

        # Reset altitude
        self._reset_initial_altitude(self.current_floor)

        # Call elevator
        if target_floor > self.current_floor:
            button_type = 'up'
        else:
            button_type = 'down'
        self.switchbot_ros_client.control_device(
            self.elevator_configuration[self.current_floor]['buttons'][button_type],
            'press'
        )

        # Wait until arrive
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('waiting')
            if self.door_is_open:
                break

        # Ride on when arrive
        self._move_to(
            self.elevator_configuration[self.current_floor]['inside_pose']['position'],
            self.elevator_configuration[self.current_floor]['inside_pose']['orientation']
        )

        # Call elevator from target floor
        if target_floor < self.current_floor:
            button_type = 'up'
        else:
            button_type = 'down'
        self.switchbot_ros_client.control_device(
            self.elevator_configuration[target_floor]['buttons'][button_type],
            'press'
        )

        # Change map
        self.pub_change_floor(
            String(data=self.elevator_configuration[target_floor]['map_name']))

        # look to the door
        self.look_at_client()

        # Wait until arrive
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.loginfo('door: {}, floor: {}, rest: {}'.format(
                self.door_is_open, self.elevator_floor, self.rest_elevator))
            if self.door_is_open \
                    and self.elevator_floor == target_floor \
                    and self.rest_elevator:
                break

        # Get off when arrive
        self._move_to(
            self.elevator_configuration[target_floor]['outside_pose']['position'],
            self.elevator_configuration[target_floor]['outside_pose']['orientation']
        )

        self.current_floor = target_floor

        rospy.loginfo('Finished.')

    def _move_to(self, position, orientation, frame_id='map'):

        goal = MoveBaseGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = position[2]
        goal.pose.orientation.x = orientation[0]
        goal.pose.orientation.y = orientation[1]
        goal.pose.orientation.z = orientation[2]
        goal.pose.orientation.w = orientation[3]
        self.move_base_client.send_goal_and_wait(goal)
        return self.move_base_client.get_result()

    def _reset_initial_altitude(self, floor):
        self.initial_elevator_floor = floor
        self.initial_altitude = rospy.wait_for_message(
            '~input_barometer', Barometer, timeout=rospy.Duration(30)).altitude

    def _callback_door_points(self, msg):

        rospy.logdebug('door points: {}'.format(msg.data))
        if msg.data < self.threshold_door_points:
            self.door_is_open = True
        else:
            self.door_is_open = False
        self.pub_door_is_open.publish(Bool(data=self.door_is_open))

    def _callback_barometer(self, msg):

        rospy.logdebug('altitude: {}'.format(msg.altitude))
        altitude_diff_list = [
            {
                'floor': key,
                'altitude_diff':
                (entry['altitude'] - self.initial_altitude) -
                (msg.altitude - self.initial_altitude)
            }
            for key, entry in self.elevator_configuration.items()]
        nearest_entry = min(
            altitude_diff_list,
            key=lambda x: math.fabs(x['altitude_diff'])
        )
        rospy.logdebug('nearest_entry: {}'.format(nearest_entry))
        if math.fabs(nearest_entry['altitude_diff']) < self.threshold_altitude:
            self.elevator_floor = nearest_entry['floor']
            self.pub_current_elevator_floor.publish(
                Int16(data=self.elevator_floor))

    def _callback_imu(self, msg):

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
