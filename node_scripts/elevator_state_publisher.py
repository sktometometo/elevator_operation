#!/usr/bin/env python

import math

import rospy

from spinal.msg import Barometer
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32


class ElevatorStatePublisher(object):

    def __init__(self):

        self.pub_current_floor = rospy.Publisher('~current_floor', Int16, queue_size=1)
        self.pub_elevator_movement = rospy.Publisher('~elevator_movement', String, queue_size=1)

        self.pub_change_floor = rospy.Publisher('~change_floor', String, queue_size=1)
        self._start_subscriber()

        elevator_config = rospy.get_param('~elevator_config', [])
        initial_floor = rospy.get_param('~initial_floor', 7)
        threshold_altitude = rospy.get_param('~threshold_altitude', 2)
        threshold_accel = rospy.get_param('~threshold_accel', 0.2)

        self.elevator_config = {entry['floor']: entry for entry in elevator_config}

        self.state_current_floor = initial_floor
        self.state_elevator_movement = 'halt'
        self.prestate_elevator_movement = 'halt'

        self.param_threshold_altitude = threshold_altitude
        self.param_threshold_accel = threshold_accel
        self.param_anchor_floor = initial_floor
        self.param_anchor_altitude = self.state_altitude
        self.param_anchor_time = rospy.Time.now()
        self.param_stable_accel = self.state_acc

    def spin(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

            # update elevator movement state
            acc_diff = self.state_acc - self.state_stable_accel
            if self.state_elevator_movement == 'halt':
                if acc_diff > self.threshold_accel:
                    self.state_elevator_movement = 'up_accel'
                elif acc_diff < - self.threshold_accel:
                    self.state_elevator_movement = 'down_accel'
            elif self.state_elevator_movement == 'up_accel':
                if acc_diff <= self.threshold_accel:
                    self.state_elevator_movement = 'up_stable'
            elif self.state_elevator_movement == 'up_stable':
                if acc_diff < - self.threshold_accel:
                    self.state_elevator_movement = 'up_decel'
            elif self.state_elevator_movement == 'up_decel':
                if acc_diff >= - self.threshold_accel:
                    self.state_elevator_movement = 'halt'
            elif self.state_elevator_movement == 'down_accel':
                if acc_diff >= - self.threshold_accel:
                    self.state_elevator_movement = 'down_stable'
            elif self.state_elevator_movement == 'down_stable':
                if acc_diff > self.threshold_accel:
                    self.state_elevator_movement = 'down_decel'
            elif self.state_elevator_movement == 'down_decel':
                if acc_diff <= self.threshold_accel:
                    self.state_elevator_movement = 'halt'

            # update current floor state
            altitude_diff_list = [
                {
                    'floor': key,
                    'altitude_diff':
                    (entry['altitude'] - self.elevator_config[self.param_anchor_floor]['altitude']) - (self.state_altitude - self.param_anchor_altitude)
                }
                for key, entry in self.elevator_configuration.items()]
            nearest_entry = min(
                altitude_diff_list,
                key=lambda x: math.fabs(x['altitude_diff'])
            )
            if math.fabs(nearest_entry['altitude_diff']) < self.threshold_altitude:
                self.state_current_floor = nearest_entry['floor']

            # change floor if state is changed to halt
            if self.prestate_elevator_movement != 'halt' and \
                    self.state_elevator_movement == 'halt':
                self.pub_change_floor.publish(String(data=self.elevator_config[self.state_current_floor]['map_name']))

            # update anchor if state is changed to halt
            if self.prestate_elevator_movement != 'halt' and \
                    self.state_elevator_movement == 'halt':
                self._update_anchor_floor_and_altitude()

            # update anchor if altitude is not changed
            if rospy.Time.now() - self.param_anchor_time > rospy.Duration(5) and \
                    math.fabs(self.state_altitude - self.param_anchor_altitude) < self.param_threshold_altitude:
                self._update_anchor_floor_and_altitude()

            # store state to prestate
            self.prestate_elevator_movement = self.state_elevator_movement

    def _update_anchor_floor_and_altitude(self):

        self.param_anchor_floor = self.state_current_floor
        self.param_anchor_altitude = self.state_altitude
        self.param_anchor_time = rospy.Time.now()

    def _start_subscriber(self):

        self.subscriber_barometer = rospy.Subscriber(
            '~input_barometer',
            Barometer,
            self._callback_barometer)
        self.subscriber_imu = rospy.Subscriber(
            '~input_accel',
            Float32,
            self._callback_imu)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown() \
                and self.state_altitude is not None \
                and self.state_acc is not None:
            rate.sleep()

    def _callback_barometer(self, msg):

        self.state_altitude = msg.altitude

    def _callback_imu(self, msg):

        self.state_acc = msg.data
