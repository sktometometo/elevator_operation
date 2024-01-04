#!/usr/bin/env python


import rospy
import numpy as np

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


const_G = 9.80665


def imu2vector(msg):
    return np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z])

class M5StackAccelExtractor:

    def __init__(self):
        self._pub_accel = rospy.Publisher("/elevator_accel", Float32, queue_size=1)
        self._sub_imu = rospy.Subscriber("~imu", Imu, self._callback_imu)
        self._initialization_buffer_length = rospy.get_param('~initialization_buffer_length', 10)
        self._initialized = False
        self._scale_factor = 1.0
        self._gravity_direction = np.array([0, 0, 1.0])
        self._gravity = np.array([0, 0, 9.8])
        self._buffer = []

    def extract(self, imu_vector):
        return np.dot(self._scale_factor * imu_vector, self._gravity_direction)

    def _callback_imu(self, msg):
        imu_vector = imu2vector(msg)
        if not self._initialized:
            self._buffer.append(imu_vector)
            rospy.logwarn("storing buffer: {}".format(len(self._buffer)))
            if len(self._buffer) > self._initialization_buffer_length:
                average_vector = np.average(self._buffer, axis=0)
                self._scale_factor = const_G / np.linalg.norm(average_vector)
                self._gravity = self._scale_factor * average_vector
                self._gravity_direction = self._gravity / np.linalg.norm(self._gravity)
                self._buffer = []
                self._initialized = True
        else:
            elevator_accel = self.extract(imu_vector)
            msg_ea = Float32()
            msg_ea.data = elevator_accel
            self._pub_accel.publish(msg_ea)


if __name__ == '__main__':
    rospy.init_node('m5stack_accel_extractor')
    node = M5StackAccelExtractor()
    rospy.spin()
