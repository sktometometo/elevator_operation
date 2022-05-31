#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger, TriggerResponse


class StoreBaseFrame(object):

    def __init__(self):

        self._fixed_frame_id = rospy.get_param('~fixed_frame_id', 'odom')
        self._base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self._stored_frame_id = rospy.get_param('~stored_frame_id', 'stored_frame')

        self._transform_fixed_to_stored = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._br = tf2_ros.TransformBroadcaster()
        self._srv = rospy.Service('~store', Trigger, self.handler)

    def spin(self):

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
            if self._transform_fixed_to_stored is not None:
                self._transform_fixed_to_stored.header.stamp = rospy.Time.now()
                self._br.sendTransform(self._transform_fixed_to_stored)

    def handler(self, req):

        try:
            transform = self._tf_buffer.lookup_transform(
                    self._base_frame_id,
                    self._fixed_frame_id,
                    rospy.Time(),
                    rospy.Duration(1.0)
                )
            self._transform_fixed_to_stored = transform
            res = TriggerResponse()
            res.success = True
            return res
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._transform_fixed_to_stored = None
            res = TriggerResponse()
            res.success = False
            res.message = '{}'.format(e)
            return res


if __name__ == '__main__':
    rospy.init_node('store_base_frame')
    node = StoreBaseFrame()
    node.spin()
