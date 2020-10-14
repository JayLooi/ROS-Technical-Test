#!/usr/bin/env python

import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        br.sendTransform((6.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), 'map-1', 'map')
        br.sendTransform((-6.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), 'map-2', 'map')
        rate.sleep()

