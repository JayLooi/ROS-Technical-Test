#!/usr/bin/env python

from __future__ import print_function
import argparse as ap

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Bool


class SetWaypoints:
    def __init__(self, num_of_waypoint, init_points=None):
        rospy.init_node('set_waypoints')
        self.num_of_waypoint = num_of_waypoint
        self.marker_server = InteractiveMarkerServer('waypoint_marker')
        self.init_points = init_points
        if self.init_points:
            if len(self.init_points) != self.num_of_waypoint:
                raise Exception('Length of init_points not equal to num_of_waypoint')
        
        self.waypoint_names = []
        self.waypoints = Polygon()
        self.is_lock_points = False
        self.waypoints_publisher = rospy.Publisher('/waypoints', Polygon, queue_size=1)
        self.lock_points_subscriber = rospy.Subscriber('/lock_points', Bool, callback=self._setPointsState, queue_size=1)
        self._renderWaypoints()
        rospy.Timer(rospy.Duration(1.0), self._pubWaypoints)
    
    def _createWaypoint(self, name, init_x, init_y):
        self.waypoint_names.append(name)
        waypoint = Point32()
        waypoint.x = init_x
        waypoint.y = init_y
        self.waypoints.points.append(waypoint)
        
        waypoint_imarker = InteractiveMarker()
        waypoint_imarker.header.frame_id = 'odom'
        waypoint_imarker.name = name
        waypoint_imarker.description = name
        waypoint_imarker.pose.position.x = init_x
        waypoint_imarker.pose.position.y = init_y
        
        marker = Marker()
        marker.type = Marker.SPHERE
        scale = marker.scale
        scale.x = scale.y = scale.z = 0.15
        color = marker.color
        color.r, color.g, color.b, color.a = (0.0, 0.0, 1.0, 1.0)
        
        move_control = InteractiveMarkerControl()
        move_control.name = 'move_xy'
        move_control.always_visible = True
        orientation = move_control.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = (0.0, 1.0, 0.0, 1.0)
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.markers.append(marker)
        
        waypoint_imarker.controls.append(move_control)
        self.marker_server.insert(waypoint_imarker, self._feedbackCallback)
        self.marker_server.applyChanges()
        
    def _feedbackCallback(self, feedback):
        pose = feedback.pose
        point = self.waypoints.points[self.name_to_index[feedback.marker_name]]
        if self.is_lock_points:
            pose.position.x = point.x
            pose.position.y = point.y
            self.marker_server.setPose(feedback.marker_name, pose)
            self.marker_server.applyChanges()
        
        else:
            point.x = pose.position.x
            point.y = pose.position.y
        
        #rospy.loginfo('%s position: (%f, %f, %f)' % (feedback.marker_name, position.x, position.y, position.z))
        
    def _pubWaypoints(self, event=None):
        if not self.is_lock_points:
            self.waypoints_publisher.publish(self.waypoints)
    
    def _renderWaypoints(self):
        from random import uniform
        if self.init_points is None:
            self.init_points = [(uniform(-5.0, 5.0), uniform(-5.0, 5.0)) for i in range(self.num_of_waypoint)]
        
        for i, (x, y) in enumerate(self.init_points):
            self._createWaypoint('Waypoint-%d' % i, x, y)
        
        self.name_to_index = {name: index for index, name in enumerate(self.waypoint_names)}
        self.waypoints_publisher.publish(self.waypoints)
        
    def _setPointsState(self, msg):
        self.is_lock_points = msg.data


if __name__ == '__main__':
    def positive_int(val):
        int_val = int(val)
        if int_val <= 0:
            raise ap.ArgumentTypeError('Invalid unsigned int value: %s' % val)
        
        return int_val                    
    
    parser = ap.ArgumentParser()
    parser.add_argument('num_of_waypoints', type=positive_int)
    args, unknown = parser.parse_known_args()
    set_waypoints = SetWaypoints(args.num_of_waypoints, [(2, 0), (-4, 0), (0, -4), (-1, 3), (4.5, 1)])
    rospy.spin()


