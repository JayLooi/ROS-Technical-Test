#!/usr/bin/env python

from __future__ import print_function
import argparse as ap
import math

import rospy
from geometry_msgs.msg import Twist, Polygon, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from path_following.cfg import RobotControlParamsConfig


class PathFollow:
    def __init__(self, model_name):
        self.Kp_lin = rospy.get_param('~Kp_lin')
        self.Ki_lin = rospy.get_param('~Ki_lin')
        self.Kd_lin = rospy.get_param('~Kd_lin')
        self.Kp_ang = rospy.get_param('~Kp_ang')
        self.Ki_ang = rospy.get_param('~Ki_ang')
        self.Kd_ang = rospy.get_param('~Kd_ang')
        self.i_lin_windup_limit = rospy.get_param('~I_lin_windup_limit')
        self.i_ang_windup_limit = rospy.get_param('~I_ang_windup_limit')
        self.max_lin_vel = rospy.get_param('~max_lin_vel')
        self.max_ang_vel = rospy.get_param('~max_ang_vel')
        self.setpoint_tolerance = rospy.get_param('~setpoint_tolerance')
        self.model_name = model_name
        self.curr_x = 0
        self.curr_y = 0
        self.curr_heading = 0
        self.target_x = 0
        self.target_y = 0
        self.prev_time = 0
        self.prev_dist = self.prev_ang_err = 0
        self.p_lin = self.i_lin = self.d_lin = 0
        self.p_ang = self.i_ang = self.d_ang = 0
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.is_navigating = Bool(False)
        self.waypoints = []
        self.counter = 0
        self.lock_points_publisher = rospy.Publisher('/lock_points', Bool, queue_size=1)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, callback=self._odomCallback)
        self.waypoints_subscriber = rospy.Subscriber('/waypoints', Polygon, callback=self._updateWaypoints, queue_size=1)
        self.initialpose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback=self._initialisePose, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / 20.0), self._update)
        
    def _updatePose(self, pose):
        self.curr_x = pose.position.x
        self.curr_y = pose.position.y
        orientation = pose.orientation
        _, _, self.curr_heading = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    def _update(self, event=None):
        if self.is_navigating.data:
            if math.sqrt((self.target_x - self.curr_x) ** 2 + (self.target_y - self.curr_y) ** 2) < self.setpoint_tolerance:
                self.counter += 1
                if self.counter < len(self.waypoints):
                    self.target_x = self.waypoints[self.counter][0]
                    self.target_y = self.waypoints[self.counter][1]
                
                else:
                    self._stop()
                    return
            
            self._PID()
            
    def _resetPID(self):
        self.prev_time = 0
        self.prev_dist = self.prev_ang_err = 0
        self.p_lin = self.i_lin = self.d_lin = 0
        self.p_ang = self.i_ang = self.d_ang = 0
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self._publishCmdVel()
        
    def _initialisePose(self, msg):
        if not self.is_navigating.data:
            pose = msg.pose.pose
            self._updatePose(pose)
            
            robot_state = ModelState()
            robot_state.model_name = self.model_name
            robot_state.pose.position = pose.position
            robot_state.pose.orientation = pose.orientation
            
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                set_state(robot_state)
                
                if self.waypoints:
                    self._resetPID()
                    self.target_x = self.waypoints[0][0]
                    self.target_y = self.waypoints[0][1]
                    self.prev_time = rospy.Time.now().to_sec()
                    self.is_navigating.data = True
                    self.lock_points_publisher.publish(self.is_navigating)
            
            except rospy.ServiceException as e:
                rospy.logwarn('Robot initial state setting failed.\n%s' % str(e))
                
        else:
            self._stop()
    
    def _odomCallback(self, msg):
        self._updatePose(msg.pose.pose)
        
    def _updateWaypoints(self, msg):
        self.waypoints = [(point.x, point.y) for point in msg.points]
        
    def _publishCmdVel(self):
        self.vel_publisher.publish(self.twist)
        
    def _PID(self):
        curr_time = rospy.Time.now().to_sec()
        x_diff = self.target_x - self.curr_x
        y_diff = self.target_y - self.curr_y
        curr_dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        curr_ang_err = math.atan2(y_diff, x_diff) - self.curr_heading
        
        # unwrap angle_err into [-pi, pi]
        while curr_ang_err < -math.pi:
            curr_ang_err += 2 * math.pi
        
        while curr_ang_err > math.pi:
            curr_ang_err -= 2 * math.pi
        
        
        delta_t = curr_time - self.prev_time
        
        if delta_t > 0:
            self.p_lin = curr_dist
            self.i_lin += curr_dist * delta_t
            self.d_lin = (curr_dist - self.prev_dist) / delta_t
            
            if self.i_lin > self.i_lin_windup_limit:
                self.i_lin = self.i_lin_windup_limit
            
            elif self.i_lin < -self.i_lin_windup_limit:
                self.i_lin = -self.i_lin_windup_limit
            
            self.p_ang = curr_ang_err
            self.i_ang += curr_ang_err * delta_t
            self.d_ang = (curr_ang_err - self.prev_ang_err) / delta_t
            
            if self.i_ang > self.i_ang_windup_limit:
                self.i_ang = self.i_ang_windup_limit
            
            elif self.i_ang < -self.i_ang_windup_limit:
                self.i_ang = -self.i_ang_windup_limit
            
            self.twist.linear.x = self.Kp_lin * self.p_lin + self.Ki_lin * self.i_lin + self.Kd_lin * self.d_lin
            if abs(self.twist.linear.x) > self.max_lin_vel:
                self.twist.linear.x = math.copysign(self.max_lin_vel, self.twist.linear.x)
            
            self.twist.angular.z = self.Kp_ang * self.p_ang + self.Ki_ang * self.i_ang + self.Kd_ang * self.d_ang
            if abs(self.twist.angular.x) > self.max_ang_vel:
                self.twist.angular.x = math.copysign(self.max_ang_vel, self.twist.angular.x)
            
            self._publishCmdVel()
        
        self.prev_time = curr_time
        self.prev_dist = curr_dist
        self.prev_ang_err = curr_ang_err
        
    def _stop(self):
        self.counter = 0
        self.is_navigating.data = False
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self._publishCmdVel()
        self.lock_points_publisher.publish(self.is_navigating)


if __name__ == "__main__":
    parser = ap.ArgumentParser()
    parser.add_argument('model')
    args, unknown = parser.parse_known_args()
    
    rospy.init_node('path_follow')
    path_follow = PathFollow(args.model)
    
    def reconfigCallback(config, level):
        path_follow.Kp_lin = config['Kp_lin']
        path_follow.Ki_lin = config['Ki_lin']
        path_follow.Kd_lin = config['Kd_lin']
        path_follow.Kp_ang = config['Kp_ang']
        path_follow.Ki_ang = config['Ki_ang']
        path_follow.Kd_ang = config['Kd_ang']
        path_follow.i_lin_windup_limit = config['I_lin_windup_limit']
        path_follow.i_ang_windup_limit = config['I_ang_windup_limit']
        path_follow.max_lin_vel = config['max_linear_vel']
        path_follow.max_ang_vel = config['max_angular_vel']
        path_follow.setpoint_tolerance = config['setpoint_tolerance']
        return config
    
    srv = Server(RobotControlParamsConfig, reconfigCallback)
    rospy.spin()
    

        
