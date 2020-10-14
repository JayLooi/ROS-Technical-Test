#!/usr/bin/env python

from __future__ import print_function
import argparse as ap
import os
import math
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rosbag
import dynamic_reconfigure.client
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


if os.name == 'nt':
    import msvcrt
else:
    import sys
    import tty
    import termios


def getch():
    if os.name == 'nt':
        return msvcrt.getch()
    
    fd = sys.stdin.fileno()
    ori_settings = termios.tcgetattr(fd)
    tty.setraw(fd)
    try:
        ch = sys.stdin.read(1)
    
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, ori_settings)
    
    return ch
    

class SimplifyPath:
    def __init__(self, in_bag_fp, out_bag_fp):
        self.dyn_client = dynamic_reconfigure.client.Client('dynamic_reconfig_server')
        self.in_bag_fp = in_bag_fp
        self.out_bag_fp = out_bag_fp
        self._updateDynamicParams()
        self.ori_path_publisher = rospy.Publisher('/ori_path', PoseArray, queue_size=1)
        self.new_path_publisher = rospy.Publisher('/new_path', PoseArray, queue_size=1)
        rospy.init_node('simplify_path')
        self.pose_arr = PoseArray()
        self.pose_arr.header.frame_id = 'map-2'
        
        self.topic_list = []
        self.topic = None
        with rosbag.Bag(self.in_bag_fp) as bag:
            self.choice_list = 'Select one topic below (press number key corresonding to the topic):'
            for i, (key, val) in enumerate(bag.get_type_and_topic_info()[1].iteritems(), start=1):
                self.choice_list += '\n%d. %s (%s)' % (i, key, val[0])
                self.topic_list.append(key)
    
    def _updateDynamicParams(self):
        self.eps = rospy.get_param('/dynamic_reconfig_server/epsilon')
        self.N = rospy.get_param('/dynamic_reconfig_server/N')
    
    def _getOriPath(self, topic):
        pose_arr = PoseArray()
        pose_arr.header.frame_id = 'map-1'
        messages = []
        with rosbag.Bag(self.in_bag_fp) as in_bag:
            for tp, msg, t in in_bag.read_messages(topic):
                pose_arr.poses.append(msg.pose)
                messages.append(msg)
        
        return pose_arr, messages

    def _selectTopic(self):
        n = len(self.topic_list)
        if n > 0:
            choice = 0
            print(self.choice_list)
            while 1:
                try:
                    choice = int(getch())
                    if choice > 0 and choice <= n:
                        self.topic = self.topic_list[choice-1]
                        rospy.loginfo('Topic selected: %s' % self.topic)
                        break
                
                except:
                    pass
                    
            return self.topic
    
    def _DouglasPeucker(self, poses):
        start = np.array([poses[0].pose.position.x, poses[0].pose.position.y])
        end = np.array([poses[-1].pose.position.x, poses[-1].pose.position.y])
        norm = np.linalg.norm
        max_dist = 0
        sep_index = 0
        groups = []
        filtered_poses = []
        for i, ps in enumerate(poses[1:-1], start=1):
            p = np.array([ps.pose.position.x, ps.pose.position.y])
            dist = np.abs(np.cross(p-start, end-start)) / norm(end-start)
            
            if dist > max_dist:
                max_dist = dist
                sep_index = i
        
        if max_dist > self.eps:
            result1, grp1 = self._DouglasPeucker(poses[:sep_index+1])
            result2, grp2 = self._DouglasPeucker(poses[sep_index:])
            filtered_poses.extend(result1[:-1])
            filtered_poses.extend(result2)
            groups.extend(grp1)
            groups.extend(grp2)
        
        else:
            filtered_poses.extend([poses[0], poses[-1]])
            groups.append(poses[:-1])
        
        return filtered_poses, groups
    
    def _increaseGroups(self, groups, m):
        if m > 0:
            max_points = 0
            max_index = 0
            for i, grp in enumerate(groups):
                n = len(grp)
                if n > max_points:
                    max_points = n
                    max_index = i
            
            longest = groups.pop(max_index)
            sep_i = max_points // 2
            split1, split2 = longest[:sep_i], longest[sep_i:]
            groups.insert(max_index, split2)
            groups.insert(max_index, split1)
            groups = self._increaseGroups(groups, m - 1)
        
        return groups
    
    def _compressToNWaypoints(self, poses, groups):
        n = len(groups)
        if self.N > n:
            groups = self._increaseGroups(groups, self.N - n)
        
        if self.N < n:
            rospy.logwarn(  'For epsilon of %f, the path can only be compressed to minimum of %d waypoints. '   \
                            'Try increasing the epsilon values to obtain smaller number of waypoints' % (self.eps, n))
            self.N = n
            self.dyn_client.update_configuration({'N': self.N})
        
        return [grp[0] for grp in groups]

    def _compressWaypoints(self, poses):
        poses, groups = self._DouglasPeucker(poses)
        groups.append([poses[-1],])
        return self._compressToNWaypoints(poses, groups)
    
    def _saveNewPath(self):
        with rosbag.Bag(self.out_bag_fp, 'w') as out_bag:
            for msg in self.new_messages:
                out_bag.write(self.topic, msg)
    
    def _promptUser(self):
        print('Press:')
        print('Key <S> or <s>: Save the new path in a bag file.')
        print('Key <R> or <r>: Re-compute the path.')
        print('Key <C> or <c>: Select other topic.')
        print('Key <ESC>     : End the node.')
        while 1:
            ch = getch().lower()
            if ch in ('s', 'r', 'c', '\x1b'):
                return ch
    
    def mainloop(self):
        ch = 'c'
        while 1:
            if ch == 'c':
                ori_path, ori_messages = self._getOriPath(self._selectTopic())
                rospy.set_param('/max_N', len(ori_messages))
                self.ori_path_publisher.publish(ori_path)
                ch = 'r'
            
            if ch == 'r':
                self._updateDynamicParams()
                self.new_messages = self._compressWaypoints(ori_messages)
                self.pose_arr.poses = [msg.pose for msg in self.new_messages]
                self.new_path_publisher.publish(self.pose_arr)
                rospy.loginfo('Done computing new path with number of waypoints reduced to %d.' % len(self.new_messages))
            
            elif ch == 's':
                self._saveNewPath()
        
            elif ch == '\x1b':
                break
            
            ch = self._promptUser()


if __name__ == '__main__':
    def inBagFilepathCheck(fp):
        if not os.path.isfile(fp):
            raise ap.ArgumentTypeError('Invalid filepath: %s' % fp)
        
        return fp
    
    
    def outBagFilepathCheck(fp):
        try:
            with open(fp, 'w'): pass
            return fp
        
        except Exception as e:
            raise ap.ArgumentTypeError('Invalid filepath: %s\n%s' % (fp, e))
            
    
    parser = ap.ArgumentParser(description='')
    parser.add_argument('in_bag_fp', type=inBagFilepathCheck)
    parser.add_argument('out_bag_fp', type=outBagFilepathCheck)
    args, unknown = parser.parse_known_args()
    simplify_path = SimplifyPath(args.in_bag_fp, args.out_bag_fp)
    simplify_path.mainloop()

