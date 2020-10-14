#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from simplify_path.cfg import SimplifyPathReconfigConfig

def callback(config, level):
    try:
        max_N = rospy.get_param('/max_N')
        if config['N'] > max_N:
            config['N'] = max_N
    
    except:
        pass
    
    rospy.loginfo(config)
    return config

if __name__ == '__main__':
    rospy.init_node('dynamic_reconfig_server')

    srv = Server(SimplifyPathReconfigConfig, callback)
    rospy.spin()
