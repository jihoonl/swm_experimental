#!/usr/bin/env python

import rospy
import actionlib
import yaml

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from swm_experimental.msg import *

class PoseLoader(object):
    def __init__(self):
        self.filename = rospy.get_param('~filename')

    def load_file(self,filename):
        pose_yaml = None 
        with open(filename) as f:
            pose_yaml = yaml.load(f)

            tables = pose_yaml['table']
            print tables[0]


        return pose_yaml

        
    def spin(self):
        table_poses = self.load_file(self.filename)

        rospy.spin()


        

if __name__ == '__main__':
    rospy.init_node('pose_loader')
    pl = PoseLoader()
    rospy.loginfo('Initialized')
    pl.spin()
    rospy.loginfo('Bye Bye')

