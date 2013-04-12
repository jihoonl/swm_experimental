#!/usr/bin/env python

import rospy
import actionlib
import yaml

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from rospy_message_converter import json_message_converter, message_converter

class PoseLoader(object):
    def __init__(self):
        self.filename = rospy.get_param('~filename')

        self.service = {}
        self.service['add_table_region'] = rospy.ServiceProxy('add_table_region',AddSemanticRegion)

    def load_file(self,filename):
        pose_yaml = None 
        with open(filename) as f:
            pose_yaml = yaml.load(f)
            tables = pose_yaml['table']

        return tables

    def insert_table(self,table):
        req = AddSemanticRegionRequest()
        req.name = table['name']

        req.pose_stamped.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',table['pose'])
        req.pose_stamped.header.frame_id = table['frame_id']
        req.region.radius = float(table['radius'])
        resp = self.service['add_table_region'](req)

        rospy.loginfo("%s is inserted as %s"%(table['name'],resp.instance_id))


        
    def spin(self):
        tables = self.load_file(self.filename)

        for t in tables:
            self.insert_table(t)




        

if __name__ == '__main__':
    rospy.init_node('pose_loader')
    pl = PoseLoader()
    rospy.loginfo('Initialized')
    pl.spin()
    rospy.loginfo('Bye Bye')

