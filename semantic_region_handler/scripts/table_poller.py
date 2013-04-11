#!/usr/bin/env python
import rospy
import json

from semantic_region_handler import *

if __name__ == '__main__':

    rospy.init_node('polling_table')
    spatial_world_model_ns = '/spatial_world_model'
    concert_name = "concert"
    instance_tags = [concert_name,'table']
    description_tags = [concert_name,'table','radius']
    descriptor_ref = json.dumps({'type':'semantic_circle'}) 

    sph = TablePoller(spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref)
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')

