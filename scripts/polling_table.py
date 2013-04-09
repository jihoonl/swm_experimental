#!/usr/bin/env python

import rospy
import actionlib

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *

class SemanticPoseHandler(object):
    def __init__(self):
        
        self.spatial_world_model_ns = '/spatial_world_model'
        self.concert_name = "concert"
        self.tags = ['semantic_pose','concert']

        rospy.loginfo('Connecting to World Model handler...')
        # Setting actions to spaital world model handler
        self._action = {}
        self._action['woits'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_instance_tag_search',WorldObjectInstanceTagSearchAction)

        self._action['woits'].wait_for_server()
        rospy.loginfo('World Model Handler Ready')

        self._publisher = {}
        self._publisher['polling_table'] = rospy.Publisher('polling_table',WorldObjectInstance)

    def spin(self):
        
        while not rospy.is_shutdown():
            self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.tags))
            resp = self._action['woits'].get_result()
            print str(resp)
            rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('polling_table')
    sph = SemanticPoseHandler()
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')


  
