#!/usr/bin/env python

import rospy
import actionlib

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from swm_experimental.msg import *

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
        self._publisher['poses_only'] = rospy.Publisher('poses_only',PoseArray, latch=True)
        self._publisher['table_pose_lists'] = rospy.Publisher('table_pose_lists',TablePoseList,latch=True)

    def parse(self,instances):

        posearray = PoseArray()
        posearray.header.frame_id = '/map'
        posearray.header.stamp = rospy.Time.now()
    
        table_pose_list = TablePoseList()

        for i in instances:
            pose = i.pose.pose.pose
            posearray.poses.append(pose)

            table = TablePose()
            table.name = i.name
            table.pose_stamped.header = i.pose.header
            table.pose_stamped.pose = i.pose.pose.pose
            table_pose_list.tables.append(table)
    
        return posearray, table_pose_list
                   

    def spin(self):
        
        while not rospy.is_shutdown():
            self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.tags))
            resp = self._action['woits'].get_result()
            
            posearray, table_pose_lists = self.parse(resp.instances)

            self._publisher['poses_only'].publish(posearray)
            self._publisher['table_pose_lists'].publish(table_pose_lists)

            rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node('polling_table')
    sph = SemanticPoseHandler()
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')


  
