#!/usr/bin/env python

import rospy
import actionlib
import copy

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from swm_experimental.srv import *

class SemanticPoseHandler(object):
    def __init__(self):
        
        self.spatial_world_model_ns = '/spatial_world_model'
        self.concert_name = "concert"
        self.tags = ['semantic_pose','concert']

        rospy.loginfo('Connecting to World Model handler...')
        # Setting actions to spaital world model handler
        self._action = {}
        self._action['cwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/create_world_object_instance',CreateWorldObjectInstanceAction)
        self._action['rwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/remove_world_object_instance',RemoveWorldObjectInstanceAction)
        self._action['uwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/update_world_object_instance',UpdateWorldObjectInstanceAction)
        self._action['woits'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_instance_tag_search',WorldObjectInstanceTagSearchAction)

        self._action['cwoi'].wait_for_server()
        self._action['rwoi'].wait_for_server()
        self._action['uwoi'].wait_for_server()
        self._action['woits'].wait_for_server()
        rospy.loginfo('World Model Handler Ready')

        self._service = {}
        self._service['add_semantic_pose'] = rospy.Service('add_semantic_pose',AddSemanticPose,self.processAddSemanticPose)
        self._service['get_semantic_pose'] = rospy.Service('get_semantic_pose',GetSemanticPose,self.processGetSemanticPose)
        self._service['remove_semantic_pose'] = rospy.Service('remove_semantic_pose',RemoveSemanticPose,self.processRemoveSemanticPose)

    def processAddSemanticPose(self,req):

        result = self.addSemanticPose(req.name,req.pose_stamped.header,req.pose_stamped.pose)
        return AddSemanticPoseResponse("Pose Added")

    def processRemoveSemanticPose(self,req):

        self._action['rwoi'].send_goal_and_wait(RemoveWorldObjectInstanceGoal(req.instance_id))
        resp = self._action['rwoi'].get_result()

        return RemoveSemanticPoseResponse(resp.result)

    def addSemanticPose(self,name,header,pose):
        self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.tags))
        resp = self._action['woits'].get_result()

        instance = WorldObjectInstance()
        instance.expected_ttl = rospy.Duration(1)
        instance.tags = copy.deepcopy(self.tags)
        instance.tags.append(name)
        instance.source.origin = self.concert_name
        instance.source.creator = rospy.get_name()
        
        instance.name = name
        pcs = PoseWithCovarianceStamped()
        pcs.pose.pose = pose
        pcs.header = header
        pcs.pose.covariance =  [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        instance.pose = pcs

        self._action['cwoi'].send_goal_and_wait(CreateWorldObjectInstanceGoal(instance))



    def processGetSemanticPose(self,req):
        self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.tags))
        resp = self._action['woits'].get_result()

        print str(resp)

        p_stamped = PoseStamped()
        return GetSemanticPoseResponse(p_stamped)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':

    rospy.init_node('add_semantic_pose')
    sph = SemanticPoseHandler()
    rospy.loginfo('Initialized')
    sph.spin()
    rospy.loginfo('Bye Bye')


  
