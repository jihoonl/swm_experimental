#!/usr/bin/env python

import rospy
import actionlib
import copy
import json

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from semantic_region_handler.srv import *
from rospy_message_converter import json_message_converter

class SemanticRegionHandler(object):
    def __init__(self,spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref):
        
        self.spatial_world_model_ns = spatial_world_model_ns
        self.concert_name = concert_name
        self.instance_tags = instance_tags 
        self.description_tags = description_tags 
        self.descriptor_ref = descriptor_ref 

        rospy.loginfo('Connecting to World Model handler...')
        # Setting actions to spaital world model handler
        self._action = {}
        self._action['cwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/create_world_object_instance',CreateWorldObjectInstanceAction)
        self._action['cwod'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/create_world_object_description',CreateWorldObjectDescriptionAction)
        self._action['rwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/remove_world_object_instance',RemoveWorldObjectInstanceAction)
        self._action['uwoi'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/update_world_object_instance',UpdateWorldObjectInstanceAction)
        self._action['woits'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_instance_tag_search',WorldObjectInstanceTagSearchAction)
        self._action['wodts'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_description_tag_search',WorldObjectDescriptionTagSearchAction)

        self._action['cwoi'].wait_for_server()
        self._action['cwod'].wait_for_server()
        self._action['rwoi'].wait_for_server()
        self._action['uwoi'].wait_for_server()
        self._action['woits'].wait_for_server()
        self._action['wodts'].wait_for_server()
        rospy.loginfo('World Model Handler Ready')

        self._service = {}
        self._service['add_semantic_region'] = rospy.Service('add_semantic_region',AddSemanticRegion,self.process_add_semantic_region)
        self._service['get_semantic_region'] = rospy.Service('get_semantic_region',GetSemanticRegion,self.process_get_semantic_region)
        self._service['remove_semantic_region'] = rospy.Service('remove_semantic_region',RemoveSemanticRegion,self.process_remove_semantic_region)

    def process_add_semantic_region(self,req):
        # Search database to check whether it exists already
        description_id = self.get_radius_description(req.radius)

        # if no matched description
        if description_id == None:
            description_id = self.create_radius_world_description(req.radius)

        instance_id = self.add_radius_region_instance(req.name,req.pose_stamped.header,req.pose_stamped.pose,description_id)
        return AddSemanticRegionResponse(instance_id)


    def get_radius_descriptor(radius):
        # Check whether this raiuus descriptor is already in the database
        self._action['cwod'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.description_tags))
        resp = self._action['cwod'].get_result()

        description_id = None
        for d in resp.descriptions: 
            data = json_message_converter.convert_json_to_ros_message('semantic_region_handler/Region',d.descriptors[0].data)
            if data.radius == radius:
                description_id = d.description_id
                break

        return description_id
        
    def create_radius_world_description(self,radius):
        radius_descriptor = self.create_radius_descriptor(radius)

        description = WorldObjectDescription()
        description.name = str(radius)
        description.tags = copy.deepcopy(self.description_tags)
        description.descriptors.append(radius_descriptor)

        self._action['cwod'].send_goal_and_wait(CreateWorldObjectDescriptionGoal(description))
        result = self._action['cwod'].get_result()

        return result.description_id

    def create_radius_descriptor(self,radius):
        region = Region(radius)

        d = Descriptor()
        d.type = '/semantic_region_handler/Region'
        d.data = json_message_converter.convert_ros_message_to_json(region)
        d.ref = self.descriptor_ref
        d.tags = self.description_tags

        return d

    def add_radius_region_instance(self,name,header,pose,description_id):
        instance = WorldObjectInstance()
        instance.name = name
        instance.description_id = description_id
        instance.expected_ttl = rospy.Duration(1)
        instance.pose = self.create_pose_cov(header,pose)
        instance.source.origin = self.concert_name
        instance.source.creator = rospy.get_name()
        instance.tags = copy.deepcopy(self.instance_tags)

        self._action['cwoi'].send_goal_and_wait(CreateWorldObjectInstanceGoal(instance))
        result = self._action['cwoi'].get_result()
        return result.instance_id

    def create_pose_cov(self,pose,header):
        pcs = PoseWithCovarianceStamped()
        pcs.pose.pose = pose
        pcs.header = header
        pcs.pose.covariance =  [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        return pcs


    def process_remove_semantic_region(self,req):
        self._action['rwoi'].send_goal_and_wait(RemoveWorldObjectInstanceGoal(req.instance_id))
        resp = self._action['rwoi'].get_result()

        return RemoveSemanticRegionResponse(resp.result)

    def process_get_semantic_region(self,req):
        self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.instance_tags))
        resp = self._action['woits'].get_result()
        
        print str(resp)

        instance = None
        for i in resp.instances:
            if i.name == req.name:
                instances = i
                break
                
        return GetSemanticRegionResponse(instances.pose_stamped,instances.radius)

    def spin(self):
        rospy.spin()

