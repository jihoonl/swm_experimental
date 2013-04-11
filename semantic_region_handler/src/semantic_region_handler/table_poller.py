#!/usr/bin/env python
import rospy
import actionlib

from worldlib.msg import *
from world_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from semantic_region_handler.msg import *
from rospy_message_converter import json_message_converter

class TablePoller(object):
    def __init__(self,spatial_world_model_ns,concert_name,instance_tags,description_tags,descriptor_ref):
        self.spatial_world_model_ns = spatial_world_model_ns
        self.concert_name = concert_name
        self.instance_tags = instance_tags
        self.description_tags = description_tags
        self.descriptor_ref =descriptor_ref

        rospy.loginfo('Connecting to World Model handler...')
        # Setting actions to spaital world model handler
        self._action = {}
        self._action['woits'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_instance_tag_search',WorldObjectInstanceTagSearchAction)
        self._action['wodts'] = actionlib.SimpleActionClient(self.spatial_world_model_ns + '/world_object_description_tag_search',WorldObjectDescriptionTagSearchAction)
        self._action['woits'].wait_for_server()
        self._action['wodts'].wait_for_server()
        rospy.loginfo('World Model Handler Ready')

        self._publisher = {}
        self._publisher['table_markers'] = rospy.Publisher('table_markers',MarkerArray, latch=True)
        self._publisher['table_pose_list'] = rospy.Publisher('table_pose_list',TablePoseList,latch=True)

        self.descriptions = {}

    def parse(self,instances):
        # TablePostList preparation
        table_pose_list = TablePoseList()
        # Markers
        marker_list = MarkerArray()

        marker_id = 1
        for i in instances:
            table = TablePose()
            table.name = i.name
            table.pose_cov_stamped = i.pose
            table.region = self.get_region(i.description_id)
            table_pose_list.tables.append(table)

            marker = Marker()
            marker.id = marker_id
            marker.header = i.pose.header
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.SPHERE
            marker.ns = self.concert_name
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration.from_sec(1)
            marker.pose = i.pose.pose.pose
            marker.scale.x = table.region.radius
            marker.scale.y = table.region.radius
            marker.scale.z = 0.1
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_list.markers.append(marker)

            marker_id = marker_id + 1
    
        return marker_list, table_pose_list

    def get_region(self,description_id):
        description = self.descriptions[description_id]
        message = json_message_converter.convert_json_to_ros_message(description.descriptors[0].type, description.descriptors[0].data)
        
        return message


    def get_world_object_description(self,description_id):
        self._action['wodts'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.instance_tags))
        resp = self._action['wodts'].get_result()

        descriptions = [ d for d in resp.descriptions if d.description_id == description_id]

        if len(descriptions) > 1:
            rospy.logwarn("There is more than one descriptions with id [%s]. Selecting the first one"%description_id)
        description = descriptions[0]

        return description

    def get_table_instances(self):
        self._action['woits'].send_goal_and_wait(WorldObjectInstanceTagSearchGoal(self.instance_tags))
        resp = self._action['woits'].get_result()                                                               

        return resp.instances

    def update_table_descriptions(self,instances):
        for i in instances:
            if not (i.description_id in self.descriptions): 
                self.descriptions[i.description_id] = self.get_world_object_description(i.description_id)


    def spin(self):
        while not rospy.is_shutdown():
            # Get Instances from database
            table_instances = self.get_table_instances()
            self.update_table_descriptions(table_instances)

            marker_list, table_pose_list = self.parse(table_instances)
            
            self._publisher['table_markers'].publish(marker_list)
            self._publisher['table_pose_list'].publish(table_pose_list)

            rospy.sleep(1)


