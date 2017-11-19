#!/usr/bin/env python

import pickle
import numpy as np
import tf
from std_msgs.msg import Float64

from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
import rospy
from rospy_message_converter import message_converter
import yaml

from sensor_stick.pcl_helper import *

from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject

from pr2_robot.srv import *


def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)




test_scene_num = Int32()
test_scene_num.data = 1




# TODO parse the pick list ./config/dropbox.yaml file, parse dropbox.yaml



# get parameters
object_list_param = rospy.get_param('/object_list')
dropbox_list_param = rospy.get_param('/dropbox')
yaml_filename = str(test_scene_num.data)+"-zoutput.yaml"
saved=0
count=0
#print dropbox_list_param[0]['group']
#as red

#dict_list = []
all_objects={}
for i in range(0, len(object_list_param)):
    object_name_str = object_list_param[i]['name']
    object_group_str = object_list_param[i]['group']

    # Initialize a variable
    #object_name = String()

    # Populate the data field
    #object_name.data = object_name_str

    arm_name = String()
    #arm_name.data = object_group_str

    place_pose = Pose()
    if object_group_str == dropbox_list_param[0]['group']:
        place_pose.position.x = dropbox_list_param[0]['position'][0]
        place_pose.position.y = dropbox_list_param[0]['position'][1]
        place_pose.position.z = dropbox_list_param[0]['position'][2]
        arm_name.data = dropbox_list_param[0]['name']
    elif object_group_str == dropbox_list_param[1]['group']:
        place_pose.position.x = dropbox_list_param[1]['position'][0]
        place_pose.position.y = dropbox_list_param[1]['position'][1]
        place_pose.position.z = dropbox_list_param[1]['position'][2]
        arm_name.data = dropbox_list_param[1]['name']
    all_objects.update({object_name_str:[arm_name,place_pose]})


def detected_callback(detected_msg):
    #check objs
    rospy.loginfo('Detected  objects: {}'.format(len(detected_msg.objects)))
    #rospy.loginfo('Detected  objects: {}'.format(detected_msg.objects[0].label))
    dict_list = []
    global saved
    global count
    left_object=detected_msg.objects
    for i in range(0,len(detected_msg.objects)):
        #rospy.loginfo(detected_msg.objects[i].label)
        object_name_str=detected_msg.objects[i].label
        points_arr = ros_to_pcl(detected_msg.objects[i].cloud).to_array()
        center_point = np.mean(points_arr, axis=0)[:3]
        rospy.wait_for_service('pick_place_routine')
        if saved == 0:
             rospy.loginfo(detected_msg.objects[i].label)
             pick_pose = Pose()
             pick_pose.position.x=np.asscalar(center_point[0])
             pick_pose.position.y=np.asscalar(center_point[1])
             pick_pose.position.z=np.asscalar(center_point[2])
             arm_name=all_objects.get(object_name_str)[0]
             place_pose=all_objects.get(object_name_str)[1]
             object_name = String()
             object_name.data = object_name_str
             yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
             dict_list.append(yaml_dict)
             rospy.loginfo(center_point)
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            left_object.remove(detected_msg.objects[i])
            for k in range(0,len(left_object)):
                pcl_collision_pub.publish(left_object[k].cloud)
            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    if saved == 0:
        #send_to_yaml(yaml_filename+str(count), dict_list)
        rospy.loginfo(yaml_filename+str(count))
        #saved=1
        count+=1


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('detecting', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/detected_objects", DetectedObjectsArray, detected_callback, queue_size=1)
    pcl_collision_pub = rospy.Publisher("/pr2/3d_map/points", PointCloud2, queue_size=1)


    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
