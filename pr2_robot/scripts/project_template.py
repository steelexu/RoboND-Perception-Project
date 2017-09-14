#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    #  Convert ROS msg to PCL data
    pcloud=ros_to_pcl(pcl_msg)
    # DO: Statistical Outlier Filtering (mean distance+x*std_dev)=outlier
    outlier_filter = pcloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.3
    outlier_filter.set_std_dev_mul_thresh(x)
    stats_filtered = outlier_filter.filter()

    #  Voxel Grid Downsampling
    vox=stats_filtered.make_voxel_grid_filter()
    #vox=pcloud.make_voxel_grid_filter()
    LEAF_SIZE=0.005
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloudfiltered=vox.filter()
    # TODO: PassThrough Filter
    passthru=cloudfiltered.make_passthrough_filter()
    filteraxis='z'
    passthru.set_filter_field_name(filteraxis)
    axismin=0.6
    axismax=1.2	
    passthru.set_filter_limits(axismin,axismax)
    cloudfiltered=passthru.filter()

    passthru=cloudfiltered.make_passthrough_filter()
    filteraxis='y'
    passthru.set_filter_field_name(filteraxis)
    axismin=-0.4
    axismax=0.4
    passthru.set_filter_limits(axismin,axismax)
    cloudfiltered=passthru.filter()

    # DO: RANSAC Plane Segmentation
    seg=cloudfiltered.make_segmenter()

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    maxdistance=0.02
    seg.set_distance_threshold(maxdistance)
    inliers,coefficients=seg.segment()


    # DO: Extract inliers and outliers
    extractedinliers=cloudfiltered.extract(inliers,negative=False)
    extractedoutliers=cloudfiltered.extract(inliers,negative=True)
    # DO: Euclidean Clustering
    white_cloud=XYZRGB_to_XYZ(extractedoutliers)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(2000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()



    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages
    ros_cloud_object = pcl_to_ros(extractedoutliers)
    ros_cloud_table = pcl_to_ros(extractedinliers)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_object)
    pcl_table_pub.publish(ros_cloud_table)
# Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []
    # Classify the clusters! (loop through each detected cluster one at a time)

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = extractedoutliers.extract(pts_list)
        # DO: convert the cluster from pcl to ROS using helper function
        ros_cluster_cloud = pcl_to_ros(pcl_cluster)
        pcl_cluster_pub.publish(ros_cluster_cloud)
        # Extract histogram features

        # Compute the associated feature vector


        # DO: complete this step just as you did before in capture_features.py
        chists = compute_color_histograms(ros_cluster_cloud, using_hsv=True)
        normals = get_normals(ros_cluster_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
 

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    #try:
    #    pr2_mover(detected_objects_list)
    #except rospy.ROSInterruptException:
    #    pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)  
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1) 


    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()