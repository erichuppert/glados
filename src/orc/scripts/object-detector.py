#!/usr/bin/python2
import rospy
import message_filters
import cv2
import math
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import time
import tf

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PointStamped
from roslib import message

bridge = CvBridge()
block_locations = []

def handle_msg(image, pcl_data):
    global blob_image_pub
    # get the image and find the blobs
    cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
    cv_image = cv2.GaussianBlur(cv_image, (5,5), 0)
    # keypoints = blob_detector.detect(cv_image)
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    keypoints = find_keypoints(hsv_image)
    draw_keypoints(cv_image,keypoints)
    ros_keypoints_im = bridge.cv2_to_imgmsg(cv_image)
    blob_image_pub.publish(ros_keypoints_im)
    if keypoints:
        try:
            potential_block_locations = keypoints_to_block_locations(keypoints, pcl_data)
        except:
            print("Failed to convert to block locations")
            return
        for location in potential_block_locations:
            if not block_already_seen(location):
                save_block_location(location)
        closest_block = max(keypoints, key=lambda x: x.size)
        nearest_block_msg = NearestBlock()
        nearest_block_msg.size = closest_block.size
        nearest_block_msg.x_location = round(closest_block.pt.x)
        nearest_block_msg.y_location = round(closest_block.pt.y)
        nearest_block_pix_size_pub.publish(nearest_block_msg)
        

def draw_keypoints(image, keypoints, color = (255, 0, 0)):
    for kp in keypoints:
        x, y = kp
        cv2.circle(image, (int(x), int(y)), 30, color)

def keypoints_to_block_locations(keypoints, pcl_data):
    global listener
    locations = []
    for keypoint_x, keypoint_y in keypoints:
        keypoint = read_point(keypoint_x, keypoint_y, pcl_data)
        if any(math.isnan(k) for k in keypoint):
            continue
        pt = PointStamped()
        pt.header = pcl_data.header
        pt.point.x,pt.point.y,pt.point.z = keypoint
        pt_base = listener.transformPoint("odom", pt)
        locations.append(pt_base.point)
    return locations

def read_point(width, height, data) :
    # read function
    assert (round(height) < data.height) and (round(width) < data.width), "OH NOES, THIS SHOULD NOT HAPPEN"
    data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[int(round(width)), int(round(height))]])
    int_data = next(data_out)
    return int_data

color_boundaries = [
    (np.array([170, 100, 100]), np.array([180, 255, 255])), # red
    (np.array([18, 180, 80]), np.array([30, 255, 255])), # yellow
]
def find_keypoints(hsv_image):
    keypoints = []
    for min, max in color_boundaries:
        color_limited = cv2.inRange(hsv_image, min, max)
        keypoints += find_contour_centers(color_limited)

    return keypoints

def find_contour_centers(img):
    contours, _ = cv2.findContours(img.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    centers = []
    for contour in contours:
        moments = cv2.moments(contour)
        moment_area = cv2.contourArea(contour)
        if moments['m00'] != 0 and moment_area > 150:
            centers.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
    return centers

BLOCK_DISTANCE_THRESHOLD = 0.3
Z_THRESHOLD = 0.2
def block_already_seen(block_location):
    distance_evals = [(location.x-block_location.x)**2 + (location.y-block_location.y)**2 < BLOCK_DISTANCE_THRESHOLD
                      for location in block_locations]
    return any(distance_evals) or block_location.z > Z_THRESHOLD

def save_block_location(location):
    block_location_pub.publish(location)
    block_locations.append(location)

def main():
    global blob_image_pub,block_location_pub,listener
    rospy.init_node('object_detector')

    image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    pcl_sub = message_filters.Subscriber("/camera/depth/points", PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, pcl_sub], 1, 0.1)
    ts.registerCallback(handle_msg)
    nearest_block_pix_size_pub = rospy.Publisher("")
    blob_image_pub = rospy.Publisher("blobs", Image, queue_size=10)
    block_location_pub = rospy.Publisher("block_location", Point)
    listener = tf.TransformListener();

    rospy.spin()

if __name__ == "__main__":
    main()
