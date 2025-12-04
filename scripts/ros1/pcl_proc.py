#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import math
import copy

class ZFilterNode:
    def __init__(self):
        rospy.init_node('z_filter_node', anonymous=True)
        self.pub = rospy.Publisher('/velodyne_points_filtered', PointCloud2, queue_size=1)
        self.rad_filter_pub = rospy.Publisher('/velodyne_points_rad_filtered', PointCloud2, queue_size=1)
        # rospy.Subscriber('/lidar/point_cloud', PointCloud2, self.callback)
        rospy.Subscriber('/smb_arl/lidar/points_downsampled', PointCloud2, self.callback)
        # rospy.Subscriber('/input_pointcloud', PointCloud2, self.callback)
        # rospy.Subscriber('/velodyne_points', PointCloud2, self.callback)

    def callback(self, msg):
        # Convert PointCloud2 to a generator of points
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points2 = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Filter out points with z > 0.0
        # for pt in points:
        #     print(pt[0], pt[1], pt[2], " | ", math.hypot(pt[0], pt[1]))

        filtered_points = [
            pt for pt in points
            if pt[2] <= 0.75 and math.hypot(pt[0], pt[1]) > 1.5
            # if pt[2] <= 0.5
        ]
        

        rad_filtered_points = [
            pt for pt in points2
            if math.hypot(pt[0], pt[1]) > 2.0
        ]


        # print("rad_filtered_points", len(rad_filtered_points))
        # print("filtered_points", len(filtered_points))

        filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)
        self.pub.publish(filtered_msg)

        rad_filtered_msg = pc2.create_cloud_xyz32(msg.header, rad_filtered_points)
        self.rad_filter_pub.publish(rad_filtered_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ZFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass