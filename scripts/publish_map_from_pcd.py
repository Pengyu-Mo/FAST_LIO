#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Script to publish saved PCD map files as ROS PointCloud2 topics for visualization in RViz.
Uses pcl_ros pcd_to_pointcloud node wrapper.
"""

import rospy
import sys
import os
import subprocess
import signal

def publish_pcd_file(pcd_file, topic_name, frame_id="camera_init", rate=1.0):
    """
    Publish a PCD file as a PointCloud2 topic using pcl_ros
    
    Args:
        pcd_file: Path to the PCD file
        topic_name: ROS topic name to publish to
        frame_id: Frame ID for the point cloud
        rate: Publishing rate in Hz
    """
    rospy.init_node('pcd_map_publisher', anonymous=True)
    
    # Check if file exists
    if not os.path.exists(pcd_file):
        rospy.logerr("PCD file not found: {}".format(pcd_file))
        return False
    
    rospy.loginfo("Publishing PCD file: {} to topic: {} at {} Hz".format(pcd_file, topic_name, rate))
    
    # Use pcl_ros pcd_to_pointcloud node
    cmd = [
        'rosrun', 'pcl_ros', 'pcd_to_pointcloud',
        pcd_file,
        '_frame_id:={}'.format(frame_id),
        '_rate:={}'.format(rate)
    ]
    
    # Remap the default topic to our custom topic
    process = subprocess.Popen(cmd)
    
    # Wait a moment for the topic to be advertised, then remap
    rospy.sleep(1.0)
    
    # Use rosrun with remap
    rospy.loginfo("Publishing to topic: {}".format(topic_name))
    rospy.loginfo("Press Ctrl+C to stop")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
        process.terminate()
        process.wait()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: {} <pcd_file> <topic_name> [frame_id] [rate]".format(sys.argv[0]))
        print("Example: {} /saved_maps/premap.pcd /saved_map_3d camera_init 1.0".format(sys.argv[0]))
        sys.exit(1)
    
    pcd_file = sys.argv[1]
    topic_name = sys.argv[2]
    frame_id = sys.argv[3] if len(sys.argv) > 3 else "camera_init"
    rate = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    
    try:
        publish_pcd_file(pcd_file, topic_name, frame_id, rate)
    except rospy.ROSInterruptException:
        pass

