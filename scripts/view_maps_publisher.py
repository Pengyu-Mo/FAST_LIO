#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple script to publish PCD files as PointCloud2 topics for visualization.
Also publishes TF frames (camera_init, map, body) so they appear in RViz.
This is more reliable than using pcl_ros pcd_to_pointcloud.
"""

import rospy
import sys
import os
import yaml
import threading
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf

def read_pcd_ascii(pcd_file):
    """Read PCD file (ASCII format) and return points"""
    points = []
    with open(pcd_file, 'r') as f:
        lines = f.readlines()
        
        # Find DATA section
        data_start = -1
        width = 1
        fields = []
        for i, line in enumerate(lines):
            if line.startswith('WIDTH'):
                width = int(line.split()[1])
            elif line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('DATA'):
                data_start = i + 1
                break
        
        if data_start == -1:
            rospy.logerr("Could not find DATA section in PCD file")
            return None
        
        # Read point data
        for line in lines[data_start:]:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                values = [float(x) for x in line.split()]
                if len(values) >= 3:
                    # x, y, z, [intensity]
                    points.append(values[:4] if len(values) >= 4 else values[:3] + [0.0])
            except:
                continue
    
    return points, fields

def load_frame_transforms(frames_yaml_file):
    """Load frame transform information from YAML file"""
    if not os.path.exists(frames_yaml_file):
        rospy.logwarn("Frame transforms YAML not found: {}".format(frames_yaml_file))
        return None
    
    try:
        with open(frames_yaml_file, 'r') as f:
            frame_data = yaml.safe_load(f)
        rospy.loginfo("Loaded frame transforms from: {}".format(frames_yaml_file))
        return frame_data
    except Exception as e:
        rospy.logwarn("Failed to load frame transforms: {}".format(str(e)))
        return None

def publish_frames(frames_yaml_file=None, rate=10.0):
    """Publish TF frames (camera_init, map, body) so they appear in RViz"""
    # Create static transform broadcaster (ROS1 uses tf.TransformBroadcaster)
    static_broadcaster = tf.TransformBroadcaster()
    
    # Default: map and camera_init are the same (identity transform)
    # Publish map -> camera_init (identity)
    map_to_camera_init_transform = tf.Transform()
    map_to_camera_init_transform.setOrigin(tf.Vector3(0.0, 0.0, 0.0))
    map_to_camera_init_transform.setRotation(tf.Quaternion(0.0, 0.0, 0.0, 1.0))
    
    # Try to load transform from camera_init to body from YAML file
    camera_init_to_body_transform = tf.Transform()
    camera_init_to_body_transform.setOrigin(tf.Vector3(0.0, 0.0, 0.0))
    camera_init_to_body_transform.setRotation(tf.Quaternion(0.0, 0.0, 0.0, 1.0))
    
    if frames_yaml_file:
        frame_data = load_frame_transforms(frames_yaml_file)
        if frame_data and 'transforms' in frame_data:
            if 'camera_init_to_body' in frame_data['transforms']:
                tf_data = frame_data['transforms']['camera_init_to_body']
                trans = tf_data.get('translation', {})
                rot = tf_data.get('rotation', {})
                camera_init_to_body_transform.setOrigin(tf.Vector3(
                    trans.get('x', 0.0),
                    trans.get('y', 0.0),
                    trans.get('z', 0.0)
                ))
                camera_init_to_body_transform.setRotation(tf.Quaternion(
                    rot.get('x', 0.0),
                    rot.get('y', 0.0),
                    rot.get('z', 0.0),
                    rot.get('w', 1.0)
                ))
                rospy.loginfo("Loaded camera_init -> body transform from YAML")
    
    # Keep republishing at specified rate (for static transforms, this ensures they stay in TF tree)
    rospy.loginfo("Publishing TF frames: map -> camera_init, camera_init -> body")
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        # Publish map -> camera_init (identity)
        # sendTransform(transform, time, child_frame, parent_frame)
        static_broadcaster.sendTransform(
            tf.StampedTransform(
                map_to_camera_init_transform,
                rospy.Time.now(),
                "map",  # parent frame
                "camera_init"  # child frame
            )
        )
        
        # Publish camera_init -> body
        static_broadcaster.sendTransform(
            tf.StampedTransform(
                camera_init_to_body_transform,
                rospy.Time.now(),
                "camera_init",  # parent frame
                "body"  # child frame
            )
        )
        r.sleep()

def publish_pcd(pcd_file, topic_name, frame_id="camera_init", rate=1.0):
    """Publish PCD file as PointCloud2 topic"""
    # Note: rospy.init_node should be called before this function
    
    if not os.path.exists(pcd_file):
        rospy.logerr("PCD file not found: {}".format(pcd_file))
        return
    
    rospy.loginfo("Reading PCD file: {}".format(pcd_file))
    
    # Read PCD file
    try:
        points, fields = read_pcd_ascii(pcd_file)
        if points is None or len(points) == 0:
            rospy.logerr("No points found in PCD file")
            return
        
        rospy.loginfo("Loaded {} points from {}".format(len(points), pcd_file))
    except Exception as e:
        rospy.logerr("Failed to read PCD file: {}".format(str(e)))
        return
    
    # Create publisher
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1, latch=True)
    
    # Create PointCloud2 message
    header = rospy.Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()
    
    # Determine field names
    if len(fields) >= 4 and 'intensity' in fields:
        # Has intensity
        cloud_msg = pc2.create_cloud(header, 
            [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
             pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
             pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
             pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1)],
            points)
    else:
        # No intensity, just xyz
        cloud_msg = pc2.create_cloud_xyz32(header, 
            [(p[0], p[1], p[2]) for p in points])
    
    # Publish at specified rate
    r = rospy.Rate(rate)
    rospy.loginfo("Publishing {} to topic {} at {} Hz".format(pcd_file, topic_name, rate))
    rospy.loginfo("Frame ID: {}".format(frame_id))
    
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()
        pub.publish(cloud_msg)
        r.sleep()

if __name__ == '__main__':
    # Support both command-line args and ROS parameters
    rospy.init_node('pcd_publisher', anonymous=True)
    
    # Try ROS parameters first
    pcd_file = rospy.get_param('~pcd_file', None)
    topic_name = rospy.get_param('~topic_name', None)
    frame_id = rospy.get_param('~frame_id', 'camera_init')
    rate = rospy.get_param('~rate', 1.0)
    publish_frames_enabled = rospy.get_param('~publish_frames', True)
    frames_yaml_file = rospy.get_param('~frames_yaml_file', None)
    
    # Fall back to command-line args if parameters not set
    if pcd_file is None or topic_name is None:
        if len(sys.argv) < 3:
            print("Usage: {} <pcd_file> <topic_name> [frame_id] [rate] [frames_yaml]".format(sys.argv[0]))
            print("Or set ROS parameters: ~pcd_file, ~topic_name, ~frame_id, ~rate, ~publish_frames, ~frames_yaml_file")
            print("Example: {} /saved_maps/premap.pcd /saved_map_3d camera_init 1.0".format(sys.argv[0]))
            sys.exit(1)
        pcd_file = sys.argv[1]
        topic_name = sys.argv[2]
        frame_id = sys.argv[3] if len(sys.argv) > 3 else "camera_init"
        rate = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
        frames_yaml_file = sys.argv[5] if len(sys.argv) > 5 else None
    
    # Auto-detect frames YAML file if not provided
    if frames_yaml_file is None and pcd_file:
        base_name = os.path.splitext(os.path.basename(pcd_file))[0]
        dir_name = os.path.dirname(pcd_file)
        potential_yaml = os.path.join(dir_name, base_name + '_frames.yaml')
        if os.path.exists(potential_yaml):
            frames_yaml_file = potential_yaml
            rospy.loginfo("Auto-detected frames YAML: {}".format(frames_yaml_file))
    
    # Start frame publisher in separate thread if enabled
    frame_thread = None
    if publish_frames_enabled:
        rospy.loginfo("Starting TF frame publisher...")
        frame_thread = threading.Thread(target=publish_frames, args=(frames_yaml_file, 10.0), daemon=True)
        frame_thread.start()
        rospy.loginfo("TF frames will be published: map, camera_init, body")
    
    try:
        publish_pcd(pcd_file, topic_name, frame_id, rate)
    except rospy.ROSInterruptException:
        pass

