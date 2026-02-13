#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Script to save the /Laser_map topic to a PCD file (3D map) and 2D map (z=0 projection).
Also saves cumulative currPoints (accumulated cloud_registered and cloud_registered_body over time, like RViz).
Press SPACE key to save one message, then the script will stop.

Usage:
    rosrun fast_lio save_map.py [output_dir] [filename]
    
    Examples:
    rosrun fast_lio save_map.py                    # Saves to /saved_maps/premap.pcd and premap_scan.pcd
                                                   # (maps to ~/jenn/fastlio/saved_maps on host)
    rosrun fast_lio save_map.py /saved_maps my_map.pcd   # Custom filename
    rosrun fast_lio save_map.py /maps my_map.pcd         # Use /maps instead (maps to ~/jenn/fastlio/fastlio_map)
    
    ROS Parameters:
    ~accumulate_currpoints (bool, default: true): Enable cumulative currPoints saving
    ~currpoints_decay_time (float, default: 1000.0): Time window in seconds for accumulation
    
Note: Default output is /saved_maps (mounted to ~/jenn/fastlio/saved_maps on host)
      Saves 3D map from: /Laser_map (as PCD)
      Saves 2D map using z=0 projection: projects points near ground level to z=0 plane (as PCD)
      Also saves cumulative currPoints (3D + 2D) for cloud_registered and cloud_registered_body
      Single scans are NOT saved, only cumulative currPoints versions
"""

import rospy
import sys
import os
import subprocess
import select
import time
import threading
import math
import yaml
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan

class MapSaver:
    def __init__(self, output_dir=None, filename=None):
        rospy.init_node('laser_map_saver', anonymous=True)
        
        if output_dir is None:
            # Primary: Use /saved_maps (mounted to ~/jenn/fastlio/saved_maps on host)
            self.output_dir = '/saved_maps'
            # Backup: docker_ws/saved_maps (reliably mounted, in case /saved_maps mount fails)
            self.secondary_dir = '/home/mars_ugv/docker_ws/saved_maps'
        else:
            self.output_dir = output_dir
            self.secondary_dir = None
        
        # Default filename for pre-mapping use case
        if filename is None:
            self.filename = 'premap.pcd'
        else:
            self.filename = filename
        
        # Verify and create output directory
        self._verify_output_directory()
        
        self.output_path = os.path.join(self.output_dir, self.filename)
        self.laserscan_pcd_path = os.path.join(self.output_dir, self.filename.replace('.pcd', '_scan.pcd'))
        self.laserscan_yaml_path = os.path.join(self.output_dir, self.filename.replace('.pcd', '_scan.yaml'))
        self.save_requested = False
        self.pcl_process = None
        self.pointcloud_to_laserscan_process = None
        self.latest_map_cloud = None
        self.map_received = False
        self.latest_laserscan = None
        self.laserscan_received = False
        
        # Additional clouds to save
        self.latest_cloud_registered = None
        self.cloud_registered_received = False
        self.latest_cloud_registered_body = None
        self.cloud_registered_body_received = False
        
        # Cumulative currPoints for cloud_registered and cloud_registered_body (like RViz currPoints)
        self.accumulate_currpoints = rospy.get_param('~accumulate_currpoints', True)
        self.currpoints_decay_time = rospy.get_param('~currpoints_decay_time', 1000.0)  # seconds, default 1000s like RViz
        self.accumulated_cloud_registered_points = []  # List of (timestamp, points) tuples
        self.accumulated_cloud_registered_body_points = []  # List of (timestamp, points) tuples
        self.accumulation_start_time = None
        
        # Subscribe to all topics
        rospy.Subscriber('/Laser_map', PointCloud2, self.map_callback)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_registered_callback)
        rospy.Subscriber('/cloud_registered_body', PointCloud2, self.cloud_registered_body_callback)
        rospy.Subscriber('/scan_raw', LaserScan, self.laserscan_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Laser Map Saver initialized")
        rospy.loginfo("Press SPACE key to save the maps (one message only)")
        rospy.loginfo("Will save:")
        rospy.loginfo("  - /Laser_map (3D + 2D)")
        if self.accumulate_currpoints:
            rospy.loginfo("  - Cumulative cloud_registered currPoints (3D + 2D, {}s decay)".format(self.currpoints_decay_time))
            rospy.loginfo("  - Cumulative cloud_registered_body currPoints (3D + 2D, {}s decay)".format(self.currpoints_decay_time))
            rospy.loginfo("    (Accumulating points over time, no single scans will be saved)")
        rospy.loginfo("Output directory: {}".format(self.output_dir))
        if self.output_dir == '/saved_maps':
            rospy.loginfo("Host location: ~/jenn/fastlio/saved_maps/")
        elif self.output_dir == '/maps':
            rospy.loginfo("Host location: ~/jenn/fastlio/fastlio_map/")
        rospy.loginfo("=" * 60)
    
    def _verify_output_directory(self):
        """Verify the output directory exists and is writable, with mount verification"""
        # Try to create directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir, mode=0o755)
                rospy.loginfo("Created output directory: {}".format(self.output_dir))
            except Exception as e:
                rospy.logerr("Failed to create output directory {}: {}".format(self.output_dir, str(e)))
                # Try secondary directory if primary fails
                if self.secondary_dir:
                    rospy.logwarn("Falling back to secondary directory: {}".format(self.secondary_dir))
                    self.output_dir = self.secondary_dir
                    if not os.path.exists(self.output_dir):
                        os.makedirs(self.output_dir, mode=0o755)
        
        # Verify directory is writable by creating a test file
        test_file = os.path.join(self.output_dir, '.write_test')
        try:
            with open(test_file, 'w') as f:
                f.write('test')
            os.remove(test_file)
            rospy.loginfo("✓ Output directory is writable: {}".format(self.output_dir))
        except Exception as e:
            rospy.logerr("✗ Output directory is NOT writable: {} - Error: {}".format(self.output_dir, str(e)))
            # Try secondary directory
            if self.secondary_dir and self.output_dir != self.secondary_dir:
                rospy.logwarn("Falling back to secondary directory: {}".format(self.secondary_dir))
                self.output_dir = self.secondary_dir
                if not os.path.exists(self.output_dir):
                    os.makedirs(self.output_dir, mode=0o755)
                # Test secondary directory
                test_file = os.path.join(self.output_dir, '.write_test')
                try:
                    with open(test_file, 'w') as f:
                        f.write('test')
                    os.remove(test_file)
                    rospy.loginfo("✓ Secondary directory is writable: {}".format(self.output_dir))
                except Exception as e2:
                    rospy.logerr("✗ Secondary directory also NOT writable: {}".format(str(e2)))
        
        # Ensure directory has proper permissions for host access
        try:
            os.chmod(self.output_dir, 0o755)
        except Exception as e:
            rospy.logwarn("Could not set permissions on {}: {}".format(self.output_dir, str(e)))
        
        # Log mount information
        rospy.loginfo("Output directory: {} (should be mounted to host)".format(self.output_dir))
    
    def _sync_and_verify_file(self, file_path):
        """Sync file to disk, set permissions, and verify it's accessible. Returns file size."""
        file_size = 0
        try:
            # Set permissions to ensure host can access
            os.chmod(file_path, 0o644)
            
            # Force file system sync
            subprocess.run(['sync'], check=False, timeout=2)
            
            # Open and close file to ensure it's flushed
            with open(file_path, 'rb') as f:
                f.read(1)  # Touch the file
            
            # Force sync using fsync
            import fcntl
            with open(file_path, 'a') as f:
                os.fsync(f.fileno())
            
            # Small delay to ensure file system sync
            time.sleep(0.2)
            
            # Verify file is readable and get size
            if os.path.exists(file_path):
                file_size = os.path.getsize(file_path)
                with open(file_path, 'rb') as f:
                    test_read = f.read(100)
                if len(test_read) > 0 and file_size > 0:
                    rospy.loginfo("✓ File synced and verified: {} ({} bytes)".format(
                        os.path.basename(file_path), file_size))
                else:
                    rospy.logwarn("⚠ Warning: File exists but may be empty or not fully synced")
            else:
                rospy.logerr("✗ Error: File does not exist after sync: {}".format(file_path))
        except Exception as e:
            rospy.logwarn("Warning during file sync/verification: {}".format(str(e)))
            # Try to get file size anyway
            try:
                if os.path.exists(file_path):
                    file_size = os.path.getsize(file_path)
            except:
                pass
        return file_size
    
    def map_callback(self, msg):
        """Store the latest /Laser_map message"""
        self.latest_map_cloud = msg
        self.map_received = True
        rospy.loginfo_once("Received map from /Laser_map")
    
    def cloud_registered_callback(self, msg):
        """Store the latest /cloud_registered message and accumulate for currPoints"""
        self.latest_cloud_registered = msg
        self.cloud_registered_received = True
        rospy.loginfo_once("Received cloud from /cloud_registered")
        
        # Accumulate points for cumulative currPoints saving
        if self.accumulate_currpoints:
            current_time = rospy.Time.now().to_sec()
            if self.accumulation_start_time is None:
                self.accumulation_start_time = current_time
                rospy.loginfo("Started accumulating cloud_registered points for currPoints...")
            
            # Extract points from the message
            try:
                points = []
                for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                    x, y, z = point[0], point[1], point[2]
                    intensity = point[3] if len(point) > 3 else 0.0
                    points.append([x, y, z, intensity])
                
                # Store with timestamp
                self.accumulated_cloud_registered_points.append((current_time, points))
                
                # Remove old points beyond decay time
                cutoff_time = current_time - self.currpoints_decay_time
                self.accumulated_cloud_registered_points = [
                    (ts, pts) for ts, pts in self.accumulated_cloud_registered_points 
                    if ts >= cutoff_time
                ]
                
            except Exception as e:
                rospy.logwarn("Error accumulating cloud_registered points: {}".format(str(e)))
    
    def cloud_registered_body_callback(self, msg):
        """Store the latest /cloud_registered_body message and accumulate for currPoints"""
        self.latest_cloud_registered_body = msg
        self.cloud_registered_body_received = True
        rospy.loginfo_once("Received cloud from /cloud_registered_body")
        
        # Accumulate points for cumulative currPoints saving
        if self.accumulate_currpoints:
            current_time = rospy.Time.now().to_sec()
            if self.accumulation_start_time is None:
                self.accumulation_start_time = current_time
                rospy.loginfo("Started accumulating cloud_registered_body points for currPoints...")
            
            # Extract points from the message
            try:
                points = []
                for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                    x, y, z = point[0], point[1], point[2]
                    intensity = point[3] if len(point) > 3 else 0.0
                    points.append([x, y, z, intensity])
                
                # Store with timestamp
                self.accumulated_cloud_registered_body_points.append((current_time, points))
                
                # Remove old points beyond decay time
                cutoff_time = current_time - self.currpoints_decay_time
                self.accumulated_cloud_registered_body_points = [
                    (ts, pts) for ts, pts in self.accumulated_cloud_registered_body_points 
                    if ts >= cutoff_time
                ]
                
            except Exception as e:
                rospy.logwarn("Error accumulating cloud_registered_body points: {}".format(str(e)))
    
    def laserscan_callback(self, msg):
        """Store the latest LaserScan message from pointcloud_to_laserscan"""
        self.latest_laserscan = msg
        self.laserscan_received = True
        rospy.loginfo_once("Received LaserScan from /scan_raw")
    
    def save_pointcloud2_to_3d_pcd(self, msg, output_path):
        """Save PointCloud2 message to 3D PCD file"""
        if msg is None:
            return False
        
        # Remove old file if it exists
        if os.path.exists(output_path):
            try:
                os.remove(output_path)
            except:
                pass
        
        try:
            # Read all points from PointCloud2
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                intensity = point[3] if len(point) > 3 else 0.0
                points.append([x, y, z, intensity])
            
            if len(points) == 0:
                rospy.logwarn("No points found in point cloud")
                return False
            
            # Get frame_id from message header
            frame_id = msg.header.frame_id if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id') else "unknown"
            
            # Write PCD file
            with open(output_path, 'w') as f:
                # Write PCD header with frame information
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("# FRAME_ID: {}\n".format(frame_id))
                f.write("# MAP_FRAME: map (same as camera_init in FAST-LIO)\n")
                f.write("# CAMERA_INIT_FRAME: camera_init (FAST-LIO global coordinate frame)\n")
                f.write("# TIMESTAMP: {}.{}\n".format(msg.header.stamp.secs, msg.header.stamp.nsecs))
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write("WIDTH {}\n".format(len(points)))
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write("POINTS {}\n".format(len(points)))
                f.write("DATA ascii\n")
                
                # Write point data
                for pt in points:
                    f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(pt[0], pt[1], pt[2], pt[3]))
            
            # Ensure file is synced to disk and accessible
            file_size = self._sync_and_verify_file(output_path)
            
            # Also copy to backup location
            if self.secondary_dir and os.path.exists(self.secondary_dir) and self.output_dir != self.secondary_dir:
                secondary_path = os.path.join(self.secondary_dir, os.path.basename(output_path))
                try:
                    import shutil
                    shutil.copy2(output_path, secondary_path)
                    os.chmod(secondary_path, 0o644)
                    subprocess.run(['sync'], check=False, timeout=2)
                except Exception as e:
                    rospy.logwarn("Could not save backup: {}".format(str(e)))
            
            rospy.loginfo("3D PCD saved: {} ({} points, frame: {})".format(os.path.basename(output_path), len(points), frame_id))
            return True
            
        except Exception as e:
            rospy.logerr("Error saving 3D PCD to {}: {}".format(output_path, str(e)))
            return False
    
    def save_accumulated_currpoints(self, accumulated_points_list, output_path_3d, output_path_2d, name="currPoints"):
        """Save accumulated points (like RViz currPoints)"""
        if not self.accumulate_currpoints or len(accumulated_points_list) == 0:
            rospy.logwarn("No accumulated {} to save".format(name))
            return False
        
        try:
            # Combine all accumulated points
            all_points = []
            for timestamp, points in accumulated_points_list:
                all_points.extend(points)
            
            if len(all_points) == 0:
                rospy.logwarn("No points in accumulated currPoints")
                return False
            
            rospy.loginfo("Saving accumulated {}: {} points from {} scans".format(
                name, len(all_points), len(accumulated_points_list)))
            
            # Save 3D version
            if os.path.exists(output_path_3d):
                try:
                    os.remove(output_path_3d)
                except:
                    pass
            
            # Get frame_id (use appropriate one based on name)
            if "cloud_registered" in name and "body" not in name:
                if self.latest_cloud_registered and hasattr(self.latest_cloud_registered, 'header'):
                    frame_id = self.latest_cloud_registered.header.frame_id
                else:
                    frame_id = "camera_init"  # Default FAST-LIO frame
            elif "cloud_registered_body" in name:
                if self.latest_cloud_registered_body and hasattr(self.latest_cloud_registered_body, 'header'):
                    frame_id = self.latest_cloud_registered_body.header.frame_id
                else:
                    frame_id = "body"  # Default body frame
            else:
                frame_id = "unknown"
            
            with open(output_path_3d, 'w') as f:
                # Write PCD header with frame information
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("# FRAME_ID: {}\n".format(frame_id))
                f.write("# TYPE: Accumulated currPoints ({} scans)\n".format(len(accumulated_points_list)))
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write("WIDTH {}\n".format(len(all_points)))
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write("POINTS {}\n".format(len(all_points)))
                f.write("DATA ascii\n")
                
                # Write point data
                for pt in all_points:
                    f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(pt[0], pt[1], pt[2], pt[3]))
            
            file_size = self._sync_and_verify_file(output_path_3d)
            
            # Save 2D version (z=0 slice)
            points_2d = []
            z_threshold = 0.5
            for pt in all_points:
                x, y, z = pt[0], pt[1], pt[2]
                if abs(z) <= z_threshold:
                    intensity = pt[3] if len(pt) > 3 else 0.0
                    points_2d.append([x, y, 0.0, intensity])
            
            if len(points_2d) > 0:
                if os.path.exists(output_path_2d):
                    try:
                        os.remove(output_path_2d)
                    except:
                        pass
                
                with open(output_path_2d, 'w') as f:
                    # Write PCD header with frame information
                    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                    f.write("# FRAME_ID: {}\n".format(frame_id))
                    f.write("# TYPE: Accumulated currPoints 2D (z=0 projection, {} scans)\n".format(len(accumulated_points_list)))
                    f.write("VERSION 0.7\n")
                    f.write("FIELDS x y z intensity\n")
                    f.write("SIZE 4 4 4 4\n")
                    f.write("TYPE F F F F\n")
                    f.write("COUNT 1 1 1 1\n")
                    f.write("WIDTH {}\n".format(len(points_2d)))
                    f.write("HEIGHT 1\n")
                    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                    f.write("POINTS {}\n".format(len(points_2d)))
                    f.write("DATA ascii\n")
                    
                    # Write point data
                    for pt in points_2d:
                        f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(pt[0], pt[1], pt[2], pt[3]))
                
                self._sync_and_verify_file(output_path_2d)
                rospy.loginfo("2D currPoints saved: {} points (frame: {})".format(len(points_2d), frame_id))
            
            # Backup to secondary location
            if self.secondary_dir and os.path.exists(self.secondary_dir) and self.output_dir != self.secondary_dir:
                try:
                    import shutil
                    shutil.copy2(output_path_3d, os.path.join(self.secondary_dir, os.path.basename(output_path_3d)))
                    if len(points_2d) > 0:
                        shutil.copy2(output_path_2d, os.path.join(self.secondary_dir, os.path.basename(output_path_2d)))
                except Exception as e:
                    rospy.logwarn("Could not save currPoints backup: {}".format(str(e)))
            
            rospy.loginfo("Cumulative {} saved:".format(name))
            rospy.loginfo("  3D: {} ({} points, frame: {})".format(os.path.basename(output_path_3d), len(all_points), frame_id))
            if len(points_2d) > 0:
                rospy.loginfo("  2D: {} ({} points, frame: {})".format(os.path.basename(output_path_2d), len(points_2d), frame_id))
            
            return True
            
        except Exception as e:
            rospy.logerr("Error saving accumulated currPoints: {}".format(str(e)))
            return False
    
    def save_pointcloud2_to_2d_pcd(self, msg, output_path):
        """Save PointCloud2 message to 2D slice PCD file (z=0 plane)"""
        if msg is None:
            return False
        
        # Remove old file if it exists
        if os.path.exists(output_path):
            try:
                os.remove(output_path)
            except:
                pass
        
        try:
            # Convert PointCloud2 to 2D slice (project points to z=0 plane)
            points = []
            z_threshold = 0.5  # Points within ±0.5m of z=0 (ground level)
            
            for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                # Only keep points near ground level (z ≈ 0)
                if abs(z) <= z_threshold:
                    # Project to z=0
                    intensity = point[3] if len(point) > 3 else 0.0
                    points.append([x, y, 0.0, intensity])
            
            if len(points) == 0:
                rospy.logwarn("No points near ground level (z=0) found")
                return False
            
            # Write PCD file
            with open(output_path, 'w') as f:
                # Write PCD header
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write("WIDTH {}\n".format(len(points)))
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write("POINTS {}\n".format(len(points)))
                f.write("DATA ascii\n")
                
                # Write point data
                for pt in points:
                    f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(pt[0], pt[1], pt[2], pt[3]))
            
            # Ensure file is synced to disk and accessible
            self._sync_and_verify_file(output_path)
            
            # Also copy to backup location
            if self.secondary_dir and os.path.exists(self.secondary_dir) and self.output_dir != self.secondary_dir:
                secondary_path = os.path.join(self.secondary_dir, os.path.basename(output_path))
                try:
                    import shutil
                    shutil.copy2(output_path, secondary_path)
                    os.chmod(secondary_path, 0o644)
                    subprocess.run(['sync'], check=False, timeout=2)
                except Exception as e:
                    rospy.logwarn("Could not save backup: {}".format(str(e)))
            
            rospy.loginfo("2D PCD saved: {} ({} points, projected to z=0)".format(os.path.basename(output_path), len(points)))
            return True
            
        except Exception as e:
            rospy.logerr("Error saving 2D PCD to {}: {}".format(output_path, str(e)))
            return False
    
    def save_2d_map(self):
        """Convert point cloud to 2D slice (z=0 projection) and save as PCD"""
        rospy.loginfo("Saving 2D map using z=0 projection...")
        return self.pointcloud2_to_2d_pcd()
    
    def _start_pointcloud_to_laserscan(self):
        """Start pointcloud_to_laserscan node (ROS1 version)"""
        try:
            # Try ROS1 node name first (most common)
            # In ROS1, the node is typically 'pointcloud_to_laserscan' or 'pointcloud_to_laserscan_node'
            cmd = [
                'rosrun', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan',
                '_cloud_in:=/cloud_registered_body',
                '_scan:=/scan_raw',
                '_target_frame:=odom',
                '_transform_tolerance:=0.01',
                '_min_height:=0.2',
                '_max_height:=0.5',
                '_angle_min:=-1.57',
                '_angle_max:=1.57',
                '_angle_increment:=0.01',
                '_scan_time:=1.0',
                '_range_min:=0.15',
                '_range_max:=20.0',
                '_use_inf:=true',
                '_inf_epsilon:=1.0'
            ]
            
            self.pointcloud_to_laserscan_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Give it a moment to start
            time.sleep(2.0)
            
            # Check if process is still running
            if self.pointcloud_to_laserscan_process.poll() is not None:
                stderr = self.pointcloud_to_laserscan_process.stderr.read().decode()
                rospy.logerr("pointcloud_to_laserscan failed to start: {}".format(stderr))
                # Try alternative node name (some ROS1 versions use different name)
                rospy.loginfo("Trying alternative node name 'pointcloud_to_laserscan_node'...")
                try:
                    cmd_alt = [
                        'rosrun', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
                        '_cloud_in:=/cloud_registered_body',
                        '_scan:=/scan_raw',
                        '_target_frame:=odom',
                        '_transform_tolerance:=0.01',
                        '_min_height:=0.2',
                        '_max_height:=0.5',
                        '_angle_min:=-1.57',
                        '_angle_max:=1.57',
                        '_angle_increment:=0.01',
                        '_scan_time:=1.0',
                        '_range_min:=0.15',
                        '_range_max:=20.0',
                        '_use_inf:=true',
                        '_inf_epsilon:=1.0'
                    ]
                    self.pointcloud_to_laserscan_process = subprocess.Popen(
                        cmd_alt,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    time.sleep(2.0)
                    if self.pointcloud_to_laserscan_process.poll() is not None:
                        rospy.logwarn("Both node names failed. Package may not be installed.")
                        rospy.loginfo("Install with: sudo apt-get install ros-noetic-pointcloud-to-laserscan")
                        rospy.loginfo("Or clone from: https://github.com/ros-perception/pointcloud_to_laserscan")
                        return False
                except Exception as e2:
                    rospy.logwarn("Alternative node name also failed: {}".format(str(e2)))
                    return False
            
            rospy.loginfo("pointcloud_to_laserscan node started successfully")
            return True
            
        except Exception as e:
            rospy.logerr("Error starting pointcloud_to_laserscan: {}".format(str(e)))
            rospy.loginfo("Install with: sudo apt-get install ros-noetic-pointcloud-to-laserscan")
            rospy.loginfo("Or clone from: https://github.com/ros-perception/pointcloud_to_laserscan")
            return False
    
    def _stop_pointcloud_to_laserscan(self):
        """Stop pointcloud_to_laserscan node"""
        if self.pointcloud_to_laserscan_process and self.pointcloud_to_laserscan_process.poll() is None:
            try:
                self.pointcloud_to_laserscan_process.terminate()
                time.sleep(0.5)
                if self.pointcloud_to_laserscan_process.poll() is None:
                    self.pointcloud_to_laserscan_process.kill()
                rospy.loginfo("pointcloud_to_laserscan node stopped")
            except Exception as e:
                rospy.logwarn("Error stopping pointcloud_to_laserscan: {}".format(str(e)))
    
    def _save_laserscan_to_yaml(self):
        """Save LaserScan message to YAML file"""
        if self.latest_laserscan is None:
            return False
        
        try:
            # Remove old file if exists
            if os.path.exists(self.laserscan_yaml_path):
                try:
                    os.remove(self.laserscan_yaml_path)
                except:
                    pass
            
            # Convert LaserScan to dictionary for YAML
            laserscan_dict = {
                'header': {
                    'seq': self.latest_laserscan.header.seq,
                    'stamp': {
                        'secs': self.latest_laserscan.header.stamp.secs,
                        'nsecs': self.latest_laserscan.header.stamp.nsecs
                    },
                    'frame_id': self.latest_laserscan.header.frame_id
                },
                'angle_min': self.latest_laserscan.angle_min,
                'angle_max': self.latest_laserscan.angle_max,
                'angle_increment': self.latest_laserscan.angle_increment,
                'time_increment': self.latest_laserscan.time_increment,
                'scan_time': self.latest_laserscan.scan_time,
                'range_min': self.latest_laserscan.range_min,
                'range_max': self.latest_laserscan.range_max,
                'ranges': list(self.latest_laserscan.ranges),
                'intensities': list(self.latest_laserscan.intensities)
            }
            
            # Write YAML file
            with open(self.laserscan_yaml_path, 'w') as f:
                yaml.dump(laserscan_dict, f, default_flow_style=False, allow_unicode=True)
            
            # Ensure file is synced
            self._sync_and_verify_file(self.laserscan_yaml_path)
            
            # Backup to secondary location
            if self.secondary_dir and os.path.exists(self.secondary_dir) and self.output_dir != self.secondary_dir:
                secondary_path = os.path.join(self.secondary_dir, os.path.basename(self.laserscan_yaml_path))
                try:
                    import shutil
                    shutil.copy2(self.laserscan_yaml_path, secondary_path)
                    os.chmod(secondary_path, 0o644)
                    subprocess.run(['sync'], check=False, timeout=2)
                except Exception as e:
                    rospy.logwarn("Could not save LaserScan backup: {}".format(str(e)))
            
            rospy.loginfo("2D LaserScan saved to {}".format(self.laserscan_yaml_path))
            rospy.loginfo("  Ranges: {} points".format(len(self.latest_laserscan.ranges)))
            rospy.loginfo("  Frame: {}".format(self.latest_laserscan.header.frame_id))
            if self.output_dir == '/saved_maps':
                rospy.loginfo("  Host: ~/jenn/fastlio/saved_maps/{}".format(os.path.basename(self.laserscan_yaml_path)))
            elif self.output_dir == '/maps':
                rospy.loginfo("  Host: ~/jenn/fastlio/fastlio_map/{}".format(os.path.basename(self.laserscan_yaml_path)))
            
            return True
            
        except Exception as e:
            rospy.logerr("Error saving LaserScan to YAML: {}".format(str(e)))
            return False
    
    def pointcloud2_to_2d_pcd(self):
        """Convert PointCloud2 to 2D slice (z=0 plane) and save as PCD"""
        if not self.map_received or self.latest_map_cloud is None:
            return False
        
        # Remove old 2D map if it exists (keep only latest)
        if os.path.exists(self.laserscan_pcd_path):
            try:
                os.remove(self.laserscan_pcd_path)
            except:
                pass
        
        try:
            # Convert PointCloud2 to 2D slice (project points to z=0 plane)
            points = []
            z_threshold = 0.5  # Points within ±0.5m of z=0 (ground level)
            
            for point in pc2.read_points(self.latest_map_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                # Only keep points near ground level (z ≈ 0)
                if abs(z) <= z_threshold:
                    # Project to z=0
                    intensity = point[3] if len(point) > 3 else 0.0
                    points.append([x, y, 0.0, intensity])
            
            if len(points) == 0:
                rospy.logwarn("No points near ground level (z=0) found in map")
                return False
            
            # Get frame_id from message header
            frame_id = self.latest_map_cloud.header.frame_id if hasattr(self.latest_map_cloud, 'header') and hasattr(self.latest_map_cloud.header, 'frame_id') else "unknown"
            
            # Write PCD file
            with open(self.laserscan_pcd_path, 'w') as f:
                # Write PCD header with frame information
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("# FRAME_ID: {}\n".format(frame_id))
                f.write("# MAP_FRAME: map (same as camera_init in FAST-LIO)\n")
                f.write("# CAMERA_INIT_FRAME: camera_init (FAST-LIO global coordinate frame)\n")
                f.write("# TIMESTAMP: {}.{}\n".format(self.latest_map_cloud.header.stamp.secs, self.latest_map_cloud.header.stamp.nsecs))
                f.write("# PROJECTION: z=0 (2D slice from 3D map)\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z intensity\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                f.write("WIDTH {}\n".format(len(points)))
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write("POINTS {}\n".format(len(points)))
                f.write("DATA ascii\n")
                
                # Write point data
                for pt in points:
                    f.write("{:.6f} {:.6f} {:.6f} {:.6f}\n".format(pt[0], pt[1], pt[2], pt[3]))
            
            # Ensure file is synced to disk and accessible
            self._sync_and_verify_file(self.laserscan_pcd_path)
            
            # Also copy to backup location (docker_ws/saved_maps) in case primary mount fails
            if self.secondary_dir and os.path.exists(self.secondary_dir) and self.output_dir != self.secondary_dir:
                secondary_scan_path = os.path.join(self.secondary_dir, os.path.basename(self.laserscan_pcd_path))
                try:
                    import shutil
                    shutil.copy2(self.laserscan_pcd_path, secondary_scan_path)
                    os.chmod(secondary_scan_path, 0o644)
                    subprocess.run(['sync'], check=False, timeout=2)
                    rospy.loginfo("2D map backup saved to: {}".format(secondary_scan_path))
                except Exception as e:
                    rospy.logwarn("Could not save 2D map backup: {}".format(str(e)))
            
            rospy.loginfo("2D Map PCD saved to {}".format(self.laserscan_pcd_path))
            rospy.loginfo("  Points: {} (projected to z=0, frame: {})".format(len(points), frame_id))
            if self.output_dir == '/saved_maps':
                rospy.loginfo("  Host: ~/jenn/fastlio/saved_maps/{}".format(os.path.basename(self.laserscan_pcd_path)))
            elif self.output_dir == '/maps':
                rospy.loginfo("  Host: ~/jenn/fastlio/fastlio_map/{}".format(os.path.basename(self.laserscan_pcd_path)))
            else:
                rospy.loginfo("  Container: {}".format(self.laserscan_pcd_path))
            return True
            
        except Exception as e:
            rospy.logerr("Error saving 2D map PCD: {}".format(str(e)))
            return False
    
    def check_keyboard(self):
        """Check for keyboard input in a non-blocking way"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ' or key == '\n':
                return True
        return False
    
    def save_with_pcl_ros(self):
        """Use pcl_ros pointcloud_to_pcd to save one message"""
        # Create a temporary prefix that will generate one file
        # We'll use a timestamp-based prefix and monitor for file creation
        import time
        timestamp = int(time.time())
        temp_prefix = os.path.join(self.output_dir, "temp_map_{}_".format(timestamp))
        
        rospy.loginfo("Saving map using pcl_ros pointcloud_to_pcd...")
        rospy.loginfo("Waiting for one message from /Laser_map...")
        
        # Start pcl_ros pointcloud_to_pcd
        # We'll use a timeout or monitor for file creation
        cmd = [
            'rosrun', 'pcl_ros', 'pointcloud_to_pcd',
            'input:=/Laser_map',
            '_prefix:={}'.format(temp_prefix)
        ]
        
        try:
            # Start the process
            self.pcl_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for one file to be created (check every 0.1 seconds)
            max_wait = 10.0  # Maximum 10 seconds to wait
            wait_time = 0.0
            check_interval = 0.1
            
            while wait_time < max_wait:
                # Check if process has finished (error case)
                if self.pcl_process.poll() is not None:
                    stderr = self.pcl_process.stderr.read().decode()
                    if stderr:
                        rospy.logwarn("pcl_ros process ended: {}".format(stderr))
                    break
                
                # Check for created files
                import glob
                files = glob.glob(temp_prefix + "*.pcd")
                if len(files) > 0:
                    # Found a file! Rename it to the desired filename
                    first_file = sorted(files)[0]
                    if os.path.exists(self.output_path):
                        os.remove(self.output_path)  # Remove old file if exists
                    os.rename(first_file, self.output_path)
                    
                    # Clean up any remaining temp files (pcl_ros may create multiple before we stop it)
                    remaining_files = glob.glob(temp_prefix + "*.pcd")
                    for temp_file in remaining_files:
                        try:
                            # Only remove if it's not the file we just renamed
                            if temp_file != first_file and os.path.exists(temp_file):
                                os.remove(temp_file)
                        except:
                            pass
                    
                    # Ensure file is synced to disk and accessible
                    file_size = self._sync_and_verify_file(self.output_path)
                    
                    # Also copy to backup location (docker_ws/saved_maps) in case primary mount fails
                    if self.secondary_dir and os.path.exists(self.secondary_dir) and self.output_dir != self.secondary_dir:
                        secondary_path = os.path.join(self.secondary_dir, self.filename)
                        try:
                            import shutil
                            shutil.copy2(self.output_path, secondary_path)
                            os.chmod(secondary_path, 0o644)
                            subprocess.run(['sync'], check=False, timeout=2)
                            rospy.loginfo("Backup saved to: {}".format(secondary_path))
                        except Exception as e:
                            rospy.logwarn("Could not save backup: {}".format(str(e)))
                    
                    # Verify file is accessible
                    if file_size > 0:
                        rospy.loginfo("=" * 60)
                        rospy.loginfo("PCD Map saved successfully!")
                        rospy.loginfo("File: {}".format(self.output_path))
                        rospy.loginfo("Size: {:.2f} MB".format(file_size / (1024*1024)))
                        rospy.loginfo("File saved to:")
                        rospy.loginfo("  Container: {}".format(self.output_path))
                        if self.output_dir == '/saved_maps':
                            rospy.loginfo("  Host: ~/jenn/fastlio/saved_maps/{}".format(self.filename))
                        elif self.output_dir == '/maps':
                            rospy.loginfo("  Host: ~/jenn/fastlio/fastlio_map/{}".format(self.filename))
                        else:
                            rospy.loginfo("  Host: Check mounted volume for {}".format(self.output_dir))
                    else:
                        rospy.logwarn("Warning: File may not be accessible on host")
                    
                    # Also save 2D map (from same /Laser_map topic)
                    rospy.loginfo("Saving 2D map (z=0 slice) as PCD...")
                    self.save_2d_map()
                    
                    # Save cumulative currPoints for cloud_registered and cloud_registered_body
                    base_name = self.filename.replace('.pcd', '')
                    
                    if self.accumulate_currpoints:
                        # Save cumulative cloud_registered currPoints (3D and 2D)
                        rospy.loginfo("Saving cumulative cloud_registered currPoints...")
                        cloud_registered_currpoints_3d_path = os.path.join(self.output_dir, base_name + '_cloud_registered_currpoints.pcd')
                        cloud_registered_currpoints_2d_path = os.path.join(self.output_dir, base_name + '_cloud_registered_currpoints_scan.pcd')
                        self.save_accumulated_currpoints(
                            self.accumulated_cloud_registered_points,
                            cloud_registered_currpoints_3d_path,
                            cloud_registered_currpoints_2d_path,
                            "cloud_registered currPoints"
                        )
                        
                        # Save cumulative cloud_registered_body currPoints (3D and 2D)
                        rospy.loginfo("Saving cumulative cloud_registered_body currPoints...")
                        cloud_registered_body_currpoints_3d_path = os.path.join(self.output_dir, base_name + '_cloud_registered_body_currpoints.pcd')
                        cloud_registered_body_currpoints_2d_path = os.path.join(self.output_dir, base_name + '_cloud_registered_body_currpoints_scan.pcd')
                        self.save_accumulated_currpoints(
                            self.accumulated_cloud_registered_body_points,
                            cloud_registered_body_currpoints_3d_path,
                            cloud_registered_body_currpoints_2d_path,
                            "cloud_registered_body currPoints"
                        )
                    else:
                        rospy.logwarn("currPoints accumulation disabled, skipping cumulative saves")
                    
                    rospy.loginfo("=" * 60)
                    
                    # Stop the process
                    self.pcl_process.terminate()
                    time.sleep(0.5)
                    if self.pcl_process.poll() is None:
                        self.pcl_process.kill()
                    
                    return True
                
                time.sleep(check_interval)
                wait_time += check_interval
            
            # If we get here, no file was created in time
            rospy.logwarn("No map message received within {} seconds".format(max_wait))
            if self.pcl_process.poll() is None:
                self.pcl_process.terminate()
                time.sleep(0.5)
                if self.pcl_process.poll() is None:
                    self.pcl_process.kill()
            return False
            
        except Exception as e:
            rospy.logerr("Error saving map: {}".format(str(e)))
            if self.pcl_process and self.pcl_process.poll() is None:
                self.pcl_process.terminate()
            return False
    
    def run(self):
        """Main loop - wait for space key, then save"""
        # Set stdin to non-blocking mode
        import termios, tty
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            rospy.loginfo("Waiting for SPACE key press...")
            
            while not rospy.is_shutdown():
                # Check for keyboard input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == ' ':
                        rospy.loginfo("\nSPACE pressed! Saving map...")
                        self.save_with_pcl_ros()
                        break
                    elif key == '\x03':  # Ctrl+C
                        rospy.loginfo("\nInterrupted by user")
                        break
                
                # Small sleep to avoid busy waiting
                rospy.sleep(0.05)
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
            # Clean up processes if still running
            if self.pcl_process and self.pcl_process.poll() is None:
                self.pcl_process.terminate()
                time.sleep(0.5)
                if self.pcl_process.poll() is None:
                    self.pcl_process.kill()
            
            # Clean up pointcloud_to_laserscan process
            self._stop_pointcloud_to_laserscan()

if __name__ == '__main__':
    output_dir = None
    filename = None
    
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    if len(sys.argv) > 2:
        filename = sys.argv[2]
    
    try:
        saver = MapSaver(output_dir, filename)
        saver.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\nExiting...")
