#!/bin/bash
# Script to view saved maps in RViz
# Usage: ./view_maps.sh [map_directory] [3d_map_file] [2d_map_file] [cloud_registered_currpoints_3d] [cloud_registered_currpoints_2d] [cloud_registered_body_currpoints_3d] [cloud_registered_body_currpoints_2d]
# 
# Examples:
#   ./view_maps.sh                                    # Use all defaults
#   ./view_maps.sh /saved_maps premap.pcd premap_scan.pcd  # Just Laser_map
#   ./view_maps.sh /saved_maps premap.pcd premap_scan.pcd premap_cloud_registered_currpoints.pcd premap_cloud_registered_currpoints_scan.pcd premap_cloud_registered_body_currpoints.pcd premap_cloud_registered_body_currpoints_scan.pcd  # All topics

# Default values
MAP_DIR="${1:-/saved_maps}"
MAP_3D="${2:-premap.pcd}"
MAP_2D="${3:-premap_scan.pcd}"
CLOUD_REGISTERED_CURRPOINTS_3D="${4:-premap_cloud_registered_currpoints.pcd}"
CLOUD_REGISTERED_CURRPOINTS_2D="${5:-premap_cloud_registered_currpoints_scan.pcd}"
CLOUD_REGISTERED_BODY_CURRPOINTS_3D="${6:-premap_cloud_registered_body_currpoints.pcd}"
CLOUD_REGISTERED_BODY_CURRPOINTS_2D="${7:-premap_cloud_registered_body_currpoints_scan.pcd}"

echo "=========================================="
echo "Viewing saved maps in RViz"
echo "=========================================="
echo "Map directory: $MAP_DIR"
echo ""
echo "Laser_map:"
echo "  3D: $MAP_DIR/$MAP_3D"
echo "  2D: $MAP_DIR/$MAP_2D"
echo ""
echo "cloud_registered currPoints:"
echo "  3D: $MAP_DIR/$CLOUD_REGISTERED_CURRPOINTS_3D"
echo "  2D: $MAP_DIR/$CLOUD_REGISTERED_CURRPOINTS_2D"
echo ""
echo "cloud_registered_body currPoints:"
echo "  3D: $MAP_DIR/$CLOUD_REGISTERED_BODY_CURRPOINTS_3D"
echo "  2D: $MAP_DIR/$CLOUD_REGISTERED_BODY_CURRPOINTS_2D"
echo "=========================================="

# Check which files exist and set flags accordingly
PUBLISH_LASER_MAP="true"
PUBLISH_CLOUD_REGISTERED_CURRPOINTS="true"
PUBLISH_CLOUD_REGISTERED_BODY_CURRPOINTS="true"

if [ ! -f "$MAP_DIR/$MAP_3D" ]; then
    echo "WARNING: Laser_map 3D file not found: $MAP_DIR/$MAP_3D"
    PUBLISH_LASER_MAP="false"
fi

if [ ! -f "$MAP_DIR/$MAP_2D" ]; then
    echo "WARNING: Laser_map 2D file not found: $MAP_DIR/$MAP_2D"
    if [ "$PUBLISH_LASER_MAP" = "true" ]; then
        echo "  (Will still publish 3D Laser_map)"
    fi
fi

if [ ! -f "$MAP_DIR/$CLOUD_REGISTERED_CURRPOINTS_3D" ]; then
    echo "WARNING: cloud_registered currPoints 3D file not found: $MAP_DIR/$CLOUD_REGISTERED_CURRPOINTS_3D"
    PUBLISH_CLOUD_REGISTERED_CURRPOINTS="false"
fi

if [ ! -f "$MAP_DIR/$CLOUD_REGISTERED_CURRPOINTS_2D" ]; then
    echo "WARNING: cloud_registered currPoints 2D file not found: $MAP_DIR/$CLOUD_REGISTERED_CURRPOINTS_2D"
    if [ "$PUBLISH_CLOUD_REGISTERED_CURRPOINTS" = "true" ]; then
        echo "  (Will still publish 3D cloud_registered currPoints)"
    fi
fi

if [ ! -f "$MAP_DIR/$CLOUD_REGISTERED_BODY_CURRPOINTS_3D" ]; then
    echo "WARNING: cloud_registered_body currPoints 3D file not found: $MAP_DIR/$CLOUD_REGISTERED_BODY_CURRPOINTS_3D"
    PUBLISH_CLOUD_REGISTERED_BODY_CURRPOINTS="false"
fi

if [ ! -f "$MAP_DIR/$CLOUD_REGISTERED_BODY_CURRPOINTS_2D" ]; then
    echo "WARNING: cloud_registered_body currPoints 2D file not found: $MAP_DIR/$CLOUD_REGISTERED_BODY_CURRPOINTS_2D"
    if [ "$PUBLISH_CLOUD_REGISTERED_BODY_CURRPOINTS" = "true" ]; then
        echo "  (Will still publish 3D cloud_registered_body currPoints)"
    fi
fi

# Check if at least one file exists
if [ "$PUBLISH_LASER_MAP" = "false" ] && [ "$PUBLISH_CLOUD_REGISTERED_CURRPOINTS" = "false" ] && [ "$PUBLISH_CLOUD_REGISTERED_BODY_CURRPOINTS" = "false" ]; then
    echo "ERROR: No map files found! Please check the file paths."
    exit 1
fi

echo ""
echo "Publishing topics:"
[ "$PUBLISH_LASER_MAP" = "true" ] && echo "  ✓ Laser_map (3D + 2D)"
[ "$PUBLISH_CLOUD_REGISTERED_CURRPOINTS" = "true" ] && echo "  ✓ cloud_registered currPoints (3D + 2D)"
[ "$PUBLISH_CLOUD_REGISTERED_BODY_CURRPOINTS" = "true" ] && echo "  ✓ cloud_registered_body currPoints (3D + 2D)"
echo "=========================================="

# Launch the visualization
roslaunch fast_lio view_saved_maps.launch \
    map_dir:="$MAP_DIR" \
    map_3d_file:="$MAP_3D" \
    map_2d_file:="$MAP_2D" \
    cloud_registered_currpoints_3d_file:="$CLOUD_REGISTERED_CURRPOINTS_3D" \
    cloud_registered_currpoints_2d_file:="$CLOUD_REGISTERED_CURRPOINTS_2D" \
    cloud_registered_body_currpoints_3d_file:="$CLOUD_REGISTERED_BODY_CURRPOINTS_3D" \
    cloud_registered_body_currpoints_2d_file:="$CLOUD_REGISTERED_BODY_CURRPOINTS_2D" \
    publish_laser_map:="$PUBLISH_LASER_MAP" \
    publish_cloud_registered_currpoints:="$PUBLISH_CLOUD_REGISTERED_CURRPOINTS" \
    publish_cloud_registered_body_currpoints:="$PUBLISH_CLOUD_REGISTERED_BODY_CURRPOINTS" \
    rviz:=true

