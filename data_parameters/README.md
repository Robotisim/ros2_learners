# KITTI Node

A ROS2 package for processing KITTI dataset camera and LiDAR data, providing Canny edge detection for images and voxel filtering with clustering for point clouds.

## Overview

This package contains two main nodes:

- **camera_node**: Processes KITTI camera images with Canny edge detection
- **lidar_node**: Processes KITTI point cloud data with voxel filtering and Euclidean clustering

## Prerequisites

Before running the nodes, ensure you have the following dependencies installed:

### System Dependencies

```bash
sudo apt update
sudo apt install libpcl-dev libopencv-dev
```

### ROS2 Dependencies

- `rclcpp`
- `sensor_msgs`
- `pcl_conversions`
- `pcl_ros`
- `cv_bridge`
- `image_transport`

## Building

1. Navigate to your ROS2 workspace:

```bash
cd ~/ros2_ws
```

1. Build the package:

```bash
colcon build --packages-select kitti_node
```

1. Source the workspace:

```bash
source install/setup.bash
```

## Nodes

### Camera Node

Processes KITTI camera images and applies Canny edge detection.

#### Camera Subscribed Topics

- `/kitti/image/color/left` (sensor_msgs/Image): Input color camera images

#### Camera Published Topics

- `/kitti/image/canny/left` (sensor_msgs/Image): Canny edge-detected images

#### Camera Parameters

- `canny_threshold1` (double, default: 100.0): Lower threshold for Canny edge detection
- `canny_threshold2` (double, default: 200.0): Upper threshold for Canny edge detection
- `canny_aperture_size` (int, default: 3): Aperture size for Sobel operator in Canny detection

#### Running the Camera Node

```bash
ros2 run kitti_node camera_node
```

#### Running Camera Node with Custom Parameters

```bash
ros2 run kitti_node camera_node --ros-args -p canny_threshold1:=50.0 -p canny_threshold2:=150.0 -p canny_aperture_size:=5
```

### LiDAR Node

Processes KITTI point cloud data with voxel grid filtering and Euclidean clustering.

#### LiDAR Subscribed Topics

- `kitti/point_cloud` (sensor_msgs/PointCloud2): Input point cloud data

#### LiDAR Published Topics

- `kitti/voxel_cloud` (sensor_msgs/PointCloud2): Voxel-filtered point cloud
- `kitti/clustered_cloud` (sensor_msgs/PointCloud2): Clustered point cloud

#### LiDAR Parameters

- `leaf_size_x` (double, default: 0.2): Voxel grid leaf size in X direction (meters)
- `leaf_size_y` (double, default: 0.2): Voxel grid leaf size in Y direction (meters)
- `leaf_size_z` (double, default: 0.2): Voxel grid leaf size in Z direction (meters)
- `cluster_tolerance` (double, default: 0.5): Clustering tolerance distance (meters)
- `min_cluster_size` (int, default: 50): Minimum number of points per cluster
- `max_cluster_size` (int, default: 5000): Maximum number of points per cluster

#### Running the LiDAR Node

```bash
ros2 run kitti_node lidar_node
```

#### Running LiDAR Node with Custom Parameters

```bash
ros2 run kitti_node lidar_node --ros-args \
    -p leaf_size_x:=0.1 \
    -p leaf_size_y:=0.1 \
    -p leaf_size_z:=0.1 \
    -p cluster_tolerance:=0.3 \
    -p min_cluster_size:=100 \
    -p max_cluster_size:=3000
```

## Running Both Nodes Together

To run both nodes simultaneously, you can use a launch file or run them in separate terminals:

### Terminal 1 (Camera Node):

```bash
ros2 run kitti_node camera_node
```

### Terminal 2 (LiDAR Node):

```bash
ros2 run kitti_node lidar_node
```

## Parameter Tuning

### Camera Node Parameters

- **canny_threshold1**: Lower threshold for edge detection. Decrease for more sensitive edge detection.
- **canny_threshold2**: Upper threshold for edge detection. Should typically be 2-3 times canny_threshold1.
- **canny_aperture_size**: Size of Sobel kernel. Must be odd (3, 5, 7). Larger values detect thicker edges.

### LiDAR Node Parameters

- **leaf_size_x/y/z**: Smaller values preserve more detail but increase computation time. Larger values reduce data size.
- **cluster_tolerance**: Distance threshold for clustering. Smaller values create more separate clusters.
- **min/max_cluster_size**: Filter out noise (small clusters) and overly large clusters.

## Visualization

To visualize the processed data, you can use RViz2:

```bash
rviz2
```

Add the following topics for visualization:

- Camera: `/kitti/image/canny/left` (Image display)
- LiDAR: `kitti/voxel_cloud` and `kitti/clustered_cloud` (PointCloud2 displays)

## Dynamic Reconfiguration

Parameters can be changed at runtime using the ROS2 parameter service:

```bash
# Change camera parameters
ros2 param set /camera_node canny_threshold1 75.0
ros2 param set /camera_node canny_threshold2 175.0

# Change LiDAR parameters
ros2 param set /lidar_node leaf_size_x 0.15
ros2 param set /lidar_node cluster_tolerance 0.4
```

## Troubleshooting

### Common Issues

1. **No input data**: Ensure KITTI data is being published to the expected topics
1. **Build errors**: Check that all dependencies are installed
1. **Poor edge detection**: Adjust Canny thresholds based on image characteristics
1. **Clustering issues**: Adjust clustering parameters based on point cloud density
