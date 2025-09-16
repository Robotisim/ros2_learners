#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarNode : public rclcpp::Node {
public:
  LidarNode() : Node("lidar_node") {
    // Voxel size (meters)
    this->declare_parameter("leaf_size", 0.2);

    // ROI bounds in meters (x forward, y left, z up)
    this->declare_parameter("x_min", -1000.0);
    this->declare_parameter("x_max",  1000.0);
    this->declare_parameter("y_min", -1000.0);
    this->declare_parameter("y_max",  1000.0);
    this->declare_parameter("z_min", -1000.0);
    this->declare_parameter("z_max",  1000.0);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "kitti/point_cloud", 10,
      std::bind(&LidarNode::cb, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("kitti/voxel_cloud", 10);
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Params
    const double leaf = this->get_parameter("leaf_size").as_double();
    const double x_min = this->get_parameter("x_min").as_double();
    const double x_max = this->get_parameter("x_max").as_double();
    const double y_min = this->get_parameter("y_min").as_double();
    const double y_max = this->get_parameter("y_max").as_double();
    const double z_min = this->get_parameter("z_min").as_double();
    const double z_max = this->get_parameter("z_max").as_double();

    // In cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *in);

    // ROI: pass-through on x, then y, then z
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_x(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(static_cast<float>(x_min), static_cast<float>(x_max));
    pass.filter(*roi_x);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_xy(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(roi_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(static_cast<float>(y_min), static_cast<float>(y_max));
    pass.filter(*roi_xy);

    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(roi_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(z_min), static_cast<float>(z_max));
    pass.filter(*roi_xyz);

    // Voxel downsample
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(roi_xyz);
    const float ls = static_cast<float>(leaf);
    vg.setLeafSize(ls, ls, ls);

    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*out_cloud);

    // Publish
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*out_cloud, out_msg);
    out_msg.header = msg->header;
    pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
