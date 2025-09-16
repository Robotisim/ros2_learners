#include <algorithm>                      // <-- add this
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    this->declare_parameter("lower_threshold", 100.0);
    this->declare_parameter("upper_threshold", 200.0);
    this->declare_parameter("x_start", 0);
    this->declare_parameter("x_end", 0); // 0 => full width

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/kitti/image/color/left", 10,
      std::bind(&CameraNode::imageCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/kitti/image/canny/left", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      const cv::Mat& img = cv_ptr->image;

      const double lower = this->get_parameter("lower_threshold").as_double();
      const double upper = this->get_parameter("upper_threshold").as_double();

      // Cast int64_t -> int before using std::max/min
      int x0 = std::max(0, static_cast<int>(this->get_parameter("x_start").as_int()));
      int x1 = static_cast<int>(this->get_parameter("x_end").as_int());
      if (x1 <= 0) x1 = img.cols;               // 0 or negative => right edge
      x1 = std::min(x1, img.cols);

      if (x1 <= x0) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Invalid crop range [%d,%d); using full width.", x0, x1);
        x0 = 0; x1 = img.cols;
      }

      cv::Mat roi = img(cv::Rect(x0, 0, x1 - x0, img.rows));

      cv::Mat gray, edges;
      cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
      cv::Canny(gray, edges, lower, upper, 3);

      cv_bridge::CvImage out;
      out.header = msg->header;
      out.encoding = sensor_msgs::image_encodings::MONO8;
      out.image = edges;
      pub_->publish(*out.toImageMsg());
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
