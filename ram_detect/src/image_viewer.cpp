#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

/*
同时订阅深度图像话题和彩色图像话题，并且保存彩色和深度图像 可以用于获取三维位置
*/
class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer()
  : Node("image_viewer")
  {
    color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10,
      std::bind(&ImageViewer::color_callback, this, std::placeholders::_1));

    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", 10,
      std::bind(&ImageViewer::depth_callback, this, std::placeholders::_1));
  }

  void check_and_save_images()
  {
    int key = cv::waitKey(1);
    if (key == 'c')
    {
      if (!latest_color_image_.empty() && !latest_depth_image_.empty())
      {
        std::string color_image_path = "color_image.png";
        std::string depth_image_path = "depth_image.png";

        // 保存彩色图像
        cv::imwrite(color_image_path, latest_color_image_);

        // 保存深度图像，保持16位格式
        cv::imwrite(depth_image_path, latest_depth_image_);
        
        RCLCPP_INFO(this->get_logger(), "Saved images to %s and %s", color_image_path.c_str(), depth_image_path.c_str());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "No images to save.");
      }
    }
  }

private:
  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    latest_color_image_ = cv_ptr->image.clone();
    cv::imshow("Color Image", latest_color_image_);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    latest_depth_image_ = cv_ptr->image.clone();

    // 归一化深度图像以适应显示
    cv::normalize(latest_depth_image_, depth_normalized_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("Depth Image", depth_normalized_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  cv::Mat latest_color_image_;
  cv::Mat latest_depth_image_;
  cv::Mat depth_normalized_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageViewer>();

  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    node->check_and_save_images();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
