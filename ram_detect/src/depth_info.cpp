#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
/*
  获取指定像素点位置的深度信息
*/
class DepthInfo : public rclcpp::Node
{
public:
  DepthInfo()
  : Node("depth_info")
  {
    read_depth_image("/home/lm/Desktop/catkin_ws/keymap_ws/keyframe/depth/frame_000006.png");
  }

private:
  void read_depth_image(const std::string& image_path)
  {
    // 使用 OpenCV 读取深度图像
    cv::Mat depth_image = cv::imread(image_path, cv::IMREAD_UNCHANGED);

    if (depth_image.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read depth image from %s", image_path.c_str());
      return;
    }

    // 检查图像类型并进行转换
    if (depth_image.type() != CV_16UC1)
    {
      RCLCPP_WARN(this->get_logger(), "Depth image is not of type CV_16UC1, attempting to convert...");
      // 如果图像是CV_8UC1（8位无符号单通道），则将其转换为CV_16UC1
      if (depth_image.type() == CV_8UC1)
      {
        
        depth_image.convertTo(depth_image, CV_16UC1, 65535.0 / 255.0);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unsupported depth image type: %d", depth_image.type());
        return;
      }
    }

    // 确保深度图像的单位是毫米
    // 如果深度图像的单位是米，请将转换比例调整为1.0 / 1000.0

    // 指定像素点坐标
    int x = 488; // 示例坐标
    int y = 242; // 示例坐标

    if (x >= 0 && x < depth_image.cols && y >= 0 && y < depth_image.rows)
    {
      uint16_t depth_value_mm = depth_image.at<uint16_t>(y, x);
      double depth_value_m = static_cast<double>(depth_value_mm) / 1000.0; // 将毫米转换为米
      RCLCPP_INFO(this->get_logger(), "Depth value at (%d, %d): %f meters", x, y, depth_value_m);

      // 绘制圆圈在指定的像素位置
      cv::circle(depth_image, cv::Point(x, y), 5, cv::Scalar(65535), 2);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Specified pixel is out of image bounds.");
    }

    // 显示图像
    cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
    // 为了可视化，需要将深度图像转换为8位图像
    cv::Mat depth_image_8u;
    depth_image.convertTo(depth_image_8u, CV_8UC1, 255.0 / 65535.0);
    cv::imshow("Depth Image", depth_image_8u);
    cv::waitKey(0); // 等待按键按下
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthInfo>());
  rclcpp::shutdown();
  return 0;
}
