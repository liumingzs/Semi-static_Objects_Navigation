#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//  这是订阅深度话题 然后保存图像的节点，好像不需要了
class DepthImageConverter : public rclcpp::Node {
public:
    DepthImageConverter() : Node("depth_image_converter") {
        // 订阅深度图像话题
        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&DepthImageConverter::depthImageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Depth Image Converter Node has been started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Depth image received.");

        // 将 ROS 图像消息转换为 OpenCV 格式
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 如果深度图像格式为 Y14H14，需要进行解码处理
        cv::Mat depth_image = decodeY14H14(cv_ptr->image);

        // 保存深度图像为 PNG 文件
        cv::imwrite("dddepth_image.png", depth_image);
        RCLCPP_INFO(this->get_logger(), "Depth image saved as depth_image.png");
    }

    cv::Mat decodeY14H14(const cv::Mat& input_image) {
        // 假设输入图像是 Y14H14 格式，需要解码为 16 位无符号单通道图像
        cv::Mat depth_image(input_image.rows, input_image.cols, CV_16UC1);
        for (int i = 0; i < input_image.rows; ++i) {
            for (int j = 0; j < input_image.cols; ++j) {
                // 提取 14 位的深度值并扩展到 16 位
                uint16_t value = input_image.at<uint16_t>(i, j) & 0x3FFF; // 14-bit value
                depth_image.at<uint16_t>(i, j) = value << 2; // 转换为 16-bit
            }
        }
        return depth_image;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImageConverter>());
    rclcpp::shutdown();
    return 0;
}
