#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 检测图片中指定颜色的实体圆

class CircleDetector : public rclcpp::Node {
public:
    CircleDetector() : Node("circle_detector") {
        // 加载图片
        cv::Mat image = cv::imread("/home/lm/Desktop/catkin_ws/keymap_ws/src/ram_detect/src/cciimage_with_circle.png", cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image");
            return;
        }

        // 将图片从BGR转换为HSV色彩空间
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义蓝色的HSV范围
        cv::Scalar lower_blue(100, 150, 0);  // 蓝色的下限
        cv::Scalar upper_blue(140, 255, 255);  // 蓝色的上限

        // 根据定义的范围提取蓝色区域
        cv::Mat mask;
        cv::inRange(hsv_image, lower_blue, upper_blue, mask);

        // 对提取的蓝色区域进行形态学操作以去除噪声
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // 使用霍夫圆变换检测圆形
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, mask.rows / 8, 200, 20, 0, 0);

        if (circles.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No circles detected");
        } else {
            // 只取第一个圆
            cv::Vec3f c = circles[0];
            cv::Point center = cv::Point(c[0], c[1]);
            int radius = c[2];
            RCLCPP_INFO(this->get_logger(), "Circle detected: center (%d, %d), radius %d", center.x, center.y, radius);

            // 显示并保存检测结果
            cv::circle(image, center, radius, cv::Scalar(0, 255, 0), 3);  // 绿色圆边框
            cv::circle(image, center, 3, cv::Scalar(0, 0, 255), -1);  // 红色中心点
            cv::imwrite("detected_circle.png", image);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleDetector>());
    rclcpp::shutdown();
    return 0;
}
