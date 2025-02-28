#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc.hpp>
#include <string.h>  

// 先使用fastsam的文本输入检测物体在图像上的二维位置，然后使用这个代码，读取彩色和深度图的图像，获取三维位置点

class CircleDepthDetector : public rclcpp::Node {
public:
    CircleDepthDetector() : Node("circle_depth_detector") {
        // 读取RGB和深度图像路径
        rgb_image_path_ = "/home/lm/Desktop/catkin_ws/keymap_ws/src/ram_detect/src/cciimage_with_circle.png";
        depth_image_path_ = "/home/lm/Desktop/catkin_ws/keymap_ws/depth_image.png";

        // 初始化 center_ 为无效坐标
        center_ = cv::Point(-1, -1);

        // 加载并处理RGB图像
        detect_circle();

        // 加载深度图像并获取圆心的三维坐标信息
        if (center_.x != -1 && center_.y != -1) {
            get_3d_coordinates_at_circle_center();
        } else {
            RCLCPP_ERROR(this->get_logger(), "No circle detected. 3D information cannot be retrieved.");
        }
    }

private:
    std::string rgb_image_path_;
    std::string depth_image_path_;
    cv::Point center_;

    // 相机内参
    float camera_factor_ = 1000.0;
    float camera_cx_ = 328.778382;
    float camera_cy_ = 183.017877;
    float camera_fx_ = 346.805803;
    float camera_fy_ = 347.611893;

    void detect_circle() {
        // 加载RGB图像
        cv::Mat image = cv::imread(rgb_image_path_, cv::IMREAD_COLOR);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", rgb_image_path_.c_str());
            return;
        }

        // 将图像从BGR转换为HSV
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // 定义蓝色的HSV范围并提取相应的颜色区域
        cv::Scalar lower_blue(100, 150, 0);  // 蓝色的下限
        cv::Scalar upper_blue(140, 255, 255);  // 蓝色的上限
        cv::Mat mask;
        cv::inRange(hsv_image, lower_blue, upper_blue, mask);

        // 形态学操作去除噪声
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // 霍夫圆变换检测圆形
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, mask.rows / 8, 200, 20, 0, 0);

        if (circles.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No circles detected");
        } else {
            cv::Vec3f c = circles[0];
            center_ = cv::Point(c[0], c[1]);
            int radius = c[2];
            RCLCPP_INFO(this->get_logger(), "Circle detected: center (%d, %d), radius %d", center_.x, center_.y, radius);

            // 绘制检测到的圆
            cv::circle(image, center_, radius, cv::Scalar(0, 255, 0), 3);
            cv::circle(image, center_, 3, cv::Scalar(0, 0, 255), -1);
            cv::imwrite("detected_circle.png", image);
        }
    }

    void get_3d_coordinates_at_circle_center() {
        // 读取深度图像
        cv::Mat depth_image = cv::imread(depth_image_path_, cv::IMREAD_UNCHANGED);
        if (depth_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load depth image: %s", depth_image_path_.c_str());
            return;
        }

        

        // 检查图像类型并进行必要的转换
        if (depth_image.type() != CV_16UC1) {
            RCLCPP_WARN(this->get_logger(), "Depth image is not of type CV_16UC1, attempting to convert...");
            RCLCPP_WARN(this->get_logger(), "depth_image.type():%d",depth_image.type());

            if (depth_image.type() == CV_8UC1) {
                depth_image.convertTo(depth_image, CV_16UC1, 65535.0 / 255.0);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported depth image type: %d", depth_image.type());
                return;
            }
        }

        // 获取圆心的深度值
        int x = center_.x;
        int y = center_.y;

        if (x >= 0 && x < depth_image.cols && y >= 0 && y < depth_image.rows) {
            uint16_t depth_value_mm = depth_image.at<uint16_t>(y, x);
            double depth_value_m = static_cast<double>(depth_value_mm) / 1000.0;  // 将毫米转换为米

            // 计算三维坐标
            double z = depth_value_m;
            double x_3d = (x - camera_cx_) * z / camera_fx_;
            double y_3d = (y - camera_cy_) * z / camera_fy_;

            //  x 轴上的位置（通常是左右方向）;y 轴上的位置（通常是上下方向）; z 轴上的位置（通常是远近方向，深度）
            RCLCPP_INFO(this->get_logger(), "3D Coordinates at circle center: (%f, %f, %f) meters", x_3d, y_3d, z);

            // 绘制圆圈在指定的像素位置
            cv::circle(depth_image, cv::Point(x, y), 5, cv::Scalar(65535), 2);

            // 显示图像
            cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
            cv::Mat depth_image_8u;
            depth_image.convertTo(depth_image_8u, CV_8UC1, 255.0 / 65535.0);
            cv::imshow("Depth Image", depth_image_8u);
            cv::waitKey(0);
        } else {
            RCLCPP_WARN(this->get_logger(), "Specified pixel is out of image bounds.");
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleDepthDetector>());
    rclcpp::shutdown();
    return 0;
}
