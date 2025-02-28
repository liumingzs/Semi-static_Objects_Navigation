#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>  // 用于发布图像路径消息
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cstdlib> // for system()
#include <opencv2/core.hpp>
#include <cstdio>
#include <string>
#include <array>
#include <vector>
#include <sstream>
#include <chrono>
#include <tuple>
#include <std_msgs/msg/empty.hpp>  // 用于发布任务完成的空消息   
#include <rclcpp/qos.hpp>
// 读取深度和彩色图片，根据自定义的两个像素点，获取两个像素点之间的三维数据，保存成数组points，然后调用python脚本classification.py进行分类，获取到物体的深度值

std::string color_image_path;
std::string depth_image_path;
std::vector<std::tuple<cv::Point3f, cv::Point>> points;
class MaskToDepth : public rclcpp::Node
{
public:
    MaskToDepth()
        : Node("mask_to_depth_node"),
        previous_color_path_(""), previous_depth_path_("")
    {

        path_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "image_paths",
            10,  
            std::bind(&MaskToDepth::imagepathCallback, this, std::placeholders::_1)
        );
    }

private:

    std::string previous_color_path_;  // 用于存储先前的color路径
    std::string previous_depth_path_;  // 用于存储先前的depth路径

    void imagepathCallback(const std_msgs::msg::String::SharedPtr msg){

        if (msg->data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Image paths are empty.");
            return;
        }
        // 从消息中提取路径
        std::string paths = msg->data;
        std::istringstream ss(paths);
        std::string color_path, depth_path;
        // 使用逗号分隔路径
        if (std::getline(ss, color_path, ',') && std::getline(ss, depth_path, ','))
        {
            // 检查color_path是否发生变化
            if (color_path != previous_color_path_ )
            {
                // 加载图像和相机内参
                if (!loadImagesAndParameters(color_path, depth_path)) {
                    return;
                }

                // 自定义两个像素点
                cv::Point pt1(120, 240);  // 第一个点
                cv::Point pt2(570, 240);  // 第二个点

                // 获取连线之间所有像素点的三维坐标
                points.clear();  // 清空上次的points，避免重复
                std::vector<cv::Point> line_points = getLinePoints(pt1, pt2);
                for (const auto& pt : line_points)
                {
                    cv::Point3f point_3d = calculate3DPoint(pt);
                    points.emplace_back(point_3d, pt);
                }

                // 路径变化时调用Python脚本
                runPythonScript(points, color_path);

                // 更新先前的路径记录
                previous_color_path_ = color_path;
                previous_depth_path_ = depth_path;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Paths have not changed, skipping Python script call.");
            }
        }

    }

    void runPythonScript(const std::vector<std::tuple<cv::Point3f, cv::Point>>& points, const std::string& color_image_path) {
        

        // 打开管道并传递图像路径
        std::string command = "python3 /home/lm/Desktop/catkin_ws/MobileSAM/sam_client.py " + color_image_path;
        FILE* pipe = popen(command.c_str(), "w");
        
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Failed to run Python script.");
            return;
        }

        // 输出每个点到管道
        for (const auto& [point3D, pixel] : points) {
            fprintf(pipe, "[%f, %f, %f, (%d, %d)]\n", point3D.x, point3D.y, point3D.z, pixel.x, pixel.y);
        }
        // 关闭管道
        pclose(pipe);
    }


    bool loadImagesAndParameters(const std::string& color_path, const std::string& depth_path)
    {
        // 指定彩色和深度图像的路径
        color_image_ = cv::imread(color_path, cv::IMREAD_COLOR);
        depth_image_ = cv::imread(depth_path, cv::IMREAD_UNCHANGED); // 深度图通常为单通道

        if (color_image_.empty() || depth_image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load images.");
            return false;
        }

        // 摄像头内参
        camera_factor_ = 1000.0;
        camera_cx_ = 328.778382;
        camera_cy_ = 183.017877;
        camera_fx_ = 346.805803;
        camera_fy_ = 347.611893;

        // 检查图像类型并进行转换
        if (depth_image_.type() != CV_16UC1)
        {
            RCLCPP_WARN(this->get_logger(), "Depth image is not of type CV_16UC1, attempting to convert...");
            if (depth_image_.type() == CV_8UC1)
            {
                depth_image_.convertTo(depth_image_, CV_16UC1, 65535.0 / 255.0);
            }
            else
            { RCLCPP_ERROR(this->get_logger(), "Unsupported depth image type: %d", depth_image_.type());
              return false; }
        }
        return true;
    }

    cv::Point3f calculate3DPoint(const cv::Point& pt)
    {
        if (pt.y >= 0 && pt.y < depth_image_.rows && pt.x >= 0 && pt.x < depth_image_.cols)
        {
            uint16_t depth = depth_image_.at<uint16_t>(pt.y, pt.x);

            if (depth > 0) // 确保深度值有效
            {
                // 表示坐标系
                float x = depth / camera_factor_;   //表示深度  指向前方
                float y = (pt.x - camera_cx_) * x / camera_fx_;    //表示横向距离  指向右
                float z = (pt.y - camera_cy_) * x / camera_fy_;    //表示上下距离  指向下

                // 添加负号 指向上
                // RCLCPP_INFO(this->get_logger(), "3D Coordinates: (%f, %f, %f)", x, y, z);

                return cv::Point3f(x, y, z);
            }
            else { return cv::Point3f(0, 0, 0); }
        }
        else { return cv::Point3f(0, 0, 0); }
    }

    std::vector<cv::Point> getLinePoints(const cv::Point& pt1, const cv::Point& pt2)
    {
        std::vector<cv::Point> points;
        cv::LineIterator it(color_image_, pt1, pt2, 8);
        for (int i = 0; i < cv::norm(pt2 - pt1); i++, ++it)
        {
            points.push_back(it.pos());
        }
        return points;
    }

    cv::Mat color_image_;
    cv::Mat depth_image_;

    float camera_factor_, camera_cx_, camera_cy_, camera_fx_, camera_fy_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_subscription_; // 声明图像路径订阅器
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MaskToDepth>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
