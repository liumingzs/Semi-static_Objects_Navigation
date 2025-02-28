#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include <tuple>
#include <limits>
#include <iostream>
#include <array>
#include <std_msgs/msg/empty.hpp>  // 引入std_msgs::msg::Empty
#include <std_msgs/msg/string.hpp>  // 用于发布图像路径消息
#include <iomanip>  // 用于设置文件名格式
#include <sstream>  // 用于生成文件名
#include <string>
#include <thread>
#include <chrono>
#include <geometry_msgs/msg/pose.hpp> // 确保包含这个头文件

/*
读取掩码文件，计算掩码区域的三维坐标点并绘制3D检测框,获取长方体的长宽高,并且将物体作为一个节点添加入拓扑图当中
*/

// 全局变量，用于保存最小的x, y, z坐标值
cv::Point3f min_coordinates(std::numeric_limits<float>::max(), 
                            std::numeric_limits<float>::max(), 
                            std::numeric_limits<float>::max());

// 用于存储图像路径
std::string color_image_path_;
std::string depth_image_path_;

// 保存上一次的图像路径
std::string last_color_image_path_; 
cv::Point3f center;
// 保存当前的Pose
geometry_msgs::msg::Pose current_pose_; 

// 定义初始文件编号
int file_counter = 1;

class PixelTo3DNode : public rclcpp::Node
{
public:
    PixelTo3DNode()
        : Node("pixel_to_3d_node")
    {

        // 创建订阅器，接收图像路径消息
        path_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "image_paths", 10, std::bind(&PixelTo3DNode::on_image_paths_received, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/result_out_xyz", 10);

        // 创建发布器，发布物体位置 给graph订阅，添加物体节点
        object_publisher_ = this->create_publisher<std_msgs::msg::String>("/object_node", 10);

        // 创建订阅器，接收Pose数据
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose_data", 10, std::bind(&PixelTo3DNode::on_pose_data_received, this, std::placeholders::_1));


        // 定时器每 0.3 秒检查一次 flag 文件
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300),  // 设置为 300 毫秒
            std::bind(&PixelTo3DNode::checkFlagFile, this)
        );

        // Camera intrinsics
        camera_factor_ = 1000.0;
        camera_cx_ = 328.778382;
        camera_cy_ = 183.017877;
        camera_fx_ = 346.805803;
        camera_fy_ = 347.611893;

        RCLCPP_INFO(this->get_logger(), "Node initialized.");


    }
    

private:

    // 新增处理Pose数据的回调函数
    void on_pose_data_received(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 仅在 color_image_path_ 改变时更新 current_pose_
        if (color_image_path_ != last_color_image_path_) {
            current_pose_ = *msg; // 保存当前的Pose
            last_color_image_path_ = color_image_path_; // 更新上一次的图像路径
            RCLCPP_INFO(this->get_logger(), "Updated current pose: [%f, %f, %f]",
                        current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
        }
    }

    void checkFlagFile()
    {
        std::ifstream flag_file("/home/lm/Desktop/catkin_ws/MobileSAM/flag.txt");
        if (flag_file.is_open())
        {
            std::string flag;
            std::getline(flag_file, flag);

            if (flag == "1")
            {
                RCLCPP_INFO(this->get_logger(), "Python script has completed successfully. Proceeding with next task...");

                color_image_ = cv::imread(color_image_path_, cv::IMREAD_UNCHANGED);
                depth_image_ = cv::imread(depth_image_path_, cv::IMREAD_UNCHANGED);

                if (color_image_.empty() || depth_image_.empty()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load images.");
                    return;
                }

                // 检查图像类型并进行转换
                if (depth_image_.type() != CV_16UC1)
                {
                    RCLCPP_WARN(this->get_logger(), "Depth image is not of type CV_16UC1, attempting to convert...");
                    if (depth_image_.type() == CV_8UC1)
                    {
                        depth_image_.convertTo(depth_image_, CV_16UC1, 65535.0 / 255.0);
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Unsupported depth image type: %d", depth_image_.type());
                        return;
                    }
                }
                // 从mask.txt文件读取掩码
                Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask = load_txt_mask("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/mask.txt");
                // 读取完 mask.txt 后清空文件
                std::ofstream ofs("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/mask.txt", std::ofstream::out | std::ofstream::trunc);
                ofs.close();

                if (mask.size() == 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load mask from txt file.");
                    return;
                }
                
                // 计算掩码区域的三维坐标点
                std::vector<cv::Point3f> points_3d;
                for (int y = 0; y < mask.rows(); ++y) {
                    for (int x = 0; x < mask.cols(); ++x) {
                        if (mask(y, x)) {
                            cv::Point pt(x, y);
                            cv::Point3f point_3d = calculate3DPoint(pt);
                            if (point_3d != cv::Point3f(0, 0, 0)) {
                                points_3d.push_back(point_3d);
                            }
                        }
                    }
                }

                // 计算和绘制去除异常值后的3D长方体
                auto [length, width, height] = calculateBoundingBoxWithOutliersRemoved(points_3d);

                draw3DBoundingBox(length, width, height);

                // 示例数据，用来表示图片当前的位置和姿态
                std::array<double, 3> camera_position = {current_pose_.position.x, current_pose_.position.y, current_pose_.position.z};
                std::array<double, 4> camera_orientation = {current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z, current_pose_.orientation.w};
                std::array<double, 3> object_position_in_camera = {center.x, -center.y, center.z};
                
                // 给定的旋转矩阵
                Eigen::Matrix3d rotation_matrix;
                rotation_matrix << 
                    1.66533454e-16, -9.99955738e-01, 9.40865563e-03,
                    -2.83554630e-02, -9.40487244e-03, -9.99553658e-01,
                    9.99597903e-01, -2.66786786e-04, -2.83542079e-02;

                RCLCPP_INFO(this->get_logger(), "current pose: [%f, %f, %f]",
                            current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);


                // 转换
                auto object_position_in_world = transform_to_world(camera_position, camera_orientation, object_position_in_camera,rotation_matrix);

                // 输出世界坐标系下的三维点
                RCLCPP_INFO(this->get_logger(), "Object position in world: [%f, %f, %f]",
                            object_position_in_world[0], object_position_in_world[1], object_position_in_world[2]);

                // 发布信息到 ROS 2 话题
                auto message = std_msgs::msg::String();
                message.data = color_image_path_ + "," +
                               std::to_string(object_position_in_world[0]) + "," +
                               std::to_string(object_position_in_world[1]) + "," +
                               std::to_string(object_position_in_world[2]);
                object_publisher_->publish(message);

                std::string save_directory = "/home/lm/Desktop/catkin_ws/keymap_ws/src/ram_detect/src/3d_bounding_color/";  // 替换为实际保存路径
                save_image_with_incremented_filename(color_image_,save_directory);

                // 显示绘制了点的图像
                // cv::imshow("Color Image with 3D Points", color_image_);
                // cv::waitKey(0);

                // 重置 flag 文件，以防重复触发
                std::ofstream reset_file("/home/lm/Desktop/catkin_ws/MobileSAM/flag.txt");
                reset_file << "0";
                reset_file.close();
            }
            else if (flag == "0")
            {
                // RCLCPP_INFO(this->get_logger(), "Python script did not find valid data, waiting...");
            }

            flag_file.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open flag file.");
        }
    }

    void on_image_paths_received(const std_msgs::msg::String::SharedPtr msg)
    {
        
        // 解析收到的图像路径信息
        std::string received_paths = msg->data;
        size_t delimiter_pos = received_paths.find(",");
        if (delimiter_pos != std::string::npos) {
            color_image_path_ = received_paths.substr(0, delimiter_pos);
            depth_image_path_ = received_paths.substr(delimiter_pos + 1);

        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid image paths format.");
        }
    }


    void save_image_with_incremented_filename(const cv::Mat& image, const std::string& save_directory) {
        // 确保保存目录存在
        std::string full_save_directory = save_directory;
        
        // 确保目录以'/'结尾
        if (full_save_directory.back() != '/') {
            full_save_directory += '/';
        }

        // 创建文件名，自动从1开始并递增
        std::stringstream ss;
        ss << "image_" << std::setw(6) << std::setfill('0') << file_counter << ".png";
        std::string file_name = ss.str();
        
        // 完整保存路径
        std::string full_save_path = full_save_directory + file_name;
        
        // 保存图像到指定路径
        cv::imwrite(full_save_path, image);
        
        // 打印保存路径到终端以确认
        std::cout << "Image saved to: " << full_save_path << std::endl;
        
        // 文件计数器递增
        file_counter++;
    }

    std::array<double, 3> extract_position_from_data(const std::string& data_str) {
        std::array<double, 3> position;
        size_t start = data_str.find('(') + 1;
        size_t end = data_str.find(')');
        std::string coords = data_str.substr(start, end - start);
        std::stringstream ss(coords);
        char comma;
        ss >> position[0] >> comma >> position[1] >> comma >> position[2];
        return position;
    }

    std::array<double, 3> transform_to_world(const std::array<double, 3>& camera_position,
                                                const std::array<double, 4>& camera_orientation,
                                                const std::array<double, 3>& object_position,
                                                const Eigen::Matrix3d& rotation_matrix)
        {
            // 相机的位置
            Eigen::Vector3d cam_pos(camera_position[0], camera_position[1], camera_position[2]);

            // 相机的旋转（四元数）
            Eigen::Quaterniond cam_ori(camera_orientation[3], camera_orientation[0], camera_orientation[1], camera_orientation[2]);

            // 物体在相机坐标系中的位置，设置垂直方向上的点为零，也就是只获取垂直偏移和水平偏移
            Eigen::Vector3d obj_pos(object_position[0], object_position[1], 0.0);

            // 将四元数转换为旋转矩阵
            Eigen::Matrix3d cam_rot_matrix = cam_ori.toRotationMatrix();

            // 计算最终的旋转矩阵
            Eigen::Matrix3d final_rot_matrix = rotation_matrix * cam_rot_matrix; // 先乘以给定的旋转矩阵


            // 转换到世界坐标系
            Eigen::Vector3d obj_pos_world = final_rot_matrix * obj_pos + cam_pos;

            return {obj_pos_world.x(), obj_pos_world.y(), obj_pos_world.z()};
        }

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> load_txt_mask(const std::string &filename)
        {
            std::ifstream file(filename);
            if (!file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", filename.c_str());
                return Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>();
            }

            int rows = 360;
            int cols = 640;
            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask(rows, cols);

            std::string line;
            int row = 0;

            while (std::getline(file, line) && row < rows)
            {
                std::stringstream ss(line);
                int value;
                int col = 0;

                while (ss >> value && col < cols)
                {
                    mask(row, col) = (value == 1);
                    ++col;
                }

                ++row;
            }

            file.close();
            return mask;
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
                else
                {
                    return cv::Point3f(0, 0, 0);
                }
            }
            else
            {
                return cv::Point3f(0, 0, 0);
            }
        }

    std::tuple<float, float, float> calculateBoundingBoxWithOutliersRemoved(std::vector<cv::Point3f>& points_3d) 
    {
        if (points_3d.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No 3D points available to calculate bounding box.");
            return std::make_tuple(0.0f, 0.0f, 0.0f);
        }

        auto removeOutliers = [](std::vector<float>& data) {
            std::sort(data.begin(), data.end());
            size_t q1_index = data.size() / 4;
            size_t q3_index = 3 * data.size() / 4;
            float q1 = data[q1_index];
            float q3 = data[q3_index];
            float iqr = q3 - q1;
            float lower_bound = q1 - 1.5 * iqr;
            float upper_bound = q3 + 1.5 * iqr;
            data.erase(std::remove_if(data.begin(), data.end(), [lower_bound, upper_bound](float x) {
                return x < lower_bound || x > upper_bound;
            }), data.end());
        };

        std::vector<float> x_coords, y_coords, z_coords;

        for (const auto& pt : points_3d) {
            x_coords.push_back(pt.x);
            y_coords.push_back(pt.y);
            z_coords.push_back(pt.z);
        }

        removeOutliers(x_coords);
        removeOutliers(y_coords);
        removeOutliers(z_coords);

        float min_x = *std::min_element(x_coords.begin(), x_coords.end());
        float min_y = *std::min_element(y_coords.begin(), y_coords.end());
        float min_z = *std::min_element(z_coords.begin(), z_coords.end());

        float max_x = *std::max_element(x_coords.begin(), x_coords.end());
        float max_y = *std::max_element(y_coords.begin(), y_coords.end());
        float max_z = *std::max_element(z_coords.begin(), z_coords.end());

        float length = max_x - min_x;
        float width = max_y - min_y;
        float height = max_z - min_z;


        center.x = (min_x + max_x) / 2;
        center.y = (min_y + max_y) / 2;
        center.z = (min_z + max_z) / 2; 
        
        // 打印3D边界框的长宽高，边界框中心
        RCLCPP_INFO(this->get_logger(), "Bounding Box Dimensions: Length = %f, Width = %f, Height = %f", length, width, height);
        // RCLCPP_INFO(this->get_logger(), "Bounding Box Min: (%f, %f, %f), Max: (%f, %f, %f)", min_x, min_y, min_z, max_x, max_y, max_z);
        RCLCPP_INFO(this->get_logger(), "Bounding Box Center: (%f, %f, %f)", center.x, center.y, center.z);     // 向前 向右 向上

        min_coordinates.x = min_x;
        min_coordinates.y = min_y;
        min_coordinates.z = min_z;

        return std::make_tuple(length, width, height);
    }
    
    void draw3DBoundingBox(float length, float width, float height)
    {
        float min_depth = min_coordinates.x;  
        float min_y = min_coordinates.y;  
        float min_z = min_coordinates.z;  

        // 计算长方体的8个顶点坐标
        std::vector<cv::Point3f> bbox_corners = {
            {min_depth, min_y, min_z},
            {min_depth + length, min_y, min_z},
            {min_depth + length, min_y + width, min_z},
            {min_depth, min_y + width, min_z},
            {min_depth, min_y, min_z + height},
            {min_depth + length, min_y, min_z + height},
            {min_depth + length, min_y + width, min_z + height},
            {min_depth, min_y + width, min_z + height}
        };

        // 投影到2D图像平面
        std::vector<cv::Point2f> bbox_2d;
        for (const auto& pt : bbox_corners)
        {
            // 根据相机模型，将3D点投影到2D图像平面
            float u = (pt.y * camera_fx_ / pt.x) + camera_cx_;  // y是左右方向，x是深度
            float v = (pt.z * camera_fy_ / pt.x) + camera_cy_;  // z是上下方向，x是深度
            bbox_2d.push_back(cv::Point2f(u, v));
        }

        // 绘制长方体的12条边
        cv::line(color_image_, bbox_2d[0], bbox_2d[1], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[1], bbox_2d[2], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[2], bbox_2d[3], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[3], bbox_2d[0], cv::Scalar(0, 255, 0), 2);

        cv::line(color_image_, bbox_2d[4], bbox_2d[5], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[5], bbox_2d[6], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[6], bbox_2d[7], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[7], bbox_2d[4], cv::Scalar(0, 255, 0), 2);

        cv::line(color_image_, bbox_2d[0], bbox_2d[4], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[1], bbox_2d[5], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[2], bbox_2d[6], cv::Scalar(0, 255, 0), 2);
        cv::line(color_image_, bbox_2d[3], bbox_2d[7], cv::Scalar(0, 255, 0), 2);

        // 绘制最小点和最大点
        // 最小点
        float min_u = (min_y * camera_fx_ / min_depth) + camera_cx_;
        float min_v = (min_z * camera_fy_ / min_depth) + camera_cy_;
        cv::circle(color_image_, cv::Point2f(min_u, min_v), 5, cv::Scalar(0, 0, 255), -1);  // 红色圆圈表示最小点


        // 最大点
        float max_depth = min_depth + length;
        float max_y = min_y + width;
        float max_z = min_z + height;
        float max_u = (max_y * camera_fx_ / max_depth) + camera_cx_;
        float max_v = (max_z * camera_fy_ / max_depth) + camera_cy_;
        cv::circle(color_image_, cv::Point2f(max_u, max_v), 5, cv::Scalar(255, 0, 0), -1);  // 蓝色圆圈表示最大点

        // 绘制中心点
        cv::Point3f center;
        center.x = (min_depth + max_depth) / 2;
        center.y = (min_y + max_y) / 2;
        center.z = (min_z + max_z) / 2;

        // 将中心点投影到2D图像平面
        float center_u = (center.y * camera_fx_ / center.x) + camera_cx_;
        float center_v = (center.z * camera_fy_ / center.x) + camera_cy_;

        // 用绿色圆圈绘制中心点
        cv::circle(color_image_, cv::Point2f(center_u, center_v), 5, cv::Scalar(0, 255, 0), -1);  // 绿色圆圈表示中心点


    }



    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr task_complete_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_; // 用于订阅Pose消息

    cv::Mat color_image_;
    cv::Mat depth_image_;

    float camera_factor_, camera_cx_, camera_cy_, camera_fx_, camera_fy_;


    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixelTo3DNode>());
    rclcpp::shutdown();
    return 0;
}
