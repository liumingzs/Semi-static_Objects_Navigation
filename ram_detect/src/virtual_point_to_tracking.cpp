#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp> // 替换为正确的消息类型头文件
#include <geometry_msgs/msg/pose.hpp>

using namespace std;

vector<double> pos_x;
vector<double> pos_y;
vector<double> pos_yaw;

bool needs_turn = false; // 新增的全局变量，用于决定是否需要掉头

string file_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/route_trajectory.txt"; //需要巡迹的轨迹

int pointNum = 0;
int poinyYawnum = 0;
bool will_stop;
double vel_x = 0, vel_z = 0;
int aim_index = -1, tmp_index;
double k;    //比例系数

class NavigationController : public rclcpp::Node
{
public:
    NavigationController() : Node("cmd_vel_publisher")
    {
        load_aims();
        position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NavigationController::callbackPosition, this, std::placeholders::_1));
        // 创建发布者，发布到"/cmd_vel" 话题
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // 打开记录文件
        record_file_.open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/for_record_virtual_tracking.txt", std::ios::out | std::ios::app);
        if (!record_file_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open record file.");
        }

    }


private:

    double last_x, last_y;  // 上一次保存的位置
    ofstream record_file_;   // 用于保存数据的文件

    void callbackPosition(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if(needs_turn)  // 如果需要调头
        {
            auto msg_turn = geometry_msgs::msg::Twist();
            msg_turn.angular.z = 0.2;
            pub_vel_->publish(msg_turn);
            // 设置定时器，15.7秒后停止自转
            rclcpp::sleep_for(std::chrono::milliseconds(15600));
            std::cout<<"自转完成"<<endl;
            // 停止自转
            msg_turn.angular.z = 0.0; // 设置角速度为0，停止自转
            pub_vel_->publish(msg_turn); // 发布停止指令

            // 更新标志，确保掉头只执行一次
            needs_turn = false;
        }
        
        // X轴朝前，Y轴朝左
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double currentYaw = yaw;

        double currentPositionX = msg->pose.pose.position.x;
        double currentPositionY = msg->pose.pose.position.y;
        double currentPositionZ = msg->pose.pose.position.z;

        // 检查 end_flag.txt 文件的内容
        std::ifstream flag_file("/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt");
        if (flag_file.is_open()) {
            std::string flag_content;
            std::getline(flag_file, flag_content);
            flag_file.close();

            // 如果文件内容是 "1"，表示已经查找到了指定的物体，停止发布速度并返回
            if (flag_content == "1") {
                will_stop = true;
                // 将当前位置信息存入 end_car_position.txt
                std::ofstream position_file("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/end_car_position.txt", std::ios::out);
                if (position_file.is_open()) {
                    position_file <<currentPositionX << " "
                                <<currentPositionY << " "
                                <<currentPositionZ << " "
                                <<msg->pose.pose.orientation.x<<" "
                                <<msg->pose.pose.orientation.y<<" "
                                <<msg->pose.pose.orientation.z<<" "
                                <<msg->pose.pose.orientation.w<<std::endl;
                    position_file.close();
                    std::cout << "位置信息已写入 end_car_position.txt。" << std::endl;
                } else {
                    std::cout << "无法打开文件：/home/lm/Desktop/catkin_ws/keymap_ws/end_car_position.txt" << std::endl;
                }

                cout << "已经查找到了指定的物体，停止速度发布。" << endl;
            }
        } else {
            cout << "无法打开文件：/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt" << endl;
        }
        
        // 计算当前位置与上一次保存位置的欧氏距离
        double distance_record = sqrt(pow(currentPositionX - last_x, 2) + pow(currentPositionY - last_y, 2));

        // 如果欧式距离大于等于0.2m，则保存数据
        if (distance_record >= 0.2)
        {
            // 获取当前时间戳
            rclcpp::Time timestamp = this->get_clock()->now();

            // 保存数据到文件（时间戳 x y 四元数）
            record_file_ << timestamp.seconds()+timestamp.nanoseconds() << " " 
                         << currentPositionX << " " 
                         << currentPositionY << " " 
                         << currentPositionZ << " " 
                         << q.x() << " " 
                         << q.y() << " " 
                         << q.z() << " " 
                         << q.w() << "\n";
            record_file_.flush(); // 确保数据即时写入文件
            // 更新上一次的位置
            last_x = currentPositionX;
            last_y = currentPositionY;
        }

        vector<double> bestPoints_;
        if (aim_index == -1) {
            for (int i = 0; i < pointNum ; i++) {    // 不用除以3了，这样的话会出现无法找到中间位置的起点，如果我在pointnum的二分之一处启动，它除以3了，就找不到了
                double path_x = pos_x[i];
                double path_y = pos_y[i];
                double lad = sqrt(pow(path_x - currentPositionX, 2) + pow(path_y - currentPositionY, 2));
                bestPoints_.push_back(lad); //存入的是轨迹中的每个点与当前点的距离
            }
            auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
            aim_index = distance(bestPoints_.begin(), smallest);    // 寻找最小的点与序号
        } else {
            for (int i = (aim_index - 25 >= 0 ? aim_index - 25 : 0); i < (aim_index + 25 >= pointNum ? pointNum - 1 : aim_index + 25); i++) {
                double path_x = pos_x[i];
                double path_y = pos_y[i];
                double lad = sqrt(pow(path_x - currentPositionX, 2) + pow(path_y - currentPositionY, 2));
                bestPoints_.push_back(lad);
            }
            auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
            tmp_index = distance(bestPoints_.begin(), smallest);
            if (aim_index - 25 >= 0)
                aim_index = aim_index - 25 + tmp_index;
            else
                aim_index = tmp_index;
        }
        aim_index += 1;
        // cout << "找到轨迹上最近的点是： " << aim_index << " " << pos_x[aim_index] << " " << pos_y[aim_index] << " " << pos_yaw[aim_index] << endl;
        // cout << "当前节点是："<<aim_index<<"当前位置是：" << currentPositionX << " " << currentPositionY << endl;
        int pos_before, pos_next;
        pos_before = aim_index - 3 < 0 ? 0 : aim_index - 3;
        pos_next = aim_index + 2 > pointNum - 1 ? pointNum - 1 : aim_index + 2;

        // cout << "pos_next: " << pos_next << " " << pos_x[pos_next] << " " << pos_y[pos_next] << " " << endl;

        // 点数超过总数后停止运行
        if ( aim_index + 3 >= pointNum ){
            will_stop = true;
            flag_file.open("/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt");
            if(flag_file.is_open()){
                string line_flag;
                getline(flag_file, line_flag);  // 读取文件中的一行
                flag_file.close();
                if (line_flag != "1") {     // 如果第一次导航的时候没有找到物体
                    std::ofstream second_goal_file("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/second_goal.txt");
                    if (second_goal_file.is_open()) {
                        second_goal_file << "1" << std::endl;  // 向文件写入1
                        second_goal_file.close();
                    }
                }
                else{   // 如果第一次导航的时候找到了物体
                    std::ofstream second_goal_file("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/second_goal.txt");

                    if (second_goal_file.is_open()) {
                        second_goal_file << "4" << std::endl;  // 向文件写入4
                        second_goal_file.close();
                    }
                }
            }

        }


        //计算点到直线的距离
        double dis = signedDistanceToLine(currentPositionX, currentPositionY, pos_x[pos_before],
                                          pos_y[pos_before], pos_x[pos_next], pos_y[pos_next]);
        // cout << "dis" << dis<<endl;
        // 根据经验值修改k的值

        // 使用紫川的
        k = -0.012 * 180 / 3.1415926 ;

        double angular_z = dis * k;//线性旋转角度
        //double angular_z = dis*dis;//非线性旋转角度
        vel_x = 0.06;
        vel_z = -1.2 * angular_z;
        if(vel_z > 0.35){ vel_z = 0.35;}
        if(vel_z < -0.35) {vel_z = -0.35;}
        // cout << "vel_z:" << vel_z <<endl;
        
        publishVelocity();
    }

    void publishVelocity()
    {
        // 创建一个新的 Twist 消息对象
        auto msg = geometry_msgs::msg::Twist();

        if (will_stop) {
            vel_x = 0;
            vel_z = 0;
            msg.linear.x = vel_x;   
            msg.angular.z = vel_z;  // 例如：设定角速度为 0.0 rad/s
            pub_vel_->publish(msg);

            rclcpp::shutdown(); // 先关闭ROS 2节点
            std::exit(EXIT_SUCCESS); // 退出程序
        }

        // 修改线速度和角速度
        msg.linear.x = vel_x;  
        msg.angular.z = vel_z;  // 例如：设定角速度为 0.0 rad/s

        // 输出日志，查看发布的速度
        // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: Linear: %.2f, Angular: %.2f",
        //             msg.linear.x, msg.angular.z);

        // 发布消息到 /cmd_vel
        pub_vel_->publish(msg);
    }

    bool check_flag_reverse() {
    std::ifstream file("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/flag_reverse.txt");
    if (file.is_open()) {
        std::string content;
        file >> content;  // 读取文件内容
        file.close();
        
        // 如果文件内容不为 "1"，返回 true
        if (content != "1") {
            return true;
        }
    } else {
        std::cerr << "无法打开文件 flag_reverse.txt" << std::endl;
    }
    return false;
    }

    void load_aims() {
        will_stop = false;
        cout << "开始加载目标点" << endl;
        ifstream in(file_path);
        string line;
        while (getline(in, line)) {
            stringstream ss(line);
            double time,x, y, z, qx, qy, qz, qw;
            /*
            根据读取的文件修改格式
            */
            ss >> time;
            ss.ignore(1); // 跳过逗号
            ss >> x;
            ss.ignore(1); // 跳过逗号
            ss >> y;
            ss.ignore(1); // 跳过逗号
            ss >> z;
            ss.ignore(1); // 跳过逗号
            ss >> qx;
            ss.ignore(1); // 跳过逗号
            ss >> qy;
            ss.ignore(1); // 跳过逗号
            ss >> qz;
            ss.ignore(1); // 跳过逗号
            ss >> qw;

            pos_x.push_back(x);
            pos_y.push_back(y);
            tf2::Quaternion q(qx, qy, qz, qw);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            pos_yaw.push_back(yaw);
        }
        if(check_flag_reverse()){          // 原来是这样的 if(pos_x[0] > pos_x[1] and pos_x[0] > 0){ 
            needs_turn = true;
            cout<<"开始调头"<<endl;
        }
        pointNum = pos_x.size();
        poinyYawnum = pos_yaw.size();
        cout << "加载路标点完成，一共有" << pointNum << "个点，开始巡迹"<< endl;
    }

    //计算点到直线的距离
    double signedDistanceToLine(double x0, double y0, double x1, double y1, double x2, double y2) {
        double numerator = fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1));
        double denominator = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        double sign = ((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) >= 0 ? 1 : -1;
        return sign * (numerator / denominator);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
};

int main(int argc, char *argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavigationController>();

    // 启动节点
    rclcpp::spin(node);

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}
