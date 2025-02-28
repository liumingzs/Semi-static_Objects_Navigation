#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

class StereoSync : public rclcpp::Node
{
public:
    StereoSync()
        : Node("stereo_sync")
    {
        loadCalibrationData("/home/lm/Desktop/catkin_ws/RAM_ws/calibrationdata/left.yaml", cameraMatrixLeft_, distCoeffsLeft_, rectificationMatrixLeft_, projectionMatrixLeft_);
        loadCalibrationData("/home/lm/Desktop/catkin_ws/RAM_ws/calibrationdata/right.yaml", cameraMatrixRight_, distCoeffsRight_, rectificationMatrixRight_, projectionMatrixRight_);

        sub_left_.subscribe(this, "/camera/infra1/image_rect_raw");
        sub_right_.subscribe(this, "/camera/infra2/image_rect_raw");
        sync_.reset(new message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(
            sub_left_, sub_right_, 10));
        sync_->registerCallback(std::bind(&StereoSync::callback, this, std::placeholders::_1, std::placeholders::_2));

        // 初始化 ORB 检测器
        orb_ = cv::ORB::create();
    }

private:
    void loadCalibrationData(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rectificationMatrix, cv::Mat& projectionMatrix)
    {
        YAML::Node config = YAML::LoadFile(filename);

        cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(config["camera_matrix"]["data"].as<std::vector<double>>().data())).clone();
        distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(config["distortion_coefficients"]["data"].as<std::vector<double>>().data())).clone();
        rectificationMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(config["rectification_matrix"]["data"].as<std::vector<double>>().data())).clone();
        projectionMatrix = cv::Mat(3, 4, CV_64F, const_cast<double*>(config["projection_matrix"]["data"].as<std::vector<double>>().data())).clone();
    }

    std::vector<std::tuple<cv::Point2f, cv::Point2f, float>> loadMatchesFromFile(const std::string& filename)
    {
        std::vector<std::tuple<cv::Point2f, cv::Point2f, float>> file_matches;
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return file_matches;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            float x1, y1, x2, y2, distance;
            char dummy;
            ss >> dummy >> x1 >> dummy >> y1 >> dummy >> dummy >> x2 >> dummy >> y2 >> dummy >> distance;
            file_matches.emplace_back(cv::Point2f(x1, y1), cv::Point2f(x2, y2), distance);
        }

        file.close();
        return file_matches;
    }

    void compareMatches(const std::vector<cv::DMatch>& good_matches, const std::vector<cv::KeyPoint>& keypoints_left, const std::vector<cv::KeyPoint>& keypoints_right)
    {
        auto file_matches = loadMatchesFromFile("good_matches.txt");

        int match_count = 0;
        float total_distance = 0.0;
        float radius = 10.0;

        for (const auto& match : good_matches) {
            const cv::KeyPoint& kp_left = keypoints_left[match.queryIdx];
            const cv::KeyPoint& kp_right = keypoints_right[match.trainIdx];
            
            for (const auto& file_match : file_matches) {
                float distance_left = std::sqrt(std::pow(kp_left.pt.x - std::get<0>(file_match).x, 2) +
                                                std::pow(kp_left.pt.y - std::get<0>(file_match).y, 2));
                float distance_right = std::sqrt(std::pow(kp_right.pt.x - std::get<1>(file_match).x, 2) +
                                                std::pow(kp_right.pt.y - std::get<1>(file_match).y, 2));
                                                
                if (distance_left <= radius && distance_right <= radius) {
                    match_count++;
                    total_distance += match.distance;
                    break;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Number of matching points with file: %d", match_count);

        // 如果匹配点少于10，写入文件
        if (match_count < 10 && good_matches.size()> 10) {
            saveMatchesToFile(good_matches, keypoints_left, keypoints_right, "good_matches.txt");
        }
    }

    void saveMatchesToFile(const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints_left, const std::vector<cv::KeyPoint>& keypoints_right, const std::string& filename)
    {
        std::ofstream file(filename, std::ios_base::app); // 追加模式
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }

        for (const auto& match : matches) {
            const cv::KeyPoint& kp_left = keypoints_left[match.queryIdx];
            const cv::KeyPoint& kp_right = keypoints_right[match.trainIdx];
            file << "(" << kp_left.pt.x << ", " << kp_left.pt.y << ") "
                 << "(" << kp_right.pt.x << ", " << kp_right.pt.y << ") "
                 << match.distance << std::endl;
        }

        file << "----" << std::endl; // 分割符号
        file.close();

        RCLCPP_INFO(this->get_logger(), "Matches saved to file: %s", filename.c_str());
    }

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        // 转换为OpenCV图像
        cv::Mat left_image = cv_bridge::toCvCopy(left_msg, "mono8")->image;
        cv::Mat right_image = cv_bridge::toCvCopy(right_msg, "mono8")->image;

        // 校正图像
        cv::Mat undistorted_left, undistorted_right;
        cv::undistort(left_image, undistorted_left, cameraMatrixLeft_, distCoeffsLeft_);
        cv::undistort(right_image, undistorted_right, cameraMatrixRight_, distCoeffsRight_);

        // 检测 Oriented FAST 角点位置
        std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
        orb_->detect(undistorted_left, keypoints_left);
        orb_->detect(undistorted_right, keypoints_right);

        // 计算 BRIEF 描述子
        cv::Mat descriptors_left, descriptors_right;
        orb_->compute(undistorted_left, keypoints_left, descriptors_left);
        orb_->compute(undistorted_right, keypoints_right, descriptors_right);

        // 使用 FLANN 进行匹配
        cv::Ptr<cv::flann::IndexParams> index_params = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
        cv::Ptr<cv::flann::SearchParams> search_params = cv::makePtr<cv::flann::SearchParams>(50);
        cv::FlannBasedMatcher matcher(index_params, search_params);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors_left, descriptors_right, matches);

        // 筛选匹配点对
        double max_dist = 0;
        double min_dist = 100;
        for (int i = 0; i < descriptors_left.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }

        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < descriptors_left.rows; i++) {
            if (matches[i].distance <= std::max(1.2 * min_dist, 10.0)) {
                good_matches.push_back(matches[i]);
            }
        }

        

        std::sort(good_matches.begin(), good_matches.end(), [&keypoints_left](const cv::DMatch &a, const cv::DMatch &b) {
            const cv::Point2f &pt_a = keypoints_left[a.queryIdx].pt;
            const cv::Point2f &pt_b = keypoints_left[b.queryIdx].pt;
            if (pt_a.x != pt_b.x) return pt_a.x < pt_b.x;
            return pt_a.y < pt_b.y;
        });

        // 输出 good_matches 数组内的个数
        RCLCPP_INFO(this->get_logger(), "Number of good matches: %zu", good_matches.size());

        RCLCPP_INFO(this->get_logger(), "--------------------");

        // 对比匹配点对
        compareMatches(good_matches, keypoints_left, keypoints_right);

        // 在图像上绘制匹配结果
        cv::Mat img_matches;
        cv::drawMatches(undistorted_left, keypoints_left, undistorted_right, keypoints_right, good_matches, img_matches);

        // 显示校正后的图像和匹配结果
        // cv::imshow("Undistorted Left Image", undistorted_left);
        // cv::imshow("Undistorted Right Image", undistorted_right);
        cv::imshow("Matches", img_matches);
        cv::waitKey(1);
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_left_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_right_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> sync_;

    cv::Mat cameraMatrixLeft_, distCoeffsLeft_, rectificationMatrixLeft_, projectionMatrixLeft_;
    cv::Mat cameraMatrixRight_, distCoeffsRight_, rectificationMatrixRight_, projectionMatrixRight_;
    cv::Ptr<cv::ORB> orb_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoSync>());
    rclcpp::shutdown();
    return 0;
}
