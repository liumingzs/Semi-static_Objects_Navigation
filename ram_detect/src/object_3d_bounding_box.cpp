#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
/*
订阅彩色和深度图像话题，获取指定像素点的三维坐标
*/

class PixelTo3DNode : public rclcpp::Node
{
public:
    PixelTo3DNode()
        : Node("pixel_to_3d_node")
    {
        color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&PixelTo3DNode::colorImageCallback, this, std::placeholders::_1));

        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&PixelTo3DNode::depthImageCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/result_out_xyz", 10);

        // Camera intrinsics
        camera_factor_ = 1000.0;
        camera_cx_ = 328.778382;
        camera_cy_ = 183.017877;
        camera_fx_ = 346.805803;
        camera_fy_ = 347.611893;

        RCLCPP_INFO(this->get_logger(), "Node initialized.");
    }

private:
    void colorImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Color image received.");
        try
        {
            color_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (depth_image_)
        {
            RCLCPP_INFO(this->get_logger(), "Depth image available.");

            int n = 226;  // Fixed pixel coordinates
            int m = 179;

            RCLCPP_INFO(this->get_logger(), "Pixel coordinates: (%d, %d)", m, n);

            // Ensure pixel coordinates are within image bounds
            if (m >= 0 && m < depth_image_->image.rows && n >= 0 && n < depth_image_->image.cols)
            {
                float depth = depth_image_->image.at<float>(m, n);

                if (!std::isnan(depth))
                {
                    float x = (depth / camera_factor_) * 100.0;  // Forward/backward
                    float y = (n - camera_cx_) * x / camera_fx_;  // Left/right
                    float z = (m - camera_cy_) * x / camera_fy_;  // Up/down

                    RCLCPP_INFO(this->get_logger(), "3D Coordinates: (%f, %f, %f)", x , y , z );

                    // Draw the points and distance on the color image
                    cv::circle(color_image_->image, cv::Point(n, m), 5, cv::Scalar(0, 0, 255), -1);
                    cv::putText(color_image_->image, "3D: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")",
                                cv::Point(n, m - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

                    // Show the image with OpenCV
                    cv::imshow("Color Image with 3D Coordinates", color_image_->image);
                    cv::waitKey(1);

                    // Publish the 3D coordinates
                    std_msgs::msg::Float64MultiArray array;
                    array.data = {x, y, z};
                    pub_->publish(array);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Depth value is NaN at pixel: (%d, %d)", m, n);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Pixel coordinates out of bounds: (%d, %d)", m, n);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Depth image not available.");
        }
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Depth image received.");
        try
        {
            depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

    cv_bridge::CvImagePtr color_image_;
    cv_bridge::CvImagePtr depth_image_;

    float camera_factor_, camera_cx_, camera_cy_, camera_fx_, camera_fy_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixelTo3DNode>());
    rclcpp::shutdown();
    return 0;
}
