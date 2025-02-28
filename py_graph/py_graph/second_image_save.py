# 该节点的主要功能是基于机器人里程计的位置信息，以一定的距离阈值为条件定期保存相机的彩色图像和深度图像，并将这些图像的文件路径发布到一个话题上
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import math
import os
import cv2
import numpy as np

class ImagePathPublisherNode(Node):
    def __init__(self):
        super().__init__('image_path_publisher_node')

        # 设定距离阈值
        self.distance_threshold = 0.2

        # 创建发布 image_paths 话题
        self.image_path_publisher = self.create_publisher(String, 'image_paths', 10)

        # 订阅 /odom 话题
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )

        # 订阅相机彩色图像和深度图像
        self.color_image_subscription = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/image_raw',
            self.color_image_callback,
            10
        )
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/depth/image_raw',
            self.depth_image_callback,
            10
        )

        # 创建图像保存路径
        self.color_image_dir = '/home/lm/Desktop/catkin_ws/keymap_ws/keyframe_second/color/'
        self.depth_image_dir = '/home/lm/Desktop/catkin_ws/keymap_ws/keyframe_second/depth/'
        os.makedirs(self.color_image_dir, exist_ok=True)
        os.makedirs(self.depth_image_dir, exist_ok=True)

        # 用于存储上一次的位置信息和当前帧图像
        self.last_position = None
        self.current_color_image = None
        self.current_depth_image = None
        self.bridge = CvBridge()
        self.image_index = 0

    def color_image_callback(self, msg):
        """处理彩色图像并存储为 OpenCV 格式"""
        self.current_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_image_callback(self, msg):
        """处理深度图像并存储为 OpenCV 格式"""
        self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def odometry_callback(self, msg):
        # 获取当前位置
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # 如果还没有上次位置，则保存当前为初始位置
        if self.last_position is None:
            self.last_position = current_position
            return

        # 计算与上一次位置的距离
        distance = math.sqrt(
            (current_position[0] - self.last_position[0]) ** 2 +
            (current_position[1] - self.last_position[1]) ** 2 +
            (current_position[2] - self.last_position[2]) ** 2
        )

        # 如果距离超过阈值且图像数据可用，保存并发布路径
        if distance >= self.distance_threshold and self.current_color_image is not None and self.current_depth_image is not None:
            # 更新上一次的位置
            self.last_position = current_position

            # 生成图像路径
            color_image_path = os.path.join(self.color_image_dir, f"frame_{self.image_index:06d}.png")
            depth_image_path = os.path.join(self.depth_image_dir, f"frame_{self.image_index:06d}.npy")

            # 保存图像到路径
            cv2.imwrite(color_image_path, self.current_color_image)
            np.save(depth_image_path, self.current_depth_image)

            # 创建消息并发布图像路径
            image_paths_msg = String()
            image_paths_msg.data = color_image_path + "," + depth_image_path;
            self.image_path_publisher.publish(image_paths_msg)
            self.get_logger().info(f"发布图像路径: {image_paths_msg.data}")

            # 更新图像索引
            self.image_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImagePathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
