import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ImageToVideo(Node):

    def __init__(self):
        super().__init__('image_to_video')

        # 参数设置 (可选)
        self.declare_parameter('output_video_path', '/home/lm/Videos')  # 默认保存路径
        self.declare_parameter('video_name', 'output.mp4')  # 默认视频文件名
        self.declare_parameter('fps', 30.0)  # 默认帧率

        self.output_video_path = self.get_parameter('output_video_path').get_parameter_value().string_value
        self.video_name = self.get_parameter('video_name').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value

        # 创建保存目录，如果不存在
        if not os.path.exists(self.output_video_path):
            os.makedirs(self.output_video_path)

        # 确定完整的文件路径
        self.full_video_path = os.path.join(self.output_video_path, self.video_name)


        # 创建 CvBridge
        self.bridge = CvBridge()

        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/image_raw',  # 修改为您的图像话题
            self.image_callback,
            10)  # QoS 设置
        self.subscription  # prevent unused variable warning

        self.video_writer = None
        self.frame_width = None
        self.frame_height = None

    def image_callback(self, msg):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')  # 或 'rgb8', 'mono8'等
        except Exception as e:
            self.get_logger().error(f"转换图像出错: {e}")
            return

        # 初始化视频编写器 (第一次收到图像时)
        if self.video_writer is None:
            self.frame_width = cv_image.shape[1]  # 获取图像宽度
            self.frame_height = cv_image.shape[0] # 获取图像高度
            fourcc = cv2.VideoWriter_fourcc(*'MP4V')  # 编解码器，可以尝试 'XVID', 'MJPG', 'MP4V'
            try:
                self.video_writer = cv2.VideoWriter(self.full_video_path, fourcc, self.fps, (self.frame_width, self.frame_height))
                self.get_logger().info(f"开始录制视频: {self.full_video_path},  {self.frame_width}x{self.frame_height} @ {self.fps} FPS")
            except Exception as e:
                self.get_logger().error(f"创建视频编写器出错: {e}")
                return

        try:
            # 颜色空间转换 (如果需要)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 写入帧到视频
            self.video_writer.write(cv_image)
            #self.get_logger().info("写入一帧") # 为了减少打印，注释掉

        except Exception as e:
            self.get_logger().error(f"写入帧出错: {e}")

    def destroy_node(self):
        # 释放视频编写器
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"视频录制完成，已保存到: {self.full_video_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    image_to_video = ImageToVideo()
    try:
        rclpy.spin(image_to_video)
    except KeyboardInterrupt:
        image_to_video.get_logger().info('手动停止节点...')
    finally:
        # Clean up and shutdown
        image_to_video.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()