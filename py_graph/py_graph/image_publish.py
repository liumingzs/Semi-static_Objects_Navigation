import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ImagePathPublisher(Node):
    def __init__(self):
        super().__init__('image_path_publisher')
        self.publisher_ = self.create_publisher(String, 'image_paths', 1)
        
        # 定义要发布的图像路径
        color_image_path = "/home/lm/Desktop/catkin_ws/keymap_ws/keyframe/color/frame_000006.png"
        depth_image_path = "/home/lm/Desktop/catkin_ws/keymap_ws/keyframe/depth/frame_000006.png"
        
        # 创建消息内容
        message = color_image_path + "," + depth_image_path;
        
        # 发布消息
        self.publish_image_paths(message)

    def publish_image_paths(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    image_path_publisher = ImagePathPublisher()
    rclpy.spin(image_path_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
