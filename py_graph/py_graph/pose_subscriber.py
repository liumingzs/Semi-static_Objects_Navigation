import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os
import time
import math

class OdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/viobot/pr_loop/odometry',  # 订阅 /viobot/pr_loop/odometry 话题
            self.listener_callback,
            10)
        self.subscription  # 防止未使用的变量警告
        self.output_file = "viobot_odometry.txt"
        self.start_time = time.time()

        # 确保文件开始时为空
        with open(self.output_file, 'w') as f:
            pass
        
        # 上一次保存的位置，初始化为(0, 0)
        self.last_position = (0.0, 0.0)
        self.distance_threshold = 0.1  # 0.1米为保存数据的阈值

    def listener_callback(self, msg):
        current_time = time.time() - self.start_time

        # 获取当前的位置 (x, y)
        position = msg.pose.pose.position
        current_position = (position.x, position.y)

        # 计算当前位置与上一个保存位置的欧式距离
        distance = math.sqrt((current_position[0] - self.last_position[0]) ** 2 + 
                             (current_position[1] - self.last_position[1]) ** 2)

        # 如果当前位置与上次保存的位置距离超过阈值，则保存数据
        if distance >= self.distance_threshold:
            # 获取方向（四元数：x, y, z, w）
            orientation = msg.pose.pose.orientation

            # TUM格式：时间戳, tx, ty, tz, qx, qy, qz, qw
            odometry_data = f"{current_time:.6f} {position.x:.6f} {position.y:.6f} {position.z:.6f} " \
                            f"{orientation.x:.6f} {orientation.y:.6f} {orientation.z:.6f} {orientation.w:.6f}\n"

            # 追加到文件
            with open(self.output_file, 'a') as f:
                f.write(odometry_data)

            # 更新最后保存的位置
            self.last_position = current_position

            self.get_logger().info(f"Odometry saved: {odometry_data.strip()}")

def main(args=None):
    rclpy.init(args=args)

    odometry_subscriber = OdometrySubscriber()

    rclpy.spin(odometry_subscriber)

    # 清理和关闭
    odometry_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
