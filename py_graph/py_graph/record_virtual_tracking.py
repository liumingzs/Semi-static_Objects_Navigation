import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import os

class OdometrySaver(Node):

    def __init__(self):
        super().__init__('odometry_saver')

        # 设置文件路径
        
        self.file_path = '/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/for_record_virtual_tracking.txt'

        # 清空文件内容（初始化时只执行一次）
        with open(self.file_path, 'w') as file:
            file.write("")  # 清空文件内容

        # 上次保存的位置
        self.last_pos_x = None
        self.last_pos_y = None
        self.last_pos_z = None

        # 设置保存间隔：0.2 米
        self.save_interval = 0.2

        # 创建订阅器，订阅 /odom 话题
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_position,
            10
        )

    def callback_position(self, msg):
        # 获取当前位置 (x, y, z)
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z

        # 获取姿态数据 (四元数: x, y, z, w)
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        # 如果是第一次获取位置，初始化 last_pos
        if self.last_pos_x is None or self.last_pos_y is None or self.last_pos_z is None:
            self.last_pos_x = pos_x
            self.last_pos_y = pos_y
            self.last_pos_z = pos_z
            return  # 不保存数据，直到后续位置变化

        # 计算当前位置与上次保存位置之间的距离
        distance = math.sqrt(
            (pos_x - self.last_pos_x) ** 2 +
            (pos_y - self.last_pos_y) ** 2 
        )

        # 如果距离超过保存间隔，保存数据
        if distance >= self.save_interval:
            # 获取时间戳
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

            # 打开文件并写入数据
            with open(self.file_path, 'a') as file:
                file.write(f"{timestamp} {pos_x} {pos_y} {pos_z} {orientation_x} {orientation_y} {orientation_z} {orientation_w}\n")
            
            # 更新上次保存的位置
            self.last_pos_x = pos_x
            self.last_pos_y = pos_y
            self.last_pos_z = pos_z

            # 输出日志，确认保存
            self.get_logger().info(f"Data saved: {timestamp} {pos_x} {pos_y} {pos_z} {orientation_x} {orientation_y} {orientation_z} {orientation_w}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
