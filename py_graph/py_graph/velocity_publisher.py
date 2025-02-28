import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import os
import select

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initial velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # State: whether vehicle is started
        self.started = False

        self.get_logger().info("Node initialized. Press 'i' to start the vehicle.")
        
        # Start the key listening in a separate thread
        self.listen_keyboard()

    def publish_velocity(self):
        """Publish the current velocity."""
        if self.started:
            twist = Twist()
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity
            self.publisher_.publish(twist)
            self.get_logger().info(f"Published velocities - Linear: {self.linear_velocity}, Angular: {self.angular_velocity}")

    def listen_keyboard(self):
        """Listen for keyboard inputs non-blocking."""
        def get_key():
            """Non-blocking key press capture."""
            dr, dw, de = select.select([sys.stdin], [], [], 0.1)
            if dr:
                return sys.stdin.read(1)
            return None

        while rclpy.ok():
            key = get_key()
            if key:
                if key == 'w':
                    self.linear_velocity += 0.1
                    self.get_logger().info(f"Increased linear velocity: {self.linear_velocity}")
                elif key == 's':
                    self.linear_velocity -= 0.1
                    self.get_logger().info(f"Decreased linear velocity: {self.linear_velocity}")
                elif key == 'a':
                    self.angular_velocity += 0.1
                    self.get_logger().info(f"Increased angular velocity: {self.angular_velocity}")
                elif key == 'd':
                    self.angular_velocity -= 0.1
                    self.get_logger().info(f"Decreased angular velocity: {self.angular_velocity}")
                elif key == 'i' and not self.started:
                    self.started = True
                    self.get_logger().info("Vehicle started.")
                
                # Immediately publish the updated velocity
                self.publish_velocity()  # 发布速度


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
