import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # 导入Odometry消息类型
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import networkx as nx
import cv2
from cv_bridge import CvBridge
import math
import os
from sklearn.cluster import DBSCAN
from std_msgs.msg import String  # 引入String消息类型用于发布图像路径
import ast
import numpy as np
import re
import json

class TopologicalMapNode(Node):
    def __init__(self):
        super().__init__('topological_map_node')
        
        # 初始化拓扑图
        self.graph = nx.Graph()
        self.nodes = []  # 节点列表
        self.edges = []  # 边列表
        self.image_paths = []  # 图像路径（可以根据需要修改）
        self.distance_threshold = 0.2  # 节点之间的最小距离     节点间距离大于0.2就保存
        self.file_path = 'topology_graph.graphml'  # 拓扑图文件路径
        self.last_object_position = None  # 用于存储上一个对象位置
        self.object_number = 0
        self.memory = ""

        # 建图时先清空物体名词
        self.object_sum_path = '/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/object_sum.txt'
        with open(self.object_sum_path, 'w') as file:  # 使用 'w' 模式清空文件
            pass  # 不需要写入任何内容，只需清空文件
        
        # 创建图像保存路径
        self.color_image_dir = '/home/lm/Desktop/catkin_ws/keymap_ws/keyframe/color/'
        self.depth_image_dir = '/home/lm/Desktop/catkin_ws/keymap_ws/keyframe/depth/'
        os.makedirs(self.color_image_dir, exist_ok=True)
        os.makedirs(self.depth_image_dir, exist_ok=True)

        # 创建发布器
        self.image_path_publisher = self.create_publisher(String, 'image_paths', 10)

        # 创建发布器发布三维坐标和四元数
        self.pose_publisher = self.create_publisher(Pose, 'pose_data', 10)

        # 订阅实时的pose话题
        self.subscription = self.create_subscription(
            Odometry,
            # '/viobot/pr_loop/odometry',  # viobot的话题名
            '/odom',  # 虚拟环境的话题
            self.odometry_callback,  # 新的回调函数
            10)

        # 订阅相机的彩色图像和深度图像话题
        self.color_image_subscription = self.create_subscription(
            Image,
            # '/camera/color/image_raw',    # gemini2的彩色图像话题
            '/intel_realsense_r200_depth/image_raw', # 虚拟环境的彩色图像话题   这两个话题是对应的
            self.color_image_callback,
            10)
        # Width:640, Height: 360
        self.depth_image_subscription = self.create_subscription(
            Image,
            # '/camera/depth/image_raw',    # gemini2的深度图像话题
            '/intel_realsense_r200_depth/depth/image_raw', # 虚拟环境的深度图像话题
            self.depth_image_callback,
            10)
        
        # 添加 object_node 的订阅器
        self.object_subscription = self.create_subscription(
            String,
            'object_node',  # 订阅 object_node 话题
            self.object_callback,
            10)

        # 用于存储当前帧图像
        self.current_color_image = None
        self.current_depth_image = None

        # 用于将ROS图像消息转换为OpenCV图像
        self.bridge = CvBridge()

    # 需要对应修改
    def determine_area(self, x, y):     
        """根据 x 和 y 坐标确定区域"""
        if 4.5 <= x <= 8.0 and -4.5 <= y <= 0.7:
            return "Kitchen"
        elif -2.3 < x < 4.5 and -4.5 < y <= 4.3:
            return "Living room"
        elif -9.2 <= x <= -2.3 and -5.0 <= y < 4.3:
            return "Bedroom"
        return "undefined"  # 如果坐标不在任何定义的区域内

    def publish_pose(self, pose):
        """发布三维坐标和四元数"""
        msg = Pose()
        msg.position.x = pose[0]
        msg.position.y = pose[1]
        msg.position.z = pose[2]
        msg.orientation.x = pose[3]
        msg.orientation.y = pose[4]
        msg.orientation.z = pose[5]
        msg.orientation.w = pose[6]

        # 发布消息
        self.pose_publisher.publish(msg)

    # 添加物体节点
    def object_callback(self, msg):
        """处理来自 object_node 话题的消息"""
        data = msg.data.split(',')
        print(data)
        
        if len(data) == 4:
            color_image_path = data[0]
            object_position = tuple(map(float, data[1:]))  # 将位置转换为元组
            
            file_path = '/home/lm/Desktop/catkin_ws/MobileSAM/object_name.txt'
            file_content = self.read_file_content(file_path)
            # 如果接收到的位置与上一个不同，才进行后续操作
            if (self.last_object_position is None or object_position != self.last_object_position) :
                # 添加节点
                if bool(file_content):
                    self.add_node_to_front(object_position)

                    new_node_index = self.object_number  # 获取新添加节点的索引

                    # 输出节点信息
                    self.get_logger().info(f"Added node with key=\"{new_node_index}\": {object_position}")
                    self.get_logger().info(f"Color image path with key=\"{new_node_index}\": {color_image_path}")

                    # 遍历所有节点的ID，进行距离比较    当物体被添加的时候，遍历物体节点之前的所有节点，进行比较
                    for node_id in self.graph.nodes():
                        if node_id == new_node_index:
                            continue  # 跳过新添加的节点自身
                        existing_pose = self.graph.nodes[node_id]['pos']
                        if isinstance(existing_pose, str):
                            existing_pose = ast.literal_eval(existing_pose)
                        position_array = list(existing_pose[:3])  # 提取前三个值
                        dist_to_existing = math.sqrt((object_position[0] - position_array[0]) ** 2 +
                                                    (object_position[1] - position_array[1]) ** 2)
                        # print("dist_to_existing:",dist_to_existing)
                        if new_node_index < 0 and dist_to_existing <= 0.7:
                            self.add_edge(new_node_index, node_id, dist_to_existing)
                    # 更新上一个对象位置
                    self.last_object_position = object_position
                else:
                    self.object_number-=1


    def odometry_callback(self, msg):
        # 提取当前Pose的x, y, z, 以及四元数 qx, qy, qz, qw
        current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # 发布三维坐标和四元数
        self.publish_pose(current_pose)

        if not self.nodes:
            # 如果是第一个节点，直接添加
            self.add_node(current_pose)
        else:
            # 计算与上一个节点的欧几里得距离
            previous_pose = self.nodes[-1]  # 获取最后一个节点

            dist = math.sqrt((current_pose[0] - previous_pose[0]) ** 2 +
                            (current_pose[1] - previous_pose[1]) ** 2)  # 只考虑 x 和 z 坐标  #viobot 考虑x和y
            
            if dist >= self.distance_threshold and dist <= 0.5:
                # 如果距离大于阈值，则添加新节点
                self.add_node(current_pose)
                
                # 遍历现有节点，与新节点进行距离比较
                new_node_index = len(self.nodes) - 1 + self.object_number  # 当前节点的索引
                
                # 保存彩色图像和深度图像   
                self.save_images(new_node_index)
                
                # 发布图像路径
                self.publish_image_paths(new_node_index)

                # 添加与前一个节点的边
                self.add_edge(len(self.nodes) - 2 + self.object_number, new_node_index, dist)  # 前一个节点的索引是新节点索引减一

                # 遍历所有节点的ID，进行距离比较
                for node_id in self.graph.nodes():

                    existing_pose = self.graph.nodes[node_id]['pos']
                    if isinstance(existing_pose, str):
                        existing_pose = ast.literal_eval(existing_pose)

                    position_array = list(existing_pose[:3])  # 提取前三个值
                    dist_to_existing = math.sqrt((current_pose[0] - position_array[0]) ** 2 +
                                                (current_pose[1] - position_array[1]) ** 2)

                    if (node_id < 0 and dist_to_existing <= 0.7):   # 如果遍历的节点有负数并且距离小于0.7，则添加节点
                        self.add_edge(node_id, new_node_index, dist_to_existing)
                    if dist_to_existing <= self.distance_threshold and dist_to_existing != 0.0:
                        self.add_edge(node_id, new_node_index, dist_to_existing)

        # 每次添加节点和边后保存拓扑图
        self.save_graph_to_file()

    def publish_image_paths(self, node_index):
        """发布彩色图像和深度图像的路径"""
        color_image_path = os.path.join(self.color_image_dir, f"frame_{node_index:06d}.png")
        depth_image_path = os.path.join(self.depth_image_dir, f"frame_{node_index:06d}.npy")

        # 创建字符串消息
        msg = String()
        msg.data = color_image_path + "," + depth_image_path;
        print(color_image_path)
        
        # 发布消息
        self.image_path_publisher.publish(msg)

    def color_image_callback(self, msg):
        """处理来自/color/image_raw话题的彩色图像"""
        self.current_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_image_callback(self, msg):
        """处理来自/depth/image_raw话题的深度图像"""
        self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def add_node(self, pose):
        """向图中添加节点"""
        node_index = len(self.nodes) + self.object_number
        print("add_node",node_index)
        self.nodes.append(pose)
        area = self.determine_area(pose[0], pose[1])  # 根据位置确定区域
        image_path = f"frame_{node_index:06d}.png"  # 假设图像路径格式
        self.image_paths.append(image_path)
        self.graph.add_node(node_index, pos=pose, image=image_path, area=area, memery = self.memory)

    def read_file_content(self,file_path):
        """
        读取文件内容，如果文件内容为空则返回空字符串。
        :param file_path: 文件路径
        :return: 文件内容字符串，如果文件为空或不存在则返回空字符串
        """
        if os.path.exists(file_path):  # 确保文件存在
            with open(file_path, 'r') as file:
                content = file.read().strip()  # 读取并去除首尾空格
                return content  # 返回内容（可能为空字符串）
        return ""  # 文件不存在时返回空字符串

    def add_node_to_front(self, pose):      # 加上名称和图片  图片已经加载完成，就差添加名称了
        """向图中添加节点，并将其存入列表的最前面"""
        # 使用 -1 的起始序号
        node_index = -1 + self.object_number
        self.object_number -= 1
        self.nodes.insert(0, pose)  # 将节点添加到列表的最前面
        image_path = f"object_{(-node_index):06d}.png"  # 更新图像路径格式
        
        self.image_paths.insert(0, image_path)  # 同样更新图像路径列表
        area = self.determine_area(pose[0], pose[1])  # 根据位置确定区域
        file_path = '/home/lm/Desktop/catkin_ws/MobileSAM/object_name.txt'
        file_content = self.read_file_content(file_path)

        # 根据文件内容决定是否添加名称属性
        if file_content:
            self.graph.add_node(node_index, pos=pose, image=image_path, area=area, object_name=file_content, memery = self.memory)  # 如果文件不为空，添加节点并附带名称
        else:
            self.graph.add_node(node_index, pos=pose, image=image_path, area=area, memery = self.memory)  # 如果文件为空，仅添加节点
        
        # 将信息追加到 object_sum.txt 中 (JSON 格式)
        node_data = {
            "object_name": file_content,
            "object_index": node_index,
            "object_pose": pose
        }
        
        with open(self.object_sum_path, 'a') as file:
            file.write(json.dumps(node_data, ensure_ascii=False) + '\n')

    def add_edge(self, node1_idx, node2_idx, distance):
        """向图中添加边"""
        self.edges.append((node1_idx, node2_idx, {"distance": distance}))
        self.graph.add_edge(node1_idx, node2_idx, distance=distance)

    def save_images(self, node_index):
        """保存彩色图像和深度图像"""
        if self.current_color_image is not None:
            color_image_path = os.path.join(self.color_image_dir, f"frame_{node_index:06d}.png")
            cv2.imwrite(color_image_path, self.current_color_image)

        if self.current_depth_image is not None:
            depth_image_path = os.path.join(self.depth_image_dir, f"frame_{node_index:06d}.npy")
            # 保存深度图像为 .npy 文件
            np.save(depth_image_path, self.current_depth_image)

    def save_graph_to_file(self):
        """将拓扑图保存到文件中"""
        try:
            # 确保所有节点和边的属性为GraphML支持的类型
            for node, data in self.graph.nodes(data=True):
                for key, value in data.items():
                    if not isinstance(value, (str, int, float)):
                        data[key] = str(value)  # 转换为字符串

            for u, v, data in self.graph.edges(data=True):
                for key, value in data.items():
                    if not isinstance(value, (str, int, float)):
                        data[key] = str(value)  # 转换为字符串

            nx.write_graphml(self.graph, self.file_path)
            # self.get_logger().info(f"拓扑图已成功保存为: {self.file_path}")
        except Exception as e:
            self.get_logger().error(f"保存图时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TopologicalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
