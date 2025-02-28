# 根据话题读取到的第一个位姿当作起点，终点位置需要自己输入，然后生成的最短路径会保存在route_trajectory.txt文件中
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import networkx as nx
from PIL import Image
import os
import time
import math
import sys 
import re 
import requests
import json
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# 读取 .graphml 文件
G = nx.read_graphml("/home/lm/Desktop/catkin_ws/keymap_ws/topology_graph.graphml")

# 输出图的信息
print(f"节点数: {G.number_of_nodes()}")
print(f"边数: {G.number_of_edges()}")

# 解析位置字符串
def parse_position(pos_str):
    try:
        pos_str = pos_str.strip("()")
        pos_values = list(map(float, pos_str.split(",")))
        if len(pos_values) == 3:
            return tuple(pos_values)
        elif len(pos_values) == 7:
            return tuple(pos_values)
        else:
            raise ValueError(f"位置字符串的元素数不正确: {pos_str}")
    except Exception as e:
        print(f"无法解析位置字符串 {pos_str}: {e}")
        return (0.0, 0.0, 0.0)

# 提取节点和边
def extract_nodes_and_edges(G):
    nodes = {}
    for node, data in G.nodes(data=True):
        pos_str = data['pos']
        pos = parse_position(pos_str)
        nodes[node] = pos
    edges = [(start, end, edge_data['distance']) for start, end, edge_data in G.edges(data=True)]
    return nodes, edges

# 查找最近的节点
def find_nearest_node(G, current_position):
    min_distance = float('inf')
    nearest_node = None
    for node, data in G.nodes(data=True):
        pos = parse_position(data['pos'])
        distance = math.sqrt((pos[0] - current_position[0]) ** 2 + (pos[1] - current_position[1]) ** 2 + (pos[2] - current_position[2]) ** 2)
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node

# 查找节点ID根据object_name或节点ID
def find_node_id(G, node_identifier):
    for node, data in G.nodes(data=True):
        if data.get('object_name') == node_identifier:
            with open("end_object_name.txt", "w") as file:
                file.write(node_identifier)
            return node
        elif node == node_identifier:
            return node
    return None

# 查找最短路径
def find_shortest_path(G, start_id, end_id):
    try:
        path = nx.shortest_path(G, source=start_id, target=end_id, weight='distance')
        return path
    except nx.NetworkXNoPath:
        print("找不到路径")
        return None

# 保存路径为TUM格式
def save_path_to_tum(G, path):
    with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/route_trajectory.txt", "w") as file:
        for node in path:
            pos = G.nodes[node].get("pos", "(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)")
            position = parse_position(pos)
            if len(position) == 7:
                x, y, z, qx, qy, qz, qw = position
            else:
                x, y, z = position
                qx = qy = qz = 0.0
                qw = 1.0
            timestamp = time.time()
            file.write(f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
    print("路径已保存至 route_trajectory.txt 文件")

def get_object_name_from_message( model="gpt-3.5-turbo-0125"):
    """
    通过调用 GPT API 从用户输入的消息中提取物体名词对应的序号
    :param user_message: 用户输入的消息（例如 "我想要获取到vase的位置"）
    :param model: 使用的模型（默认为 gpt-3.5-turbo-0125）
    :return: 提取出的物体名词
    """
    
    user_message = input("请输入您的寻找指令:")

    file_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/object_sum.txt"
    object_data = []
    # 打开文件并读取每一行
    with open(file_path, 'r') as file:
        for line in file:
            try:
                # 解析每一行 JSON 数据
                object_info = json.loads(line.strip())
                object_data.append(object_info)
            except json.JSONDecodeError:
                continue  # 如果解析失败，跳过该行
    # API URL和授权信息
    api_key = "sk-V8Rbc18fdd54f64583d559271b56cb218cae95c3e6fBz6wg"
    url = "https://api.gptsapi.net/v1/chat/completions"
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }

    # 请求数据
    data = {
        "model": model,
        "messages": [
            {"role": "user", "content": f"我会提供给你一个数组，数组中每个元素都包含了物体节点的名词和对应的节点:{object_data}，请你从下面的句子中推理出最对应的物体的名词：{user_message},你只用返回物体节点的名词，不需要翻译成中文，不用返回其他的任何内容"}
        ]
    }

    # 发送POST请求
    response = requests.post(url, headers=headers, data=json.dumps(data))

    # 获取并返回 'content' 字段
    response_json = response.json()
    content = response_json['choices'][0]['message']['content']
    print(f"为您寻找到的物体为{content}")
    return content.strip()

# 检查 model_relocation.txt 文件是否为空并读取终点
def get_end_id_from_file_or_input():
    file_path = "/home/lm/Desktop/catkin_ws/MobileSAM/model_relocation.txt"
    
    if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
        print("文件非空")
        # 文件非空，读取文件中的"node"后的数字
        with open(file_path, "r") as file:
            file_content = file.read()
        # 使用正则表达式提取 "node" 的值，支持正数和负数
        pattern = r'["\']node["\']:\s*(-?\d+)'
        matches = re.findall(pattern, file_content)
        if matches:
            return matches[-1]  # 返回最后一个匹配的数字作为终点
    # 如果文件为空或不存在，使用gpt返回的终点序列号
    return get_object_name_from_message()

# 检查 end_id 是否可以在 processed_path.txt 中匹配获得
def is_end_id_in_processed_path(end_id):
    processed_path_file = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/processed_path.txt"
    try:
        with open(processed_path_file, "r") as file:
            content = file.read()
            # 使用正则表达式匹配 end_id
            if re.search(rf"\b{end_id}\b", content):
                return True
    except FileNotFoundError:
        print(f"文件 {processed_path_file} 未找到。")
    return False

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )
        self.current_position = None
        self.path_generated = False  # 用于指示路径是否已生成

    def odometry_callback(self, msg):

        if self.path_generated:
            return

        # 从 /viobot/pr_loop/odometry 中获取当前位置信息
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        print(f"当前位置：{self.current_position}")

        # 查找与当前位置最近的节点
        start_id = find_nearest_node(G, self.current_position)
        
        # 查找用户指定的终点
        end_identifier = get_end_id_from_file_or_input()
        end_id = find_node_id(G, end_identifier)    # end_id是否为负的暂时先不管
        print(f"起始节点ID: {int(start_id)},end_id:{int(end_id)}")
        # if int(end_id) <= int(start_id)  and int(end_id) > 0:     修改成了下面的判读
        if  is_end_id_in_processed_path(end_id):   
            print("该节点已被遍历，请跳过")

            with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/memory_point_record.txt", "w") as file:
                file.write("1")
            sys.exit()
 
        if start_id and end_id:
            if(int(start_id) > 5):
                path = find_shortest_path(G, str(int(start_id)-5), end_id)    # 应该是没找到前面的节点，尝试一下试试
            else:
                # 查找最短路径
                path = find_shortest_path(G, start_id, end_id)
            if path:
                print("最短路径:", path)

                # 删除负数节点
                path = [node for node in path if int(node) >= 0]  # 只保留非负数节点

                # 判断数据的规律（递增或递减）
                if len(path) >= 2:
                    # 判断从第二个节点到第四个节点是否递增
                    is_increasing = all(int(path[i]) < int(path[i + 1]) for i in range(1, 3))  # 检查 path[1] < path[2] 和 path[2] < path[3]
                    
                    # 判断从第二个节点到第四个节点是否递减
                    is_decreasing = all(int(path[i]) > int(path[i + 1]) for i in range(1, 3))  # 检查 path[1] > path[2] 和 path[2] > path[3]

                # 检查最后一个元素是否小于 160
                if len(path) > 0 and int(path[-1]) < 160:
                    # 根据规律添加五个数
                    last_number = int(path[-1])
                    for _ in range(5):
                        if is_increasing:
                            last_number += 1  # 递增
                        else:
                            last_number -= 1  # 递减
                        path.append(str(last_number))  # 确保数据格式为字符串

                # print("处理后的路径:", path)

                # 检查路径前四个节点
                if len(path) >= 4:
                    first_four_nodes = path[:4]
                    # print("前四个节点:", first_four_nodes)

                    is_increasing = all(int(first_four_nodes[i]) < int(first_four_nodes[i + 1]) for i in range(len(first_four_nodes) - 1))
                    is_decreasing = all(int(first_four_nodes[i]) > int(first_four_nodes[i + 1]) for i in range(len(first_four_nodes) - 1))
                    
                    # 判断节点序号是否递增 递增说明不需要调头
                    if is_increasing or is_decreasing:
                        print("节点序号递增，更新flag_reverse")
                        with open('/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/flag_reverse.txt', 'w') as file:
                            file.write('1')
                    else:
                        print("节点序号未递增，不更新")


                output_file_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/processed_path.txt"
                with open(output_file_path, "a") as file:  # 使用 "a" 模式追加写入
                    file.write(str(path) + "\n")  # 直接将列表写入文件

                # 保存路径至TUM格式
                save_path_to_tum(G, path)

                sys.exit(0)  # 在此处终止进程
                
                # 路径生成完成后设置标志并销毁节点
                self.path_generated = True
                self.destroy_node()  # 销毁节点
                rclpy.shutdown()  # 停止ROS环境
                
        else:
            print("找不到起始点或终点")
        
        

def main(args=None):
    rclpy.init(args=args)
    odometry_subscriber = OdometrySubscriber()
    rclpy.spin(odometry_subscriber)  # 运行节点
    print("节点已成功停止")

if __name__ == '__main__':
    main()
