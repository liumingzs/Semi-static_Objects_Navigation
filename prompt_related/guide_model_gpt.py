import time
import requests
import subprocess
import json
import xml.etree.ElementTree as ET
import re
import math
import sys

# =================== 文件检查功能 ===================
def wait_for_file_content(file_path, target_content="1", check_interval=0.5):
    """
    循环检查文件内容，直到内容为指定值。
    :param file_path: 文件路径
    :param target_content: 目标内容
    :param check_interval: 检查间隔时间（秒）
    """
    while True:
        try:
            with open(file_path, "r") as file:
                content = file.read().strip()
            if content == target_content:
                print(f"文件内容为 '{target_content}'，继续执行后续代码...")
                break
            else:
                # print(f"文件内容为 '{content}'，等待 {check_interval} 秒后重试...")
                time.sleep(check_interval)
        except FileNotFoundError:
            print(f"文件未找到：{file_path}")
            time.sleep(check_interval)


# =================== 拓扑图解析功能 ===================
def get_area_and_node_for_object(object_name, graphml_file):
    """
    从拓扑图文件中查找指定物体所在区域及节点ID。
    :param object_name: 物体名称
    :param graphml_file: GraphML 文件路径
    :return: 匹配的区域和节点ID
    """
    namespaces = {'ns0': 'http://graphml.graphdrawing.org/xmlns'}  # 根据实际命名空间调整

    try:
        tree = ET.parse(graphml_file)
        root = tree.getroot()

        # 遍历所有节点
        for node in root.findall('.//ns0:node', namespaces):
            object_name_in_node = node.find(".//ns0:data[@key='d4']", namespaces)
            if object_name_in_node is not None and object_name_in_node.text.strip() == object_name:
                area_element = node.find(".//ns0:data[@key='d2']", namespaces)
                if area_element is not None:
                    area = area_element.text.strip()
                    node_id = node.get("id")
                    return area, node_id
    except Exception as e:
        print(f"解析拓扑图文件时出错：{e}")

    return None, None


# =================== 文件检查功能 ===================
def check_navigation_status(file_path):
    """
    检查导航状态文件内容。
    :param file_path: 文件路径
    """
    try:
        with open(file_path, 'r') as file:
            content = file.read().strip()
        if content == "1":
            return True
        else:
            return False
    except FileNotFoundError:
        print(f"文件未找到：{file_path}")
    except Exception as e:
        print(f"读取文件时发生错误：{e}")


# =================== 模型调用与处理 ===================
def generate_relocation_request(object_name, area, node_id, second_call_instruction=None):
    """
    生成物体位置推测请求。
    :param object_name: 物体名称
    :param area: 区域名称
    :param node_id: 节点ID
    """
    existing_content = '''你是一个智能物体查找助手,你的任务是根据给定的物体信息和拓扑图,提供物体可能的当前位置。每个区域包含多个节点,我们将根据物体之前的位置推测它的可能新位置,请你只回复一次答案，回复的格式如下所示。
    - 拓扑图中有以下物体："vase", "blue garbage bin","coca cola can","biscuit box","monocular camera",这些物体对应的位置分别如下所示
        - "vase"的具体位置为[3.180166, 0.492778],它的大致位置在"Living room"的左上角，它的序号为-1
        - "blue garbage bin"的具体位置为[2.428859, -4.420703],它的大致位置在"Living room"的右上角，它的序号为-4
        - "coca cola can"的具体位置为[-3.884565, -0.266045],它的大致位置在"Bedroom"的左上角，它的序号为-12
        - "biscuit box"的具体位置为[-5.747039, -3.668698],它的大致位置在"Bedroom"的右侧区域的中间位置，它的序号为-8
    - 拓扑图包含3个区域:Living room,Kitchen,Bedroom,每个区域的节点序号如下所示，下面的节点都是按照数组里的位置顺序排列的:
        - Living room:
            - [146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 0, 159, 1,  2, 167, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, -1, 14, 15, 16, 17, 18, 19, 20, 21]，这些节点排列在Living room的左侧，排列顺序是从下到上
            - [59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, -4, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92], 这些节点排列在Living room的右侧，排列顺序是从上到下
        - Kitchen: 
            - [22, 23, 24, 25, 26, 27, 28, 29], 这些节点排列在Kitchen的左侧 ，排列顺序是从下到上
            - [30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51]这些节点排列在Kitchen的上方 ，排列顺序是从左到右
            - [52, 53, 54, 55, 56, 57, 58], 这些节点排列在Kitchen的右侧 ，排列顺序是从上到下
        - Bedroom: 
            - [93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, -8, 105, 106, 107, 108], 这些节点排列在Bedroom的右侧, 排列顺序是从上到下
            - [109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128]，这些节点排列在Bedroom的下方，排列顺序是从右到左
            - [129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, -10, 141, 142, 143, 144, 145], 这些节点排列在Bedroom的左侧, 排列顺序是从下到上
    - 场景中的物体如下所示，它们在拓扑图中没有指定序号：
        - "dining table"的具体位置为[6.57,1.83],它的大致位置在"Kitchen"的左侧区域的中间位置
        - "refrigerator"的具体位置为[8.702,-1.03],它的大致位置在"Kitchen"的上方区域的中间位置
        - "sofa"的具体位置为[0.33,-1.90],它的大致位置在"Living room"的右侧区域的中间位置
        - "Bedroom Chair"的具体位置为[-8.27,-4.504],它的大致位置在"Bedroom"的右侧区域的下方位置
        - "bed"的具体位置为[-6.16,2.030],它的大致位置在"Bedroom"的左侧区域的中间位置
        - "TV"的具体位置为[0.33,-1.90],它的大致位置在"Living room"的右侧区域的中间位置
        - "kitchen cabinet"的具体位置为[7.889,-4.071],它的大致位置在"Kitchen"的右侧区域的上方位置
        - "shoeRack"的具体位置为[4.297,-5.173],它的大致位置在"Living room"的右侧区域的上方位置
        - "ReadingDesk"的具体位置为[-8.987,2.057],它的大致位置在"Bedroom"的左侧区域的下方位置
    【以下是具体的指令，思考推论及回复示例】
    - 我的指令："vase"原来在"Living room"的"-1"号节点，现在它不见了，请你输出可能的位置
        - 你的回复:"vase"一般用作装饰物，可能会出现在"Living room",放在"Living room"中的"shoeRack"附近用于装饰鞋柜，由于"shoeRack"的大致位置在"Living room"的右侧区域的上方位置，由于[59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, -4, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92]这些节点按照从上到下的顺序排列在Living room的右侧，所以上方位置可能在[59, 60, 61, 62]这些节点中，从中任意挑选一个节点输出的话，会输出"node":60
    - 我的指令："blue garbage bin"原来在"Living room"的"-4"号节点，现在它不见了，请你输出可能的位置
        - 你的回复:"blue garbage bin"一般用于盛放垃圾，可能会出现在"Kitchen"，放在"Kitchen"中的"dinning table"附近用于盛放垃圾，由于"dining table"的大致位置在"Kitchen"的左侧区域的中间位置，由于[22, 23, 24, 25, 26, 27, 28, 29]这些节点按照从下到上的顺序排列在Kitchen的左侧, 所以中间位置可能在[24, 25, 26]这些节点中，从中任意挑选一个节点输出的话，会输出"node":25
    - 我的指令："blue garbage bin"原来在"Living room"的"-4"号节点，现在它不见了，请你输出可能的位置                   
        - 你的回复:"blue garbage bin"一般用于盛放垃圾，可能会出现在"Bedroom"，放在"Bedroom"中的"ReadingDesk"附近用于盛放垃圾，由于"ReadingDesk"的大致位置在"Bedroom"的下方区域的左侧位置，由于[109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128]这些节点按照从右到左的顺序排列在Bedroom的下方，从中任意挑选一个节点输出的话，所以左侧位置可能在[122, 123, 124, 125, 126, 127, 128]这些节点中，从中任意挑选一个节点输出的话，会输出"node":125
    【回复格式】
        -"xxx"一般用于xxx,可能会出现在"xxx",放在"xxx"里的"xxxx"附近用于xxx, 由于"xxx"的大致位置在"xxx"的x侧区域的x方位置,由于[xx,xx,xx,xxx,xxx]这些节点按xxxx的顺序排列在"xxx"的x侧，所以下方位置可能在[xx,xx,xx,xxx]这些节点中，从中任意挑选一个节点输出的话，会输出"node":xxx
    我现在的指令是：
    '''

    if second_call_instruction: #如果有最后一个参数传入
        # 读取文件内容
        output_filename = "model_relocation.txt"
        with open(output_filename, 'r') as file:
            model_relocation_content = file.read()
        additional_content = f"{object_name}原来在{area}的{node_id}号节点,现在{object_name}不见了，请你输出可能的位置,你需要排除掉{{{model_relocation_content}}}这个推理，提供新的按照回复格式回答的推理答案"

    else:   #如果没有最后一个参数传入
        additional_content = f"{object_name}原来在{area}的{node_id}号节点,现在{object_name}不见了，请你输出可能的位置"

    new_content = existing_content + "\n" + additional_content

    # 请求数据
    data = {
        "model": "gpt-4o",
        "messages": [
            {"role": "user", "content": new_content}
        ]
    }

    # API URL 和请求头
    url = "https://api.gptsapi.net/v1/chat/completions"
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer sk-V8Rbc18fdd54f64583d559271b56cb218cae95c3e6fBz6wg"
    }

    # 发送请求并保存结果
    response = requests.post(url, headers=headers, data=json.dumps(data))
    response_json = response.json()
    content = response_json['choices'][0]['message']['content']

    # 将响应写入文件
    output_filename = "model_relocation.txt"
    with open(output_filename, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"推理完成--{content}--已保存至 {output_filename}")


# =================== ROS 2 节点启动功能 ===================
def run_ros2_node(workspace_dir, command):
    """
    在指定的工作空间中启动 ROS 2 节点。
    :param workspace_dir: 工作空间路径
    :param command: 要执行的命令
    """
    try:
        subprocess.run(
            f"bash -c 'cd {workspace_dir} && source install/setup.bash && {command}'",
            shell=True,
            check=True
        )
        print(f"ROS 2 节点 '{command}' 成功启动！")
    except subprocess.CalledProcessError as e:
        print(f"启动 ROS 2 节点失败: {e}")

# ================= 获取反向巡迹的路径点 ===============
def reverse_file_content(input_file, output_file):
    """
    倒置文件内容并写入新文件。
    :param input_file: 输入文件路径
    :param output_file: 输出文件路径
    """
    try:
        with open(input_file, "r") as file:
            lines = file.readlines()
        
        # 倒置文件内容
        reversed_lines = lines[::-1]
        
        with open(output_file, "w") as file:
            file.writelines(reversed_lines)
        
        print(f"文件内容已成功倒置并保存到 {output_file}")
    except FileNotFoundError:
        print(f"输入文件未找到：{input_file}")
    except Exception as e:
        print(f"处理文件时发生错误：{e}")

# ================= 获取记忆节点的序号和位置 ===============
def get_memory_object_name_from_message():
    """
    通过调用 GPT API 获取到目标物体所对应的拥有属性的记忆节点
    :param end_object_name: 目标物体的名称
    :param model: 使用的模型（默认为 gpt-3.5-turbo-0125
    :return: 提取出对应的节点
    """

    # 读取 end_object_name.txt 文件内容
    with open("/home/lm/Desktop/catkin_ws/keymap_ws/end_object_name.txt", "r", encoding="utf-8") as file:
        end_object_name = file.read().strip()

    file_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/object_memory.txt"
    object_data = []

    # 读取 object_memory.txt 文件
    try:
        with open(file_path, 'r') as file:
            for line in file:
                try:
                    # 使用正则表达式匹配行内容
                    match = re.match(r'^(.+)\s+(\d+)\s+\[(.*?)\]\s+(\d+)$', line.strip())
                    if match:
                        object_name = match.group(1).strip()  # 物体名称，去除前后空格
                        node_id = match.group(2)     # 节点ID
                        position = match.group(3)    # 位置
                        attribute = match.group(4)   # 属性
                        print(f"object_name: '{object_name}', end_object_name: '{end_object_name}'")
                        
                        # 只有当 object_name 等于 end_object_name 时才添加到 object_data
                        if object_name == end_object_name:
                            object_info = {
                                "object_name": object_name,
                                "node_id": node_id,
                                "position": [float(x) for x in position.split(', ')],
                                "attribute": int(attribute)
                            }
                            object_data.append(object_info)
                except ValueError as ve:
                    print(f"解析失败: {ve}")
                    continue  # 如果解析失败，跳过该行
    except FileNotFoundError:
        print(f"错误: 文件 {file_path} 未找到。")
        return None
    except Exception as e:
        print(f"读取文件时发生错误: {e}")
        return None
    if not object_data:
        print("未从 object_memory.txt 中读取到有效数据。")
        return None
        
    # API URL和授权信息
    api_key = "sk-V8Rbc18fdd54f64583d559271b56cb218cae95c3e6fBz6wg"
    url = "https://api.gptsapi.net/v1/chat/completions"
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }

    # 请求数据
    data = {
        "model": "gpt-3.5-turbo",
        "messages": [
            {"role": "user", "content": f"我会提供给你一个数组，数组中每个元素都包含了物体节点序号，物体节点位置和物体节点的属性，数组为:{object_data}，你唯一要做的就是提取出{end_object_name}对应的所有元素中的position属性并返回,除此之外不用返回其他任何输出"}
        ]
    }

    # 发送POST请求
    response = requests.post(url, headers=headers, data=json.dumps(data))

    # 获取并返回 'content' 字段
    response_json = response.json()
    
    content = response_json['choices'][0]['message']['content']
    return content.strip()

# ============计算欧几里得距离===============
def calculate_distance(pos1, pos2):
    """计算两个三维位置之间的欧几里得距离"""
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 )

# =============读取 positions_and_poses.txt 文件，并与 positions 中的每个位置进行对比===============
def compare_positions_with_file(positions):
    # 如果 positions 是一个单独的位置，将其包装成一个列表
    if isinstance(positions, list) and all(isinstance(x, float) for x in positions):
        positions = [positions]
    file_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/positions_and_poses.txt"
    
    # 读取文件内容
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
    except FileNotFoundError:
        print(f"错误: 文件 {file_path} 未找到。")
        return None
    except Exception as e:
        print(f"读取文件时发生错误: {e}")
        return None
    
    closest_positions = []  # 存放每个 position 的最近位置
    
    # 遍历 positions 列表中的每个位置
    for idx, pos in enumerate(positions):
        closest_distance = float('inf')  # 初始时设置为正无穷大
        closest_line = None  # 存放与当前 position 最接近的行（时间戳、位置、姿态）
        
        # 遍历 positions_and_poses.txt 文件中的每一行
        for line in lines:
            parts = line.strip().split()
            if len(parts) < 7:
                continue  # 跳过不完整的行
            
            timestamp = parts[0]  # 时间戳
            position = [float(parts[1]), float(parts[2]), float(parts[3])]  # 位置 (x, y, z)
            orientation = [float(parts[4]), float(parts[5]), float(parts[6])]  # 姿态 (x, y, z, w)

            # 计算与当前 position 的欧几里得距离
            distance = calculate_distance(pos, position)

            # 如果当前距离小于之前的最小距离，更新最小距离和最近的行
            if distance < closest_distance:
                closest_distance = distance
                closest_line = (timestamp, position, orientation)
        
        # 保存每个位置对应的最近时间戳和位置
        closest_positions.append({
            "position": pos,
            "closest_timestamp": closest_line[0],
            "closest_position": closest_line[1],
            "closest_orientation": closest_line[2],
            "closest_distance": closest_distance
        })
    
    return closest_positions

# ============获取到记忆节点的位置的序号，用于route_plan实现规划路径==============
def get_object_node_from_message(end_object_positon):
    """
    通过调用 GPT API 获取到目标物体所对应的节点序号
    :param end_object_positon: 目标物体的位置
    :param model: 使用的模型（默认为 gpt-3.5-turbo-0125）
    :return: 提取出对应的节点
    """

    final_position = end_object_positon

    file_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/object_memory.txt"
    object_data = []
    
    # 读取 object_memory.txt 文件
    try:
        with open(file_path, 'r') as file:
            for line in file:
                try:
                    # 使用正则表达式匹配行内容
                    match = re.match(r'^(.+)\s+(\d+)\s+\[(.*?)\]\s+(\d+)$', line.strip())
                    if match:
                        object_name = match.group(1).strip()  # 物体名称，去除前后空格
                        node_id = match.group(2)     # 节点ID
                        position = match.group(3)    # 位置
                        attribute = match.group(4)   # 属性
                        
                        object_info = {
                            "object_name": object_name,
                            "node_id": node_id,
                            "position": [float(x) for x in position.split(', ')],
                            "attribute": int(attribute)
                        }
                        object_data.append(object_info)
                except ValueError as ve:
                    print(f"解析失败: {ve}")
                    continue  # 如果解析失败，跳过该行
    except FileNotFoundError:
        print(f"错误: 文件 {file_path} 未找到。")
        return None
    except Exception as e:
        print(f"读取文件时发生错误: {e}")
        return None

    if not object_data:
        print("错误: 未从 object_memory.txt 中读取到有效数据。")
        return None

    # API URL和授权信息
    api_key = "sk-V8Rbc18fdd54f64583d559271b56cb218cae95c3e6fBz6wg"
    url = "https://api.gptsapi.net/v1/chat/completions"
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }

    # 请求数据
    data = {
        "model": "gpt-3.5-turbo",
        "messages": [
            {"role": "user", "content": f"我会提供给你一个数组，数组中每个元素都包含了物体节点序号，物体节点位置和物体节点的属性，数组为:{object_data}，你唯一要做的就是提取出{final_position}中对应的node_id属性并返回,除此之外不用返回其他任何输出，需要注意的是{final_position}中的数据可能不完全和数组中的数据相等，只要相差不超过0.02，即可返回对应的node_id属性，属性的数值在原有的基础上加上4,返回格式为：\"node\":xx"}
        ]
    }

    # 发送POST请求
    response = requests.post(url, headers=headers, data=json.dumps(data))

    # 获取并返回 'content' 字段
    response_json = response.json()
    
    content = response_json['choices'][0]['message']['content']
    
    return content.strip()

def copy_file(source_path, destination_path):
    """
    将源文件的内容复制到目标文件中。
    :param source_path: 源文件路径
    :param destination_path: 目标文件路径
    """
    try:
        # 打开源文件并读取内容
        with open(source_path, "r", encoding="utf-8") as source_file:
            content = source_file.read()  # 读取全部内容
        
        # 将内容写入目标文件
        with open(destination_path, "w", encoding="utf-8") as destination_file:
            destination_file.write(content)
        
        print(f"文件内容已从 {source_path} 复制到 {destination_path}。")
    except FileNotFoundError:
        print(f"错误: 文件 {source_path} 未找到。")
    except Exception as e:
        print(f"复制文件时发生错误: {e}")


# =================== 主程序入口 ===================
if __name__ == "__main__":
    # 等待文件内容变为 "1" 才开启第二次导航
    wait_for_file_content("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/second_goal.txt")

    # 读取 end_object_name.txt 文件内容
    with open("/home/lm/Desktop/catkin_ws/keymap_ws/end_object_name.txt", "r", encoding="utf-8") as file:
        object_name = file.read().strip()

    # 获取物体的区域和节点信息
    area, node_id = get_area_and_node_for_object(object_name, "/home/lm/Desktop/catkin_ws/keymap_ws/topology_graph.graphml")
    print(f"物体原始信息：区域 - {area}, 节点ID - {node_id}")

    # 生成请求并保存响应
    generate_relocation_request(object_name, area, node_id)     # 再加上一次
    time.sleep(5)
    # 启动 ROS 2 节点
    ros2_ws_dir = "/home/lm/Desktop/catkin_ws/keymap_ws/"
    run_ros2_node(ros2_ws_dir, "ros2 run py_graph route_plan")
    print("第一次使用推理后的信息进行导航")


    with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/memory_point_record.txt", "r") as input_file:
        content = input_file.read().strip()  # 读取内容并去除空白字符
        if content != "1":
            # 等待用户按下回车键
            input("")
            run_ros2_node(ros2_ws_dir, "ros2 run ram_detect virtual_point_to_tracking")

    with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/memory_point_record.txt", "w") as output_file:
        output_file.write("")  # 写入空字符串以清空文件
    
    time.sleep(2)
    

    if(check_navigation_status("/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt") == True):
        print("第一次进行推理找到了物体")
        sys.exit()

    # 第二次进行位置推理
    generate_relocation_request(object_name, area, node_id, "second") 
    time.sleep(5)
    # 启动 ROS 2 节点
    ros2_ws_dir = "/home/lm/Desktop/catkin_ws/keymap_ws/"
    run_ros2_node(ros2_ws_dir, "ros2 run py_graph route_plan")
    print("第二次使用推理后的信息进行导航")

    with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/memory_point_record.txt", "r") as input_file:
        content = input_file.read().strip()  # 读取内容并去除空白字符
        if content != "1":
            # 等待用户按下回车键
            input("")
            run_ros2_node(ros2_ws_dir, "ros2 run ram_detect virtual_point_to_tracking")

    with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/memory_point_record.txt", "w") as output_file:
        output_file.write("")  # 写入空字符串以清空文件
    
    time.sleep(2)

    if(check_navigation_status("/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt") == True):
        print("第二次进行推理找到了物体")
        sys.exit()

    # 使用gpt提取出所有object_memory里面的指定物体的节点和位置
    
    memory_object_list = get_memory_object_name_from_message()  # 获取到的是position

    # 获取到的是所有对应的记忆节点对应的positions
    if memory_object_list != None and isinstance(memory_object_list, str):
        try:
            # 将返回的字符串解析为 JSON 数组
            object_name_list = json.loads(memory_object_list)
            
            # 获取 position 属性
            positions = [position for position in object_name_list]
            print("所有对应的记忆节点对应的positions:",positions)

            # 从整个拓扑图的轨迹中(position_and_poses.txt)获取到与记忆节点最近的那个节点 ---> 用来当作想要巡迹的点
            closest_positions = compare_positions_with_file(positions)
            
            if closest_positions:
                for item in closest_positions:
                    closest_position = [f"{x:.5f}" for x in item['position']]
                    print(f"Position: {item['position']} - 最近位置: {item['closest_position']} "
                        f"时间戳: {item['closest_timestamp']} 距离: {item['closest_distance']}")
                                
                    # 返回的数据 ---> node_id:88
                    result_node_number = get_object_node_from_message(closest_position)
                    print("result_node_number:",result_node_number)
                    # 将响应写入model_relocation.txt文件之后，可以再次调用route_plan节点,实现在get_end_id_from_file_or_input函数中获取节点序号，生成巡迹序列
                    output_filename = "model_relocation.txt"
                    with open(output_filename, "w", encoding="utf-8") as f:
                        f.write(result_node_number)
                    print(f"响应内容--{result_node_number}--已保存至 {output_filename}")

                    with open("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/memory_point_record.txt", "r") as file:
                        content = file.read().strip()  # 读取内容并去除空白字符
                    
                    if content == "1":  # 如果记忆节点的序号小于小车当前的序号，说明该记忆节点有问题，不需要路径规划和遍历
                        with open("model_relocation.txt", "w") as output_file:
                            output_file.write("")  # 写入空字符串以清空文件
                    else:
                        # 再次进行路径规划和导航
                        ros2_ws_dir = "/home/lm/Desktop/catkin_ws/keymap_ws/"
                        run_ros2_node(ros2_ws_dir, "ros2 run py_graph route_plan")
                        print("第二次使用记忆节点进行路径规划")
                        time.sleep(2)
                        # 等待用户按下回车键
                        input("")
                        run_ros2_node(ros2_ws_dir, "ros2 run ram_detect virtual_point_to_tracking")
                    break       # 在这里break的意思应该是只获取到最近的一个节点
        except json.JSONDecodeError as e:
            print(f"JSON解析失败: {e}")
    else:
        print("没有对应的记忆节点")
    


    # 检查导航状态
    # 如果为 False，说明二次导航未找到目标物
    if(check_navigation_status("/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt") == False):  
        print("二次导航未找到目标物，决定遍历所有拓扑图节点")
        orginal_derection_file = "/home/lm/Desktop/catkin_ws/keymap_ws/positions_and_poses.txt"
        adverse_derection_file = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/route_trajectory.txt"
        flag_reverse_path_file = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/flag_reverse.txt"
        if  check_navigation_status(flag_reverse_path_file) == False:
            print("调头")
            reverse_file_content(orginal_derection_file, adverse_derection_file)    # 获取到反转后的巡迹路线
        else:
            # 源文件和目标文件路径
            source_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/positions_and_poses.txt"
            destination_path = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/route_trajectory.txt"
            # 调用函数复制文件
            copy_file(source_path, destination_path)

        # 等待用户按下回车键
        input("")
        run_ros2_node(ros2_ws_dir, "ros2 run ram_detect virtual_point_to_tracking")
        print("第三次遍历所有路径进行物体寻找")