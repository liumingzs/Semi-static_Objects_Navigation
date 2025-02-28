# 接收图像和深度信息，进行深度数据的聚类，预测物体掩码，裁剪图像并保存，然后用CLIP模型计算图像与预定义文本标签的相似性，并生成最终的处理结果。
from flask import Flask, request, jsonify
import numpy as np
import torch
import cv2
import os
from sklearn.cluster import DBSCAN
from mobile_sam import sam_model_registry, SamPredictor
import clip
from PIL import Image
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET
import math
import time
from xml.dom import minidom
import re


# 初始化序号，从1开始
node_index = 1
image_filename = None

# 初始化 Flask 应用
app = Flask(__name__)

# 模型加载（只执行一次）
sam_checkpoint = "/home/lm/Desktop/catkin_ws/MobileSAM/weights/mobile_sam.pt"
model_type = "vit_t"
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)
# 定义对象标签列表
object_labels = ["vase", "blue garbage bin","mug beer","biscuit box","monocular camera","labtop macbook","monitor"]
text = clip.tokenize(object_labels).to(device)

print("加载 MobileSAM 模型...")
sam = sam_model_registry[model_type](checkpoint=sam_checkpoint).to(device).eval()
predictor = SamPredictor(sam)

@app.route('/predict', methods=['POST'])
def predict_mask():
    global node_index,image_filename
    try:
        # 从 JSON 中解析数据
        data = request.get_json()

        image_path = data['image_path']
        print(image_path)
        depth_points = np.array(data['points'], dtype=object)

        # 分离深度和像素数据
        points = depth_points[:, :3]  # 深度数据 [x, y, z]
        pixels = depth_points[:, 3:]  # 像素坐标 [u, v]

        # 筛选深度小于 1.8 的点
        valid_indices = points[:, 0] <= 3.0
        points = points[valid_indices]
        pixels = pixels[valid_indices]

        # 使用 DBSCAN 聚类
        dbscan = DBSCAN(eps=0.2, min_samples=5)
        labels = dbscan.fit_predict(points[:, [0]])

        # 计算每个簇的平均深度并排序
        cluster_depths = {k: np.mean(points[labels == k][:, 0])
                          for k in np.unique(labels) if k != -1}
        sorted_clusters = sorted(cluster_depths.items(), key=lambda item: item[1])

        # 创建新标签数组
        new_labels = np.full_like(labels, -1)
        for new_label, (old_label, _) in enumerate(sorted_clusters):
            new_labels[labels == old_label] = new_label
        if len(sorted_clusters) < 2:
            return jsonify({"message": "Less than 2 clusters found."}), 400

        # 提取 Cluster 1 和 Cluster 2 的数据
        cluster_1_points = depth_points[new_labels == 0]
        cluster_2_points = depth_points[new_labels == 1]

        if len(cluster_1_points) < len(cluster_2_points) / 2:
            # 解嵌套，将 list([u, v]) 转换为普通的二维列表
            cluster_1_pixels = np.array([p[0] for p in cluster_1_points[:, 3:]])

            mid_index = len(cluster_1_pixels) // 2
            x, y = cluster_1_pixels[mid_index]  # 现在可以正确解包

            # 读取并处理图像
            image = cv2.imread(image_path)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            predictor.set_image(image)
            print("image_path,x,y:",image_path,x,y)
            # 执行掩码预测
            input_point = np.array([[x, y]])
            input_label = np.array([1])
            masks, scores, _ = predictor.predict(
                point_coords=input_point,
                point_labels=input_label,
                multimask_output=True,
            )
            best_mask = masks[np.argmax(scores)]

            # 保存掩码
            binary_mask = (best_mask > 0).astype(int)
            np.savetxt("/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/mask.txt", binary_mask, fmt='%d')

            # 获取掩码边界框
            mask_nonzero = np.nonzero(binary_mask)
            min_y, max_y = np.min(mask_nonzero[0]), np.max(mask_nonzero[0])
            min_x, max_x = np.min(mask_nonzero[1]), np.max(mask_nonzero[1])

            # 扩大边界框
            margin = 50
            min_x = max(min_x - margin, 0)
            max_x = min(max_x + margin, image.shape[1])
            min_y = max(min_y - margin, 0)
            max_y = min(max_y + margin, image.shape[0])

            # 裁剪图像
            cropped_image = image[min_y:max_y, min_x:max_x]

            # 创建保存路径
            output_dir = '/home/lm/Desktop/catkin_ws/keymap_ws/src/ram_detect/src/3d_bounding_color_retangle_second/'
            os.makedirs(output_dir, exist_ok=True)
            image_filename = f"object_{node_index:06d}.png"
            # 生成图片名，按序号递增
            output_image_path = os.path.join(output_dir, f"object_{node_index:06d}.png")

            # 保存裁剪后的图像
            cv2.imwrite(output_image_path, cv2.cvtColor(cropped_image, cv2.COLOR_RGB2BGR))
            
            image = preprocess(Image.open(output_image_path)).unsqueeze(0).to(device)

            with open("/home/lm/Desktop/catkin_ws/MobileSAM/flag.txt", "w") as f:
                f.write("1")

            # 增加序号
            node_index += 1

            with torch.no_grad():
                # 获取图像和文本的特征
                image_features = model.encode_image(image)
                text_features = model.encode_text(text)
                
                # 计算图像和文本的logits
                logits_per_image, logits_per_text = model(image, text)
                
                # 计算每个文本标签的概率
                probs = logits_per_image.softmax(dim=-1).cpu().numpy()
            
            # 找到最大概率的索引
            max_prob_index = np.argmax(probs)

            # 获取最大概率对应的文本
            max_prob_text = object_labels[max_prob_index]
            
            # 获取最大概率
            max_prob = probs[0][max_prob_index]

            # 读取 end_name.txt 文件内容
            end_name_file = "/home/lm/Desktop/catkin_ws/keymap_ws/end_object_name.txt"
            with open(end_name_file, "r") as f:
                end_name_content = f.read().strip()
            print(f"max_prob: {max_prob} ,max_prob_text: {max_prob_text}")
            # 如果最大概率大于0.85，并且和终点物体描述相同，则查找成功，结束了。如果end_flag.txt里为空或者0，则说明没找到
            if max_prob > 0.800 and max_prob_text == end_name_content:

                end_flag_file = "/home/lm/Desktop/catkin_ws/keymap_ws/end_flag.txt"
                with open(end_flag_file, "w") as file:
                    file.write("1")  # 填入1 表示已经完成查找

                # 获取对应的深度值和 3D 空间坐标
                x_depth, y_depth, z_depth = cluster_1_points[mid_index, :3]  # 提取深度和 3D 坐标
                print(f"物体对应的图像坐标系下的坐标为: ({x_depth}, {y_depth}, {z_depth})")
                object_position = [round(x_depth, 5), -1.0 * round(y_depth, 5), round(z_depth, 5)]

                time.sleep(1)
                with open('/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/end_car_position.txt', 'r') as f:
                    line = f.readline().strip()  # 读取文件中的一行
                    data = line.split()  # 通过空格分割数据

                    # 提取相机位置和四元数姿态，并保留五位小数
                    camera_position = [round(float(data[0]), 5), round(float(data[1]), 5), round(float(data[2]), 5)]  # 位置 [x, y, z]
                    camera_orientation = [
                        round(float(data[3]), 5), 
                        round(float(data[4]), 5), 
                        round(float(data[5]), 5), 
                        round(float(data[6]), 5)] 

                # 获取到了物体的世界坐标系
                world_position = transform_to_world(camera_position, camera_orientation, object_position)
                print(world_position)
                graphml_file = "/home/lm/Desktop/catkin_ws/keymap_ws/topology_graph.graphml"
                update_closest_node_with_camera_position(camera_position, graphml_file, end_name_content)  # 为找到的节点添加记忆节点

                whether_to_update_graph_path = '/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/second_goal.txt'
                if( read_and_check_file(whether_to_update_graph_path) ):
                    update_and_reconnect_topology(end_name_file, world_position, graphml_file)  # 这是找到半动态物体后更新拓扑图的方法
                
                fix_graphml_format(graphml_file)    # 更改拓扑图文件格式
                return jsonify({"message": "Find end object successfully."}), 200
            else:return jsonify({"message": "max_prob and max_prob_test no match "}), 400

        else:
            with open("/home/lm/Desktop/catkin_ws/MobileSAM/flag.txt", "w") as f:
                f.write("0")
            return jsonify({"message": "Can't find the object from the depth value."}), 400

    except Exception as e:
        return jsonify({"error": str(e)}), 500

def read_and_check_file(file_path):
    try:
        # 打开文件并读取内容
        with open(file_path, 'r') as file:
            content = file.read().strip()
        
        # 判断内容是否为 '1'
        if content == '1':
            print("文件内容为1，执行后续操作...")
            # 在这里添加你的后续操作逻辑
            return True
        else:
            print(f"文件内容为: {content}，未执行后续操作。")
            return False
    
    except FileNotFoundError:
        print(f"错误：文件 {file_path} 未找到。")
    except Exception as e:
        print(f"发生错误：{e}")


def fix_graphml_format(graphml_file):
    """
    修复 GraphML 文件：
    1. 将 <graphml 替换为 <graphml xmlns="http://graphml.graphdrawing.org/xmlns"
    2. 将 <data key="d3" /> 替换为 <data key="d3"></data>
    
    参数:
        graphml_file (str): GraphML 文件路径
    """
    with open(graphml_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 替换 <graphml 添加 xmlns 属性 (只替换第一个匹配项)
    content = re.sub(
        r'<graphml\b',
        '<graphml xmlns="http://graphml.graphdrawing.org/xmlns"',
        content,
        count=1
    )
    
    # 替换 <data key="d3" /> 为 <data key="d3"></data>
    content = re.sub(
        r'<data key="d3"\s*/>',
        '<data key="d3"></data>',
        content
    )
    
    with open(graphml_file, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"GraphML 文件已成功修复：{graphml_file}")

def remove_namespace(xml_root):
    """移除XML中的所有命名空间前缀"""
    for elem in xml_root.iter():
        if '}' in elem.tag:
            elem.tag = elem.tag.split('}', 1)[1]  # 移除命名空间
    return xml_root

def update_closest_node_with_camera_position(camera_position, graphml_file, object_name):
    """
    在 GraphML 文件中找到与 camera_position 最近的节点，并在 <data key='d3'> 中写入 object_name对应的键值。
    同时将最近节点的信息保存到 object_memory.txt 文件中。

    参数:
        camera_position (list): 摄像头的位置 [x, y, z]。
        graphml_file (str): 拓扑图文件的路径。
        object_name (str): 物体名称，用于映射到键值。
    """

    # 定义 object_name 到 <data key='d3'> 的映射关系
    object_mapping = {
        'vase': '1',
        'blue garbage bin': '2',
        'biscuit box': '3',
        'coca cola can': '4'
    }
    
    # 获取对应的数值
    d3_value = object_mapping.get(object_name)
    if d3_value is None:
        print(f"未定义的 object_name: {object_name}，无法更新 <data key='d3'>。")
        return

    # 解析 GraphML 文件
    tree = ET.parse(graphml_file)
    root = tree.getroot()
    
    # 移除命名空间
    root = remove_namespace(root)
    
    closest_node_id = None
    closest_node_position = None
    min_distance = float('inf')
    
    # 遍历所有节点，计算与 camera_position 的距离
    for node in root.findall('.//node'):
        position_element = node.find("./data[@key='d0']")
        if position_element is not None:
            position_text = position_element.text.strip('()').split(',')
            try:
                node_position = [float(coord) for coord in position_text]
                distance = math.sqrt(
                    (node_position[0] - camera_position[0]) ** 2 +
                    (node_position[1] - camera_position[1]) ** 2
                )
                if distance < min_distance:
                    min_distance = distance
                    closest_node_id = node.get('id')
                    closest_node_position = node_position
            except ValueError:
                print(f"节点 {node.get('id')} 的位置数据格式不正确，跳过该节点。")
    
    if closest_node_id is None:
        print("未找到有效的节点，无法更新。")
        return
    
    # 更新最近节点的 <data key='d3'> 标签
    for node in root.findall('.//node'):
        if node.get('id') == closest_node_id:
            data_element = node.find("./data[@key='d3']")
            if data_element is not None:
                data_element.text = d3_value
                print(f"最近的节点ID: {closest_node_id}，已将 <data key='d3'> 更新为{d3_value}。")
            if data_element is  None:
                data_element.text = ''  

    # 保存最近节点信息到 object_memory.txt
    memory_file = "/home/lm/Desktop/catkin_ws/keymap_ws/txt_file/object_memory.txt"
    try:
        with open(memory_file, 'a') as file:
            node_position_str = f"[{closest_node_position[0]}, {closest_node_position[1]}, {closest_node_position[2]}]" if closest_node_position else "(N/A)"
            file.write(f"{object_name} {closest_node_id} {node_position_str} {d3_value}\n")
        print(f"最近节点信息已保存到 {memory_file}。")
    except Exception as e:
        print(f"保存到 {memory_file} 时发生错误: {e}")

    # 保存修改后的 GraphML 文件
    tree = ET.ElementTree(root)
    tree.write(graphml_file, encoding="utf-8", xml_declaration=True)
    print(f"GraphML 文件已成功更新并保存: {graphml_file}")

def update_and_reconnect_topology(end_name_file, world_position, graphml_file, distance_threshold=0.7):
    # 读取 end_name_content
    with open(end_name_file, "r") as f:
        end_name_content = f.read().strip()

    print(f"正在更新GraphML文件，目标节点数据: {end_name_content}")

    # 解析 GraphML 文件
    tree = ET.parse(graphml_file)
    root = tree.getroot()
    
    # 移除命名空间
    root = remove_namespace(root)
    
    # 初始化存储信息
    target_node_id = None
    node_positions = {}  # 保存所有节点ID和对应的3D位置

    # Step 1: 更新目标节点的坐标
    for node in root.findall('.//node'):
        data_element = node.find("./data[@key='d4']")
        if data_element is not None and data_element.text == end_name_content:
            target_node_id = node.get('id')
            print(f"找到目标节点: {target_node_id}，更新坐标...")

            # 更新 key="d0" 数据为世界坐标
            position_element = node.find("./data[@key='d0']")
            if position_element is not None:
                new_position = f"({world_position[0]:.5f},{world_position[1]:.5f},{world_position[2]:.5f})"
                position_element.text = new_position
                print(f"节点 {target_node_id} 坐标已更新为: {new_position}")


            # 更新或创建 key="d1" 数据为 image_filename
            image_data_element = node.find("./data[@key='d1']")
            if image_data_element is not None:
                image_data_element.text = image_filename
            else:
                # 如果<d1>不存在，则创建
                ET.SubElement(node, 'data', key="d1").text = image_filename
            print(f"节点 {target_node_id} 的图片路径已更新为: {image_filename}")
            break

    if not target_node_id:
        print("未找到目标节点，无法进行更新和重连。")
        return

    # Step 2: 删除与目标节点相连的边
    edges_to_remove = []
    for edge in root.findall('.//edge'):
        source = edge.get('source')
        target = edge.get('target')
        if source == target_node_id or target == target_node_id:
            edges_to_remove.append(edge)

    for edge in edges_to_remove:
        root.find('.//graph').remove(edge)
    print(f"已删除与目标节点 {target_node_id}")

    # Step 3: 提取所有节点的位置
    for node in root.findall('.//node'):
        position_element = node.find("./data[@key='d0']")
        if position_element is not None:
            position = position_element.text.strip('()').split(',')
            position = [float(coord) for coord in position]
            node_positions[node.get('id')] = position

    # Step 4: 重新计算距离，添加符合条件的边
    new_edges = []
    for node_id, position in node_positions.items():
        if node_id != target_node_id:  # 不计算自身
            distance = calculate_distance(world_position, position)
            if distance < distance_threshold:
                edge_id = f"e_{target_node_id}_{node_id}"
                new_edges.append((edge_id, target_node_id, node_id, distance))
    # 注册命名空间
    namespace = "http://graphml.graphdrawing.org/xmlns"
    ET.register_namespace("", namespace)
    # 在GraphML中添加新边并添加<data key="d5">
    graph_element = root.find('.//graph')

    for _, source, target, distance in new_edges:  # 去掉 edge_id
        # 添加 <edge> 元素
        edge_element = ET.SubElement(graph_element, 'edge', source=source, target=target)
        edge_element.tail = "\n"  # 强制换行

        # 添加 <data key="d5"> 标签
        data_element = ET.SubElement(edge_element, 'data', key="d5")
        data_element.text = f"{distance:.17f}"  # 高精度表示距离
        data_element.tail = "\n"  # 确保 <data> 后换行

        print(f"新边已添加到拓扑图，源节点: {source}, 目标节点: {target}, 距离: {distance:.17f}")

    # Step 5: 保存更新后的GraphML文件，直接写入文件，保留结构
    tree = ET.ElementTree(root)
    tree.write(graphml_file, encoding="utf-8", xml_declaration=True)
    
    print(f"GraphML 文件已成功更新并保存: {graphml_file}")

def calculate_distance(pos1, pos2):
    """计算两个3D坐标之间的欧几里得距离"""
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def transform_to_world(camera_position, camera_orientation, object_position):
    """
    将物体的相机坐标转换到世界坐标系。

    参数:
        camera_position: list[float] (长度为 3)，相机的世界坐标 [x, y, z]。
        camera_orientation: list[float] (长度为 4)，相机的旋转四元数 [x, y, z, w]。
        object_position: list[float] (长度为 3)，物体在相机坐标系中的位置 [x, y, z]。

    返回:
        list[float]: 物体在世界坐标系中的位置 [x, y, z]。
    """
    # 相机的位置
    cam_pos = np.array(camera_position)

    # 相机的旋转（四元数）
    cam_ori = R.from_quat([camera_orientation[0], camera_orientation[1], camera_orientation[2], camera_orientation[3]])

    # 物体在相机坐标系中的位置，只考虑水平和垂直方向（z 置为 0）
    obj_pos = np.array([object_position[0], object_position[1], 0.0])

    # 转换到世界坐标系
    obj_pos_world = cam_ori.apply(obj_pos) + cam_pos

    return obj_pos_world.tolist()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
