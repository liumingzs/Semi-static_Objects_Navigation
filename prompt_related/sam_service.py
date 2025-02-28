# file: sam_service.py
from flask import Flask, request, jsonify
import numpy as np
import torch
import cv2
import os
from sklearn.cluster import DBSCAN
from mobile_sam import sam_model_registry, SamPredictor
import clip
from PIL import Image

# 初始化序号，从1开始
node_index = 1

# 用于记录上一次的文件内容
last_file_content = None

# 初始化 Flask 应用
app = Flask(__name__)

# 模型加载（只执行一次）
sam_checkpoint = "/home/lm/Desktop/catkin_ws/MobileSAM/weights/mobile_sam.pt"
model_type = "vit_t"
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)
# 定义对象标签列表
object_labels = ["vase", "blue garbage bin","coca cola can","biscuit box","monocular camera"]
text = clip.tokenize(object_labels).to(device)

print("加载 MobileSAM 模型...")
sam = sam_model_registry[model_type](checkpoint=sam_checkpoint).to(device).eval()
predictor = SamPredictor(sam)

@app.route('/predict', methods=['POST'])
def predict_mask():
    global node_index,last_file_content
    try:
        # 从 JSON 中解析数据
        data = request.get_json()

        image_path = data['image_path']
        depth_points = np.array(data['points'], dtype=object)

        # 分离深度和像素数据
        points = depth_points[:, :3]  # 深度数据 [x, y, z]
        pixels = depth_points[:, 3:]  # 像素坐标 [u, v]

        # 筛选深度小于 1.8 的点
        # valid_indices = points[:, 0] <= 1.8
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
        # print("len(sorted_clusters):",len(sorted_clusters))
        if len(sorted_clusters) < 2:
            return jsonify({"message": "Less than 2 clusters found."}), 400

        # 提取 Cluster 1 和 Cluster 2 的数据
        cluster_1_points = depth_points[new_labels == 0]
        # print("cluster_1_points:",len(cluster_1_points),cluster_1_points[0])
        cluster_2_points = depth_points[new_labels == 1]
        # print("cluster_2_points:",len(cluster_2_points),cluster_2_points[0])

        if len(cluster_1_points) < len(cluster_2_points) / 2:   # 是不是有可能这就是花瓶识别错误的原因
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
            output_dir = '/home/lm/Desktop/catkin_ws/keymap_ws/src/ram_detect/src/3d_bounding_color_retangle/'
            os.makedirs(output_dir, exist_ok=True)
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
            print("最大概率:",max_prob)


            # 如果最大概率大于0.85，则保存文本到文件
            # if max_prob > 0.85:
            if max_prob > 0.700:
                output_file_path = "/home/lm/Desktop/catkin_ws/MobileSAM/object_name.txt"
                
                with open(output_file_path, "r") as file:
                    file_content = file.read().strip()  # 去掉空格和换行符
                # 如果文件内容与当前要保存的文本相同，则清空文件内容
                if file_content == max_prob_text or last_file_content == max_prob_index:
                    last_file_content = max_prob_index
                    print(f"last_file_content:{last_file_content}")
                    with open(output_file_path, "w") as file:
                        file.write("")  # 清空文件内容
                    print(f"文件内容与当前文本相同，已清空文件 {output_file_path}")
                else:
                    # 否则将文本保存到文件
                    last_file_content = max_prob_index
                    with open(output_file_path, "w") as file:
                        file.write(max_prob_text)
                    print(f"文本 '{max_prob_text}' 已保存到 {output_file_path}")
                print("file_content:",file_content,"max_prob_text:",max_prob_text,"last_file_content:",last_file_content)

            else:
                print("无匹配文字")
                output_file_path = "/home/lm/Desktop/catkin_ws/MobileSAM/object_name.txt"
                print("max_prob_text:",max_prob_text,"last_file_content:",last_file_content)
                print(output_image_path)

                with open(output_file_path, "w") as file:
                    file.write("")  # 清空文件内容

            return jsonify({"message": "Mask generated successfully."}), 200
        else:
            with open("/home/lm/Desktop/catkin_ws/MobileSAM/flag.txt", "w") as f:
                f.write("0")

            return jsonify({"message": "Cluster 2 has no valid data points."}), 400


    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
