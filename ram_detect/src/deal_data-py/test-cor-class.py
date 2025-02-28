import sys
import numpy as np
from sklearn.cluster import DBSCAN
from fastsam import FastSAM, FastSAMPrompt
import warnings
warnings.filterwarnings("ignore")


'''
获取物体中心的像素点
'''
# 从标准输入读取数据
data = []
for line in sys.stdin:
    line = line.strip()
    if line:
        # 解析每一行
        point = eval(line)  # 确保数据来源可信
        data.append(point)

# 将列表转换为 numpy 数组
data = np.array(data, dtype=object)  # 使用 dtype=object 处理不规则数据

# 进一步处理数据，提取所需信息
# 将点和像素分开
points = np.array([d[:3] for d in data])  # 3D点
pixels = np.array([d[3] for d in data])   # 像素坐标

# 提取深度和横向距离
X = points[:, [0]]  # 选择深度作为特征

# 使用 DBSCAN 进行聚类
dbscan = DBSCAN(eps=0.2, min_samples=5)
labels = dbscan.fit_predict(X)

# 打印 Cluster 2 的数据点
cluster_2_points = data[labels == 2]
if cluster_2_points.size > 0:
    # 逐行输出每个点
    # for point in cluster_2_points:
    #     print(point)  

    # 获取 Cluster 2 中心位置的像素点坐标
    cluster_2_pixels = np.array([d[3] for d in cluster_2_points])
    mid_index = len(cluster_2_pixels) // 2  # 找到中间索引
    mid_pixel = cluster_2_pixels[mid_index]  # 获取中间位置的像素坐标
    x, y = mid_pixel
    # print("Middle pixel coordinate of Cluster 2:", mid_pixel)
    print("x:", x, "y:", y)

    # fastsam模型配置
    model = FastSAM('/home/lm/Desktop/catkin_ws/FastSAM/weights/FastSAM.pt')
    IMAGE_PATH = '/home/lm/Documents/EVF-SAM/assets/frame_color_000018.png'
    DEVICE = 'cuda'
    everything_results = model(IMAGE_PATH, device=DEVICE, retina_masks=True, imgsz=1024, conf=0.4, iou=0.9,)
    prompt_process = FastSAMPrompt(IMAGE_PATH, everything_results, device=DEVICE)
    # 生成掩码的mask.txt文件
    ann = prompt_process.point_prompt(points=[[522, 240]], pointlabel=[1])

else:
    print("Cluster 2 has no data points.")