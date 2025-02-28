import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import DBSCAN

# 读取数据
data = np.loadtxt('/home/lm/Desktop/catkin_ws/keymap_ws/3d_coordinates.txt')

# 分别提取深度和坐标
depth = data[:, 0]
x = data[:, 1]
y = data[:, 2]

# 使用 DBSCAN 进行聚类
X = np.column_stack((x, y, depth))  # 将坐标和深度合并为一个二维数组
dbscan = DBSCAN(eps=0.2, min_samples=5)
labels = dbscan.fit_predict(X)

# 打印类别数量
unique_labels = np.unique(labels)
num_clusters = len(unique_labels[unique_labels != -1])  # 排除噪声类别
print(f"Number of clusters (excluding noise): {num_clusters}")

# 打印每个簇中点的个数
cluster_counts = {}  # 用于存储每个簇的点数
cluster_points = {}  # 用于存储每个簇的点

for k in unique_labels:
    if k != -1:  # 排除噪声
        count = np.sum(labels == k)
        cluster_counts[k] = count  # 存储到字典中
        print(f"Cluster {k} has {count} points.")

# 计算每个簇的质心
centroids = {}
for k in cluster_points:
    centroids[k] = np.mean(cluster_points[k][:, :3], axis=0)  # 计算质心

# 计算质心到原点的距离并排序
distances = {k: np.linalg.norm(v) for k, v in centroids.items()}
sorted_clusters = sorted(distances.keys(), key=lambda k: distances[k])

# 根据距离命名簇
cluster_names = {sorted_clusters[i]: f'cluster_{i}_points' for i in range(len(sorted_clusters))}
clusters = {name: cluster_points[int(cluster_id)] for cluster_id, name in cluster_names.items()}

# 打印每个簇的数量
for cluster_id, count in cluster_counts.items():
    print(f"Cluster {cluster_id} has {count} points.")

# 创建三维散点图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 为每个簇分配不同的颜色
colors = plt.cm.viridis(np.linspace(0, 1, len(unique_labels)))

for k, color in zip(unique_labels, colors):
    if k == -1:
        # 噪声点
        color = 'k'  # 黑色表示噪声
    class_member_mask = (labels == k)

    # 绘制散点
    ax.scatter(x[class_member_mask], y[class_member_mask], depth[class_member_mask], color=color, label=f'Cluster {k}')

# 设置标签
ax.set_xlabel('X (横向距离)')
ax.set_ylabel('Y (纵向距离)')
ax.set_zlabel('Depth (深度)')

# 添加图例
ax.legend()

# 显示图形
plt.title('3D Coordinates Visualization with DBSCAN Clustering')
plt.show()
