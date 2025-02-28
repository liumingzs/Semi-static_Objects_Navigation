import numpy as np
import sys
import time

def load_and_convert_to_2d(depth_path):
    """加载 .npy 文件并转换为二维数组"""
    # 加载 .npy 文件
    depth_data = np.load(depth_path)
    
    # 检查数据是否为二维
    if depth_data.ndim == 2:
        return depth_data
    else:
        raise ValueError("The provided .npy file is not a 2D array.")


def save_2d_to_text(depth_data, output_path):
    """将整个二维数组的数据保存到文本文件，每行一个数据"""
    with open(output_path, 'w') as file:
        # 遍历二维数组的每个元素并写入到文件
        for row in depth_data:
            for value in row:
                file.write(f"{value:.3f}\n")  # 每个数据保存为一行

    # print(f"All depth data has been saved to: {output_path}")


if __name__ == "__main__":
    # 从命令行参数获取文件路径
    depth_path = sys.argv[1]
    output_path = "/home/lm/Desktop/catkin_ws/keymap_ws/npy_data.txt"
    
    try:
        depth_2d = load_and_convert_to_2d(depth_path)
        # 打印二维数组的形状，方便调试
        # print(f"Depth data shape: {depth_2d.shape}")
        
        # 将整个二维数组保存为文本文件
        save_2d_to_text(depth_2d, output_path)
        
    except Exception as e:
        print(f"An error occurred: {e}")
