import sys
import numpy as np
import requests

def main():
    # 从命令行获取图像路径
    if len(sys.argv) < 2:
        print("Error: No image path provided.")
        sys.exit(1)

    color_image_path = sys.argv[1]

    # 从标准输入读取点数据
    data = []
    for line in sys.stdin:
        line = line.strip()
        if line:
            point = eval(line)
            data.append(point)

    # 转换为 NumPy 数组
    data = np.array(data, dtype=object)


    # 准备发送给 Flask 的数据
    payload = {
        "image_path": color_image_path,
        "points": data.tolist()  # 转换为可序列化的列表格式
    }

    # 发送 POST 请求
    response = requests.post("http://localhost:5000/predict", json=payload)

    # 检查响应状态码
    if response.status_code == 200:
        print("预测成功，掩码已保存到 mask.txt")
    else:
        print(f"预测失败，状态码: {response.status_code}")
        print("返回信息:", response.text)

if __name__ == "__main__":
    main()
