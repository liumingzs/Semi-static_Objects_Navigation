import cv2

# 在一张图片的中心绘制一个实心圆后生成一个新的图片

# 读取原始图片
image = cv2.imread('/home/lm/Desktop/catkin_ws/keymap_ws/color_image.png')

# 获取图片的高度和宽度
height, width, _ = image.shape

# 计算圆的中心
center = (298, 178)

# 定义圆的半径和颜色（蓝色，BGR格式）
radius = 10
color = (255, 0, 0)  # 蓝色

# 在图片上绘制实心圆
cv2.circle(image, center, radius, color, -1)  # -1 表示填充圆

# 保存新的图片
cv2.imwrite('cciimage_with_circle.png', image)
