import cv2
import numpy as np


def binarize_image(image_path):
    # 读取图片
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # 对图片进行二值化处理
    _, binary_image = cv2.threshold(image, 239, 255, cv2.THRESH_BINARY)

    # 寻找连通域
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_image, connectivity=8)

    # 找到最大连通域
    largest_component_label = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1

    # 创建一个与原始图像大小相同的掩码图像
    mask = np.zeros_like(image)

    # 将最大连通域的像素设置为白色
    mask[labels == largest_component_label] = 255

    # 获取最大连通域的边界框
    x, y, w, h = cv2.boundingRect(mask)

    # 提取最大连通域所在的ROI
    roi = image[y:y + h, x:x + w]

    # 显示原始图片和最大连通域
    cv2.imshow("Original Image", image)
    cv2.imshow("Binarized Image", binary_image)
    cv2.imshow("Largest Connected Component", mask)
    cv2.imshow("ROI", roi)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# 调用函数并传入图片文件路径
binarize_image("/home/yi/CLionProjects/robomaster/image_0.jpg")