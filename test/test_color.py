import cv2
import numpy as np

# 定义全局变量，用于存储点击的颜色
clicked_color = None

# 鼠标回调函数，获取点击点的颜色
def get_color(event, x, y, flags, param):
    global clicked_color
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_color = frame[y, x]
        print(f"Clicked color (BGR): {clicked_color}")
        hsv_color = cv2.cvtColor(np.uint8([[clicked_color]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f"Clicked color (HSV): {hsv_color}")

# 打开摄像头
cap = cv2.VideoCapture(1)

# 创建窗口并设置鼠标回调函数
cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", get_color)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 如果已经获取到颜色，进行颜色分割
    if clicked_color is not None:
        # 将点击的颜色转换为HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_color = cv2.cvtColor(np.uint8([[clicked_color]]), cv2.COLOR_BGR2HSV)[0][0]

        # 定义HSV颜色的范围
        lower_bound = np.array([hsv_color[0] - 10, 50, 50])
        upper_bound = np.array([hsv_color[0] + 10, 255, 255])

        # 创建掩码
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # 对掩码进行形态学操作，去除噪点
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 找到轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到面积最大的轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # 画出矩形框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 显示画面
    cv2.imshow("Frame", frame)

    # 按下 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()
