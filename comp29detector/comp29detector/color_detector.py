import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import cv2
import numpy as np

class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker')
        
        # 发布器
        self.publisher = self.create_publisher(Vector3, '/color_offset', 10)

        # 定义全局变量，用于存储点击的颜色
        self.color_blue_hsv = [113, 255, 196]
        self.color_orange_hsv = [11, 190, 255]
        self.clicked_color = None

        # 打开摄像头
        self.cap = cv2.VideoCapture('/dev/video0')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # 创建窗口并设置鼠标回调函数
        cv2.namedWindow("Frame")
        # cv2.setMouseCallback("Frame", self.get_color)

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, self.frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
        hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        hsv_color = self.color_blue_hsv
        lower_bound = np.array([hsv_color[0] - 5, hsv_color[1] - 30, 50])
        upper_bound = np.array([hsv_color[0] + 5, hsv_color[1] + 30, 255])

        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            center_x = x + w // 2
            center_y = y + h // 2
            frame_width = self.frame.shape[1]
            frame_height = self.frame.shape[0]

            dx = center_x - frame_width // 2
            dy = center_y - frame_height // 2

            # 创建并发布 Vector3 消息
            msg = Vector3()
            msg.x = float(dx)
            msg.y = float(dy)
            msg.z = 0.0  # z 轴值为 0
            self.publisher.publish(msg)

        # cv2.imshow("Frame", self.frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.destroy_node()
        #     cv2.destroyAllWindows()
        #     self.cap.release()
        #     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
