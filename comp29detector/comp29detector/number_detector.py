import os
import cv2
import sys
import argparse
import time
import numpy as np

import torch

realpath = os.path.abspath(__file__)
_sep = os.path.sep
realpath = realpath.split(_sep)

from .py_utils.coco_utils import COCO_test_helper

from rknnlite.api import RKNNLite

import rclpy  
from rclpy.node import Node  
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Header, Int64
from comp29msg.msg import DetectionResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
import rclpy.time
OBJ_THRESH = 0.25
NMS_THRESH = 0.45

IMG_SIZE = (640, 640)  # (width, height), such as (1280, 736)

VIDEO_PATH = '/dev/video0'
SAVE_PATH = '~/Desktop/mac_v/videos/video.mp4'

QUANTIZE_ON = False

BOX_THRESH = 0.5
NMS_THRESH = 0.6
IMG_SIZE = 320
IMG_SIZE = 640
IMG_SIZE = 224
SHOW_IMG_SIZE_X = 640
SHOW_IMG_SIZE_Y = 480
IMG_SIZE = [640, 640]

CLASSES = ("111", "222", "333")
coco_id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 31, 32, 33, 34,
         35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
         64, 65, 67, 70, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 84, 85, 86, 87, 88, 89, 90]


def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold.
    """
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score* box_confidences >= OBJ_THRESH)
    scores = (class_max_score* box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores

def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.
    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def dfl(position):
    # Distribution Focal Loss (DFL)
    x = torch.tensor(position)
    n,c,h,w = x.shape
    p_num = 4
    mc = c//p_num
    y = x.reshape(n,p_num,mc,h,w)
    y = y.softmax(2)
    acc_metrix = torch.tensor(range(mc)).float().reshape(1,1,mc,1,1)
    y = (y*acc_metrix).sum(2)
    return y.numpy()

def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1]//grid_h, IMG_SIZE[0]//grid_w]).reshape(1,2,1,1)

    position = dfl(position)
    box_xy  = grid +0.5 -position[:,0:2,:,:]
    box_xy2 = grid +0.5 +position[:,2:4,:,:]
    xyxy = np.concatenate((box_xy*stride, box_xy2*stride), axis=1)

    return xyxy

def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    defualt_branch=3
    pair_per_branch = len(input_data)//defualt_branch
    # Python 忽略 score_sum 输出
    for i in range(defualt_branch):
        boxes.append(box_process(input_data[pair_per_branch*i]))
        classes_conf.append(input_data[pair_per_branch*i+1])
        scores.append(np.ones_like(input_data[pair_per_branch*i+1][:,:1,:,:], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0,2,3,1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # filter according to threshold
    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    # nms
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores


def draw(image, boxes, scores, classes):
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = [int(_b) for _b in box]
        #print("%s @ (%d %d %d %d) %.3f" % (CLASSES[cl], top, left, right, bottom, score))
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (top, left - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

def setup_model(args):
    model_path = args.model_path
    if model_path.endswith('.pt') or model_path.endswith('.torchscript'):
        platform = 'pytorch'
        from .py_utils.pytorch_executor import Torch_model_container
        model = Torch_model_container(args.model_path)
    elif model_path.endswith('.rknn'):
        platform = 'rknn'
        from .py_utils.rknn_executor import RKNN_model_container 
        model = RKNN_model_container(args.model_path, args.target, args.device_id)
    elif model_path.endswith('onnx'):
        platform = 'onnx'
        from .py_utils.onnx_executor import ONNX_model_container
        model = ONNX_model_container(args.model_path)
    else:
        assert False, "{} is not rknn/pytorch/onnx model".format(model_path)
    print('Model-{} is {} model, starting val'.format(model_path, platform))
    return model, platform

def img_check(path):
    img_type = ['.jpg', '.jpeg', '.png', '.bmp']
    for _type in img_type:
        if path.endswith(_type) or path.endswith(_type.upper()):
            return True
    return False


parser = argparse.ArgumentParser(description='Process some integers.')
# basic params
parser.add_argument('--model_path', type=str, default="/home/cat/ws_comp29/src/weights/best123.rknn", help='model path, could be .pt or .rknn file')
parser.add_argument('--target', type=str, default='rk3588', help='target RKNPU platform')
parser.add_argument('--device_id', type=str, default=None, help='device id')
parser.add_argument('--img_show', action='store_true', default=False, help='draw the result and show')
parser.add_argument('--img_save', action='store_true', default=True, help='save the result')
# parser.add_argument('--anno_json', type=str, default='../../../datasets/COCO/annotations/instances_val2017.json', help='coco annotation path')
# parser.add_argument('--img_folder', type=str, default='../model', help='img folder path')
# parser.add_argument('--coco_map_test', action='store_true', help='enable coco map test')
args = parser.parse_args()

# init model
model, platform = setup_model(args)

co_helper = COCO_test_helper(enable_letter_box=True)


class NumberDetector(Node):
    def __init__(self, video_path): #, save_path, model_path, target, device_id='rk3588', img_show=False):
        super().__init__('number_detector')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 设置可靠性为RELIABLE
            history=QoSHistoryPolicy.KEEP_LAST,         # 只保留最新的历史消息
            depth=1                                    # 历史消息的队列长度
            )
        self.publisher_ = self.create_publisher(DetectionResult, '/number_detected', 10)
        self.cap = cv2.VideoCapture(video_path)
        
    def run(self):
        while rclpy.ok():
            try:
                ret, img_src = self.cap.read()
                if img_src is None:
                    print("None frame")
                    time.sleep(1)
                    continue
                cv2.imshow("frame", img_src)
                cv2.waitKey(1)
                '''
                # using for test input dumped by C.demo
                img_src = np.fromfile('./input_b/demo_c_input_hwc_rgb.txt', dtype=np.uint8).reshape(640,640,3)
                img_src = cv2.cvtColor(img_src, cv2.COLOR_RGB2BGR)
                '''
                # Due to rga init with (0,0,0), we using pad_color (0,0,0) instead of (114, 114, 114)
                pad_color = (0,0,0)
                img = co_helper.letter_box(im= img_src.copy(), new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=(0,0,0))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                # preprocee if not rknn model
                if platform in ['pytorch', 'onnx']:
                    input_data = img.transpose((2,0,1))
                    input_data = input_data.reshape(1,*input_data.shape).astype(np.float32)
                    input_data = input_data/255.
                else:
                    input_data = img

                inference_begin = time.time()
                input_data = np.expand_dims(input_data, 0)
                # print(input_data.shape)
                
                outputs = model.run([input_data])
                inference_time = time.time() - inference_begin
                #print_info(f"inference time: {inference_time*1000:.2f}ms, fps: {1./inference_time}")
                if outputs is None:
                    continue
                boxes, classes, scores = post_process(outputs)

                
                if args.img_show:
                    # print('\n\nIMG: {}'.format(img_name))
                    img_p = img_src.copy()
                    
                    if boxes is not None:
                        draw(img_p, co_helper.get_real_box(boxes), scores, classes)

                    if args.img_show:
                        img_p = cv2.resize(img_p, (640, 640))
                        cv2.imshow("full post process result", img_p)
                        cv2.waitKey(1)
                
                #这之后是控制器，根据yolov8有没有检测到目标启动，识别文件的结果是boxes
                msg = DetectionResult()
                if boxes is not None :
                    # find the highest score
                    # print(boxes)
                    for i in range(0, len(boxes)):
                        #可以写死决定目标是 1 或 2 或 3
                        print(classes)
                        # TODO 确认目标类型
                        # 不管啥类型都得发的！
                        if classes[i] == 1 :
                            if scores[i] == max(scores):
                                # define the landing position
                                # 停于目标下方（可以写死来规定哪家飞机停哪里）
                                x_mid=(boxes[i, 0]+boxes[i, 2])/1280
                                y_mid=(boxes[i, 3])/640
                                # 下方
                                #     x_mid=(boxes[i, 0]+boxes[i, 2])/1280
                                #     y_mid=(boxes[i, 3])/640
                                # 左上方
                                #     x_mid=(boxes[i, 0])/640
                                #     y_mid=(boxes[i, 1])/640
                                # 左下方
                                #     x_mid=(boxes[i, 2])/640
                                #     y_mid=(boxes[i, 1])/640
                                msg.position = Point(x=float(x_mid), y=float(y_mid), z=0.0)
                                msg.object_name = Int64()
                                msg.object_name.data = 2
                                # TODO header
                                # msg.header = Header(stamp=rclpy.time.Time())
                                self.publisher_.publish(msg)
            except KeyboardInterrupt:
                break


def main(args=None):
    # ROS init
    rclpy.init()
    node = NumberDetector(VIDEO_PATH)
    node.run()
 


