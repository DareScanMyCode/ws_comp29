import os
import cv2
import sys
import argparse
import torch
from UavOnboard import UAVOnBoard
import apriltag


# add path
realpath = os.path.abspath(__file__)
_sep = os.path.sep
realpath = realpath.split(_sep)
#sys.path.append(os.path.join(realpath[0]+_sep, *realpath[1:realpath.index('rknn_model_zoo-main')+1]))

from py_utils.coco_utils import COCO_test_helper
import numpy as np
from rknnlite.api import RKNNLite


OBJ_THRESH = 0.25
NMS_THRESH = 0.45

# The follew two param is for map test
# OBJ_THRESH = 0.001
# NMS_THRESH = 0.65

IMG_SIZE = (640, 640)  # (width, height), such as (1280, 736)


VIDEO_PATH = '/dev/video10'
# VIDEO_PATH = 'rtsp://192.168.144.25:8554/main.264'
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
CLASSES = ("person", "bicycle", "car","motorbike ","aeroplane ","bus ","train","truck ","boat","traffic light",
           "fire hydrant","stop sign ","parking meter","bench","bird","cat","dog ","horse ","sheep","cow","elephant",
           "bear","zebra ","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
           "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife ",
           "spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza ","donut","cake","chair","sofa",
           "pottedplant","bed","diningtable","toilet ","tvmonitor","laptop	","mouse	","remote ","keyboard ","cell phone","microwave ",
           "oven ","toaster","sink","refrigerator ","book","clock","vase","scissors ","teddy bear ","hair drier", "toothbrush ")
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
    # Python ���� score_sum ���
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
        print("%s @ (%d %d %d %d) %.3f" % (CLASSES[cl], top, left, right, bottom, score))
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (top, left - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

def setup_model(args):
    model_path = args.model_path
    if model_path.endswith('.pt') or model_path.endswith('.torchscript'):
        platform = 'pytorch'
        from py_utils.pytorch_executor import Torch_model_container
        model = Torch_model_container(args.model_path)
    elif model_path.endswith('.rknn'):
        platform = 'rknn'
        from py_utils.rknn_executor import RKNN_model_container 
        model = RKNN_model_container(args.model_path, args.target, args.device_id)
    elif model_path.endswith('onnx'):
        platform = 'onnx'
        from py_utils.onnx_executor import ONNX_model_container
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process some integers.')
    # basic params
    parser.add_argument('--model_path', type=str, required= True, help='model path, could be .pt or .rknn file')
    parser.add_argument('--target', type=str, default='rk3566', help='target RKNPU platform')
    parser.add_argument('--device_id', type=str, default=None, help='device id')
    
    parser.add_argument('--img_show', action='store_true', default=False, help='draw the result and show')
    parser.add_argument('--img_save', action='store_true', default=False, help='save the result')

    # data params
    parser.add_argument('--anno_json', type=str, default='../../../datasets/COCO/annotations/instances_val2017.json', help='coco annotation path')
    # coco val folder: '../../../datasets/COCO//val2017'
    parser.add_argument('--img_folder', type=str, default='../model', help='img folder path')
    parser.add_argument('--coco_map_test', action='store_true', help='enable coco map test')

    args = parser.parse_args()

    # init model
    model, platform = setup_model(args)

    co_helper = COCO_test_helper(enable_letter_box=True)

    # run test
    cap = cv2.VideoCapture("/dev/video0")

    # �ж����ĸ����ޣ�ͼ�����Ͻ�Ϊԭ�� ��ΪX������ ��ΪY������
    def JudgeQuadrant(axis_x,axis_y):
        distanceX=axis_x-0.5
        distanceY=axis_y-0.5
        if (distanceX<0 and distanceY<0):
            return 2
        if (distanceX>0 and distanceY<0):
            return 1
        if (distanceX<0 and distanceY>0):
            return 3
        if (distanceX>0 and distanceY>0):
            return 4
    
    global print_time
    print_time=0
     # time delay print
    def print_info(info):
        global print_time
        print_time+=1
        if print_time % 10 == 0:
            print(info)
            # time.sleep(0.5)

    # cap.set(4, 640)
    import time
    Uav = UAVOnBoard()
    Uav.start_send_vel_cmd_t()
    while True:
        try:

            ret, img_src = cap.read()
            print(img_src.shape)    
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

            print_info(input_data.shape)
            
            outputs = model.run([input_data])
            inference_time = time.time() - inference_begin
            print_info(f"inference time: {inference_time*1000:.2f}ms, fps: {1./inference_time}")
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
            
            # ���ͼ��
            ret, frame = cap.read()
            # ��ⰴ��
            k = cv2.waitKey(1)
            if k==27:
                 break
            print(frame.shape)        
            # ���apriltag
            at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray)
            for tag in tags:
                cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
                cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
                cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
                cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            # ��ʾ�����
            cv2.imshow('capture', frame)
            print(tags)
            if tags is None:
                if boxes is not None:
                    if len(boxes)!=0:
                        #find the highest score
                        for i in range(0, len(boxes)):
                            if scores[i] == max(scores):
                                
                                x_mid=(boxes[i, 0]+boxes[i, 2])/1280
                                y_mid=(boxes[i, 3])/640
                                dx=abs(x_mid-0.5)
                                dy=abs(y_mid-0.5)
                                if dx>0.05 or dy>0.05:    
                                    # ����

                                    Quadrant=JudgeQuadrant(x_mid, y_mid)
                                    
                                    kx=1
                                    ky=1
                                    
                                    if Quadrant==2:
                                        #Uav.uav_send_speed_FLU( ky*dy,  kx*dx, -0.1) UAV is FRD
                                        Uav.vel_set_frd.x =  ky*dy
                                        Uav.vel_set_frd.y = -kx*dx
                                        Uav.vel_set_frd.z =  0.0
                                        #time.sleep(0.1)
                                        #print("FLD")
                                        
                                    elif Quadrant==1:
                                        #Uav.uav_send_speed_FLU( ky*dy, -kx*dx, -0.1)
                                        Uav.vel_set_frd.x =  ky*dy
                                        Uav.vel_set_frd.y =  kx*dx
                                        Uav.vel_set_frd.z =  0.0
                                        #print("FRD")

                                    elif Quadrant==3:
                                        #Uav.uav_send_speed_FLU(-ky*dy,  kx*dx, -0.1)
                                        Uav.vel_set_frd.x =  -ky*dy
                                        Uav.vel_set_frd.y =  -kx*dx
                                        Uav.vel_set_frd.z =  0.0
                                        #print("BLD")

                                    elif Quadrant==4:         
                                        #Uav.uav_send_speed_FLU(-ky*dy, -kx*dx, -0.1)
                                        Uav.vel_set_frd.x =  -ky*dy
                                        Uav.vel_set_frd.y =  kx*dx
                                        Uav.vel_set_frd.z =  0.0
                                        #print("BRD")
                else:
                    Uav.vel_set_frd.x =  0.0
                    Uav.vel_set_frd.y =  0.0
                    Uav.vel_set_frd.z =  0.0                       
                # print_info("vx=")
                # print_info(Uav.vel_set_frd.x)
                # print_info("vy=")
                # print_info(Uav.vel_set_frd.y)
                # print_info("vz=")
                # print_info(Uav.vel_set_frd.z)
            # else:
            #     if len(tags)!=0:
            #             #find the highest score
            #             for i in range(0, len(tags)):
            #                 if tags.decision_margin[i] == max(tags.decision_margin):
                                
            #                     x_mid=tags.center[0]/640
            #                     y_mid=tags.center[1]/640
            #                     dx=abs(x_mid-0.5)
            #                     dy=abs(y_mid-0.5)
            #                     if dx>0.01 or dy>0.01:    
            #                         # ����

            #                         Quadrant=JudgeQuadrant(x_mid, y_mid)
                                    
            #                         kx=1
            #                         ky=1
                                    
            #                         if Quadrant==2:
            #                             #Uav.uav_send_speed_FLU( ky*dy,  kx*dx, -0.1) UAV is FRD
            #                             Uav.vel_set_frd.x = -ky*dy
            #                             Uav.vel_set_frd.y =  kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #time.sleep(0.1)
            #                             #print("FLD")
                                        
            #                         elif Quadrant==1:
            #                             #Uav.uav_send_speed_FLU( ky*dy, -kx*dx, -0.1)
            #                             Uav.vel_set_frd.x = -ky*dy
            #                             Uav.vel_set_frd.y = -kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #print("FRD")

            #                         elif Quadrant==3:
            #                             #Uav.uav_send_speed_FLU(-ky*dy,  kx*dx, -0.1)
            #                             Uav.vel_set_frd.x =  ky*dy
            #                             Uav.vel_set_frd.y =  kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #print("BLD")

            #                         elif Quadrant==4:         
            #                             #Uav.uav_send_speed_FLU(-ky*dy, -kx*dx, -0.1)
            #                             Uav.vel_set_frd.x =   ky*dy
            #                             Uav.vel_set_frd.y =  -kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #print("BRD")
            #     else:
            #         Uav.vel_set_frd.x =  0.0
            #         Uav.vel_set_frd.y =  0.0
            #         Uav.vel_set_frd.z =  0.0


            #apriltag 
            #cap = cv2.VideoCapture("/dev/video0") line245
            # at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag25h9'))

            # while(1):
            #     # ���ͼ��
            #     ret, frame = cap.read()
            #     # ��ⰴ��
            #     k = cv2.waitKey(1)
            #     if k==27:
            #         break
                    
            #     # ���apriltag
            #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #     tags = at_detector.detect(gray)
            #     for tag in tags:
            #         cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
            #         cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
            #         cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
            #         cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom
            #     # ��ʾ�����
            #     cv2.imshow('capture', frame)
            #     if tags is not None:
            #         if len(tags)!=0:
            #             #find the highest score
            #             for i in range(0, len(tags)):
            #                 if tags.decision_margin[i] == max(tags.decision_margin):
                                
            #                     x_mid=tags.center[0]/640
            #                     y_mid=tags.center[1]/640
            #                     dx=abs(x_mid-0.5)
            #                     dy=abs(y_mid-0.5)
            #                     if dx>0.05 or dy>0.05:    
            #                         # ����

            #                         Quadrant=JudgeQuadrant(x_mid, y_mid)
                                    
            #                         kx=1
            #                         ky=1
                                    
            #                         if Quadrant==2:
            #                             #Uav.uav_send_speed_FLU( ky*dy,  kx*dx, -0.1) UAV is FRD
            #                             Uav.vel_set_frd.x =  ky*dy
            #                             Uav.vel_set_frd.y = -kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #time.sleep(0.1)
            #                             #print("FLD")
                                        
            #                         elif Quadrant==1:
            #                             #Uav.uav_send_speed_FLU( ky*dy, -kx*dx, -0.1)
            #                             Uav.vel_set_frd.x =  ky*dy
            #                             Uav.vel_set_frd.y =  kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #print("FRD")

            #                         elif Quadrant==3:
            #                             #Uav.uav_send_speed_FLU(-ky*dy,  kx*dx, -0.1)
            #                             Uav.vel_set_frd.x =  -ky*dy
            #                             Uav.vel_set_frd.y =  -kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #print("BLD")

            #                         elif Quadrant==4:         
            #                             #Uav.uav_send_speed_FLU(-ky*dy, -kx*dx, -0.1)
            #                             Uav.vel_set_frd.x =  -ky*dy
            #                             Uav.vel_set_frd.y =  kx*dx
            #                             Uav.vel_set_frd.z =  0.0
            #                             #print("BRD")
            #     else:
            #         Uav.vel_set_frd.x =  0.0
            #         Uav.vel_set_frd.y =  0.0
            #         Uav.vel_set_frd.z =  0.0
            # cap.release()
            # cv2.destroyAllWindows()
        except KeyboardInterrupt:
            break
    #     # record maps
    #     if args.coco_map_test is True:
    #         if boxes is not None:
    #             for i in range(boxes.shape[0]):
    #                 co_helper.add_single_record(image_id = int(img_name.split('.')[0]),
    #                                             category_id = coco_id_list[int(classes[i])],
    #                                             bbox = boxes[i],
    #                                             score = round(scores[i], 5).astype(np.float)
    #                                             )

    # # calculate maps
    # if args.coco_map_test is True:
    #     pred_json = args.model_path.split('.')[-2]+ '_{}'.format(platform) +'.json'
    #     pred_json = pred_json.split('/')[-1]
    #     pred_json = os.path.join('./', pred_json)
    #     co_helper.export_to_json(pred_json)

    #     from py_utils.coco_utils import coco_eval_with_json
    #     coco_eval_with_json(args.anno_json, pred_json)

