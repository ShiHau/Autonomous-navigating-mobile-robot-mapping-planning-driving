import os 
import time

import cmd_printer
import numpy as np
import torch
from args import args
from torchvision import transforms
import cv2



class Detector:
    def __init__(self, ckpt, use_gpu=False,):
        self.args = args
        # path for storing yolo model into
        model_path = os.path.join('network','fruits_model','best.pt')
        # force reload can be set to false if using same model and has been run before to speed up loading time
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)
        self.model.conf = 0.8 # confidence level (Only show results if detection confidence is more than 80%)
        self.use_gpu = use_gpu
        cmd_printer.divider(text="warning")
        print('This detector uses "RGB" input convention by default')
        print('If you are using Opencv, the image is likely to be in "BRG"!!!')
        cmd_printer.divider()
        self.colour_code = np.array([(220, 220, 220), (128, 0, 0), (155, 255, 70), (255, 85, 0), (255, 180, 0), (0, 128, 0)])
        # color of background, redapple, greenapple, orange, mango, capsicum

    def yolo_detection(self,img):

        # get results from model
        results = self.model(img)

        # seperates the results into easy to manipulate datatypes with pandas
        xmin = np.floor(np.array(results.pandas().xyxy[0]['xmin'])).astype(int)
        ymin = np.floor(np.array(results.pandas().xyxy[0]['ymin'])).astype(int)
        xmax = np.ceil(np.array(results.pandas().xyxy[0]['xmax'])).astype(int)
        ymax = np.ceil(np.array(results.pandas().xyxy[0]['ymax'])).astype(int)
        conf = np.array(results.pandas().xyxy[0]['confidence'])
        clas = np.array(results.pandas().xyxy[0]['class'])
        name = np.array(results.pandas().xyxy[0]['name'])

        num_obj = len(name)

        # create segmentation output
        height, width, channel = img.shape

        prediction = np.zeros([height,width])

        # label prediction bounding boxes
        for i in range(num_obj):
            p1 = (xmin[i].astype(int),ymin[i].astype(int))
            p4 = (xmax[i].astype(int),ymax[i].astype(int))
            fruit_type = int(clas[i]) + 1
                
            prediction = cv2.rectangle(prediction, p1, p4,(fruit_type,), -1)

        # draws bounding box output in GUI
        colour_map = self.visualise_output(prediction)

        return prediction, colour_map

    def visualise_output(self, nn_output):
        r = np.zeros_like(nn_output).astype(np.uint8)
        g = np.zeros_like(nn_output).astype(np.uint8)
        b = np.zeros_like(nn_output).astype(np.uint8)
        for class_idx in range(0, self.args.n_classes + 1):
            idx = nn_output == class_idx
            r[idx] = self.colour_code[class_idx, 0]
            g[idx] = self.colour_code[class_idx, 1]
            b[idx] = self.colour_code[class_idx, 2]
        colour_map = np.stack([r, g, b], axis=2)
        colour_map = cv2.resize(colour_map, (320, 240), cv2.INTER_NEAREST)
        w, h = 10, 10
        pt = (10, 160)
        pad = 5
        labels = ['redapple', 'greenapple', 'orange', 'mango', 'capsicum']
        font = cv2.FONT_HERSHEY_SIMPLEX 
        for i in range(1, self.args.n_classes + 1):
            c = self.colour_code[i]
            colour_map = cv2.rectangle(colour_map, pt, (pt[0]+w, pt[1]+h),
                            (int(c[0]), int(c[1]), int(c[2])), thickness=-1)
            colour_map  = cv2.putText(colour_map, labels[i-1],
            (pt[0]+w+pad, pt[1]+h-1), font, 0.4, (0, 0, 0))
            pt = (pt[0], pt[1]+h+pad)
        return colour_map