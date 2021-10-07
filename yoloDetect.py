
import sys
sys.path.append('/home/user/Workspace/yolov4/darknet')

import os
import random
from threading import Thread
from queue import Queue
import cv2
import darknet
import numpy as np
from textLog import logText


class YoloDetect():

    def __init__(self, globalData):

        self.globalData = globalData
        self.gid = globalData.getConstInitGid()
        self.isLog = globalData.getConstIsLog()
        self.outputPath = globalData.getConstOutputPath()
        self.rgbFrameQueue = globalData.colorQueue
        self.depthFrameQueue = globalData.depthQueue
        self.detectionsQueue = globalData.detectionsQueue

        self.imageSize = (0, 0)
        self.drawingRgbQueue = Queue()
        self.darknetFrameQueue = Queue()
        self.detectionQueue = Queue()

        wd = os.getcwd()
        os.chdir('/home/user/Workspace/yolov4/darknet')

        # Pre-built model.
        self.network, self.class_names, self.class_colors = darknet.load_network('cfg/yolov4-tiny.cfg', 'cfg/coco.data', 'yolov4-tiny.weights', batch_size=1)

        # Self trained model.
        #self.network, self.class_names, self.class_colors = darknet.load_network('cfg/yolov4-tiny-obj.cfg', 'data/obj.data', 'yolov4-tiny-obj_final.weights', batch_size=1)

        os.chdir(wd)

        self.darknet_width = darknet.network_width(self.network)
        self.darknet_height = darknet.network_height(self.network)


    def getConstThresh(self, ):

        return 0.25


    def convert2relative(self, bbox):

        """
        YOLO format use relative coordinates for annotation
        """

        x, y, w, h  = bbox
        _height     = self.darknet_height
        _width      = self.darknet_width

        return x/_width, y/_height, w/_width, h/_height


    def convert2original(self, image, bbox):

        x, y, w, h = self.convert2relative(bbox)

        image_h, image_w, __ = image.shape

        orig_x       = int(x * image_w)
        orig_y       = int(y * image_h)
        orig_width   = int(w * image_w)
        orig_height  = int(h * image_h)

        bbox_converted = (orig_x, orig_y, orig_width, orig_height)

        return bbox_converted


    def detections_adjust(self, detections: list) -> list:

        detections_adjust = []

        for detection in detections:
            b = self.convert2original(np.zeros(self.imageSize), detection[2])

            bounding = (int(b[0] - b[2] / 2), int(b[1] - b[3] / 2), b[2], b[3])
            
            detections_adjust.append({
                'name': detection[0],
                'assurance': detection[1],
                'bounding': bounding
            })

        return detections_adjust


    def convert_rgb_darknetImage(self) -> None:

        id = self.gid

        while not self.globalData.isTerminated:
            if self.rgbFrameQueue.qsize() > 0:
                rgbFrame = self.rgbFrameQueue.get()
                self.imageSize = rgbFrame.shape

                if self.isLog:
                    self.drawingRgbQueue.put(rgbFrame)

                frame_resized = cv2.resize(rgbFrame, (self.darknet_width, self.darknet_height), interpolation=cv2.INTER_LINEAR)
                darknetImage = darknet.make_image(self.darknet_width, self.darknet_height, 3)
                darknet.copy_image_from_bytes(darknetImage, frame_resized.tobytes())

                self.darknetFrameQueue.put_nowait(darknetImage)

                id += 1


    def inference(self) -> None:

        id = self.gid

        while not self.globalData.isTerminated:
            if self.darknetFrameQueue.qsize() > 0:
                darknet_image = self.darknetFrameQueue.get()
                detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=self.getConstThresh())
                self.detectionQueue.put(detections)

                detections_adjust = self.detections_adjust(detections)
                self.detectionsQueue.put(detections_adjust)

                darknet.free_image(darknet_image)

                id += 1

                # Truncate old frames.
                while self.darknetFrameQueue.qsize() > 1:
                    print('Frame with id = {} is truncated without yolo!'.format(id))
                    self.darknetFrameQueue.get()
                    id += 1
                    self.detectionQueue.put(detections)
                    self.detectionsQueue.put(detections_adjust)
                    pass


    def drawing(self) -> None:

        id = self.gid

        # Determine bbox colors.
        random.seed(3)

        while not self.globalData.isTerminated:
            if self.drawingRgbQueue.qsize() > 0 and self.detectionQueue.qsize() > 0:
                frame = self.drawingRgbQueue.get()
                detections = self.detectionQueue.get()
                detections_adjusted = []
                
                for label, confidence, bbox in detections:
                    bbox_adjusted = self.convert2original(frame, bbox)
                    detections_adjusted.append((str(label), confidence, bbox_adjusted))

                image = darknet.draw_boxes(detections_adjusted, frame, self.class_colors)
                cv2.imwrite(self.outputPath + ('yolov4_%010d.jpg' % (id)), image)
                print('"yolov4_%010d.jpg" saved' % (id))
                
                id += 1


    def detectQueue(self) -> None:

        Thread(target=self.convert_rgb_darknetImage).start()
        Thread(target=self.inference).start()

        if self.isLog:
            Thread(target=self.drawing).start()

