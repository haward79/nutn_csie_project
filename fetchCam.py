
from threading import Thread
from queue import Queue
import numpy as np
import cv2
from cv_bridge import CvBridge


class FetchCam():

    def __init__(self, globalData) -> None:
        
        self.globalData = globalData
        self.gid = globalData.getConstInitGid()
        self.colorId = globalData.getConstInitGid()
        self.depthId = globalData.getConstInitGid()
        self.isLog = globalData.getConstIsLog()
        self.outputPath = globalData.getConstOutputPath()
        self.colorQueue = globalData.colorQueue
        self.depthQueue = globalData.depthQueue
        self.writeColorQueue = Queue()
        self.writeDepthQueue = Queue()

        if self.isLog:
            Thread(target=self.writeImages).start()


    def getConstMaxDepthMeter(self) -> float:

        # For ZB215 only.
        return 9.22


    def writeImages(self) -> None:

        colorId = self.gid
        depthId = self.gid

        while not self.globalData.isTerminated:
            if self.writeColorQueue.qsize() > 0:
                cv2.imwrite(self.outputPath + ('color_%010d.jpg' % (colorId)), self.writeColorQueue.get())
                print('"color_%010d.jpg" saved' % (colorId))

                colorId += 1

            if self.writeDepthQueue.qsize() > 0:
                cv2.imwrite(self.outputPath + ('depth_%010d.jpg' % (depthId)), self.writeDepthQueue.get())
                print('"depth_%010d.jpg" saved' % (depthId))

                depthId += 1


    def fetchColor(self, data) -> None:
    
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        self.colorQueue.put(image)

        if self.isLog:
            self.writeColorQueue.put(image)

        self.colorId += 1


    def fetchDepth(self, data) -> None:

        # Unit: 1 == 0.05 cm
    
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        image = np.multiply(image, 0.0005)  # Convert depth unit to meter.
        self.depthQueue.put(image)

        if self.isLog:
            imageScaled = np.divide(image, self.getConstMaxDepthMeter())
            imageScaled = np.multiply(imageScaled, 255)
            imageScaled = imageScaled.astype(int)
            imageScaled = np.uint8(imageScaled)
            
            self.writeDepthQueue.put(imageScaled)

        self.depthId += 1

