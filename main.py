
from datetime import datetime
from os.path import isfile
from os.path import isdir
from os import mkdir
from queue import Queue
from threading import Thread
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from communication import Communication
from drone import Drone
from fetchCam import FetchCam
from yoloDetect import YoloDetect
from map import Map
from textLog import logText


class ShareData():

    def __init__(self) -> None:

        # Get timestamp.
        self.outputPath = 'output/' + datetime.now().strftime('%Y%m%d_%I%M%S') + '/'

        # Check existence of output directory.
        if not isdir('output'):
            mkdir('output')
            
        if not isdir(self.outputPath):
            mkdir(self.outputPath)

        self.targetLocation = (0, 0)

        self.colorQueue = Queue()
        self.depthQueue = Queue()
        self.detectionsQueue = Queue()

        self.isTerminated = False
        self.isGuidedMode = False

        Thread(target=self.checkTermination).start()

        # Start communicator.
        self.communication = Communication(self)

        # Control drone.
        self.drone = Drone(self)


    def getConstInitGid(self) -> int:

        return 1


    def getConstIsLog(self) -> bool:

        return True


    def getConstOutputPath(self) -> str:

        return self.outputPath


    def getConstTargetLocation(self) -> str:

        return self.targetLocation


    def getConstIsGuidedMode(self) -> bool:

        return self.isGuidedMode


    def checkTermination(self):

        while not self.isTerminated:
            if not isfile('/home/user/Workspace/project/.isTerminated_False'):
                self.isTerminated = True

        logText('Termination signal is DETECTED.')
        self.communication.sendData('\nTermination signal is DETECTED.\nPress ENTER to exit\n')


globalData = ShareData()
rospy.init_node('project')


# Read color frame and depth frame from camera.
fc = FetchCam(globalData)
rospy.Subscriber('/camera/color/image_raw', Image, fc.fetchColor)
rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, fc.fetchDepth)


# Detect object by yolo.
yd = YoloDetect(globalData)
Thread(target=yd.detectQueue).start()


# Create map.
map = Map(globalData)
Thread(target=map.update).start()

rospy.Subscriber('/rtabmap/proj_map', OccupancyGrid, map.fetchMap2d)
rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped, map.fetchPose)

