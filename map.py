
from threading import Thread
from queue import Queue
import numpy as np
import cv2
from drawing import drawSquare
from drone import DroneAction
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
from textLog import logText


class Map():

    def __init__(self, globalData):

        self.globalData = globalData
        self.gid = globalData.getConstInitGid()
        self.voronoiId = self.gid
        self.decisionId = self.gid
        self.isLog = globalData.getConstIsLog()
        self.outputPath = globalData.getConstOutputPath()
        self.detectionsQueue = globalData.detectionsQueue
        self.drone = globalData.drone

        self.isPoseSet = False
        self.pixelToMeter = 0.05
        self.mapWidth = 0  # Unit: pixel
        self.mapHeight = 0  # Unit: pixel
        self.mapOrigin = (0, 0)  # Unit: pixel
        self.globalLocation = (0, 0)  # Unit: pixel
        self.map = np.zeros((1, 1, 1), np.uint8)
        self.writeMapQueue = Queue()

        self.peopleLocations = []  # Unit: global coord
        self.writePeopleLocationsQueue = Queue()

        Thread(target=self.droneDo).start()

        if self.isLog:
            Thread(target=self.writeMap).start()
            Thread(target=self.writePeopleLocation).start()


    def getConstUnitBlockMeter(self) -> float:

        return 0.6


    def getConstDiff(self) -> float:

        return int(self.getConstUnitBlockMeter() / self.pixelToMeter)


    def swap2D(self, a, b):

        return (b, a)


    def isValidLocationInMap(self, location: tuple) -> bool:

        return (location[0] >= 0 and location[0] < self.mapHeight and location[1] >= 0 and location[1] < self.mapWidth)


    def isValidLocationInGlobal(self, location: tuple) -> bool:

        location = self.globalCoordToMapCoord(location)
        return self.isValidLocationInMap(location)


    def mapCoordToGlobalCoord(self, mapCoord: tuple) -> tuple:

        globalCoordX = self.mapOrigin[0] - mapCoord[0]
        globalCoordY = self.mapOrigin[1] - mapCoord[1]

        return (globalCoordX, globalCoordY)


    def globalCoordToMapCoord(self, globalCoord: tuple) -> tuple:

        mapCoordX = self.mapOrigin[0] - globalCoord[0]
        mapCoordY = self.mapOrigin[1] - globalCoord[1]

        return (mapCoordX, mapCoordY)


    def tupleLocationsToListLocations(self, locations: tuple):

        result = []

        for location in locations:
            loc = [location[0], location[1]]
            result.append(loc)

        return result


    def peopleLocationsToVoronoiLocations(self, peopleLocations: list):

        voronoiLocations = []

        for location in peopleLocations:
            (locX, locY) = self.globalCoordToMapCoord((location[0], location[1]))
            voronoiLocations.append([locX, locY])

        return voronoiLocations


    def writeMap(self):

        mapId = self.gid

        while not self.globalData.isTerminated:
            if self.writeMapQueue.qsize() > 0:
                cv2.imwrite(self.outputPath + ('slam_%010d.jpg' % (mapId)), self.writeMapQueue.get())
                print('"slam_%010d.jpg" saved' % (mapId))

                mapId += 1


    def writePeopleLocation(self):

        plId = self.gid

        while not self.globalData.isTerminated:
            if self.writePeopleLocationsQueue.qsize() > 0:
                locations = self.writePeopleLocationsQueue.get()
                map = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)

                for location in locations:
                    location = self.globalCoordToMapCoord(location)

                    if self.isValidLocationInMap(location):
                        map = drawSquare(map, location[0], location[1], 3, 3, (0, 255, 255), True)
                        logText('(id = {}) Person @(x = {}, y = {}) is a valid MAP point during write.'.format(plId, location[0], location[1]))
                    else:
                        logText('(id = {}) Person @(x = {}, y = {}) is NOT a valid MAP point during write.'.format(plId, location[0], location[1]))

                currentLocation = self.globalCoordToMapCoord(self.globalLocation)
                map = drawSquare(map, currentLocation[0], currentLocation[1], 3, 3, (0, 0, 255), True)

                cv2.imwrite(self.outputPath + ('peopleLocations_%010d.jpg' % (plId)), map)
                print('"peopleLocations_%010d.jpg" saved' % (plId))

                plId += 1


    def fetchMap2d(self, data):

        # Unit: 1 == 0.05 meter (resolution)
        # data.info.origin.position.x
        # data.info.origin.position.y

        self.pixelToMeter = data.info.resolution
        self.mapWidth = data.info.width
        self.mapHeight = data.info.height
        self.mapOrigin = (int(-data.info.origin.position.y / self.pixelToMeter), int(-data.info.origin.position.x / self.pixelToMeter))

        map = []
        for data in data.data:
            if data < 0:
                map.append(255)
            elif data == 0:
                map.append(255)
            else:
                map.append(0)

        self.map = np.uint8(np.asanyarray(map).reshape((self.mapHeight, self.mapWidth)))
        self.writeMapQueue.put(self.map)

        logText('Map updated to size (w = {}, h = {}) px.'.format(self.mapWidth, self.mapHeight))


    def fetchPose(self, data):

        # Unit: 1 == 0.05 meter (resolution)
        # data.pose.pose.position.x
        # data.pose.pose.position.y

        self.isPoseSet = True

        self.globalLocation = (int(data.pose.pose.position.x / self.pixelToMeter), int(data.pose.pose.position.y / self.pixelToMeter))
        logText('Global Location: (x = {}, y = {}) px'.format(self.globalLocation[0], self.globalLocation[1]))


    def update(self):

        id = self.gid

        while not self.globalData.isTerminated:
            if self.detectionsQueue.qsize() > 0:
                peopleLocations = []
                detections = self.detectionsQueue.get()

                # Fetch person location and save it.
                for detection in detections:
                    if detection['name'] == 'person':
                        x = detection['bounding'][0]
                        y = detection['bounding'][1]
                        width = detection['bounding'][2]
                        height = detection['bounding'][3]

                        mx = x + int(width / 2)
                        my = y + int(height / 2)

                        depthFrame = self.globalData.depthQueue.get()
                        pl_depth_meter = depthFrame[my, mx]
                        pl_depth_pixel = int(pl_depth_meter / self.pixelToMeter)
                        currentLocation = self.globalLocation

                        location = (currentLocation[0] + pl_depth_pixel, currentLocation[1])

                        if self.isValidLocationInGlobal(location):
                            peopleLocations.append(location)
                            logText('Person @(x = {}, y = {}) px is a valid GLOBAL point.'.format(location[0], location[1]))
                        else:
                            logText('Person @(x = {}, y = {}) px is NOT a valid GLOBAL point.'.format(location[0], location[1]))

                self.peopleLocations = peopleLocations
                self.writePeopleLocationsQueue.put(peopleLocations)

                id += 1


    def drone_stop(self):

        self.drone.setAction(DroneAction.STOP)


    def drone_forward(self):

        if self.globalData.isTerminated:
            return

        step = int(self.getConstUnitBlockMeter() / self.pixelToMeter)
        self.globalLocation = (self.globalLocation[0] + step, self.globalLocation[1])
        self.drone.setAction(DroneAction.FORWARD, step)


    def drone_rotateLeft(self):

        if self.globalData.isTerminated:
            return

        (self.mapWidth, self.mapHeight) = self.swap2D(self.mapWidth, self.mapHeight)
        
        self.map = np.rot90(self.map, 3)
        self.mapOrigin = (self.mapOrigin[1], self.mapWidth - self.mapOrigin[0])
        self.drone.setAction(DroneAction.ROTATE_LEFT, 1)


    def drone_rotateRight(self):

        if self.globalData.isTerminated:
            return

        (self.mapWidth, self.mapHeight) = self.swap2D(self.mapWidth, self.mapHeight)

        self.map = np.rot90(self.map)
        self.mapOrigin = (self.mapHeight - self.mapOrigin[1], self.mapOrigin[0])
        self.drone.setAction(DroneAction.ROTATE_RIGHT, 1)


    def drone_rotateBack(self):

        if self.globalData.isTerminated:
            return

        self.drone_rotateRight()
        self.drone_rotateRight()


    def droneDo(self):

        # Rotate drone clockwise and try to get pose.
        logText('[GOAL] Try to build SLAM')
        while not self.globalData.isTerminated and not self.isPoseSet:
            for i in range(4):
                self.drone_rotateRight()

            for i in range(4):
                self.drone_rotateLeft()


        # Draw voronoi diagram.
        self.peopleLocations = [(-42, -22), (-30, -61), (-51, -42), (-83, -64)]
        peopleLocs = self.peopleLocationsToVoronoiLocations(self.peopleLocations)
        voronoi = Voronoi(peopleLocs)
        voronoiVertices = voronoi.vertices
        voronoi_plot_2d(voronoi)
        plt.savefig(self.outputPath + '')
        plt.savefig(self.outputPath + ('voronoi_%010d.jpg' % (self.voronoiId)))
        self.voronoiId += 1


        # Go to target.
        logText('[GOAL] Try to reach target')
        while not self.globalData.isTerminated and (abs(self.globalData.getConstTargetLocation()[0] - self.globalLocation[0]) >= self.getConstDiff() or abs(self.globalData.getConstTargetLocation()[1] - self.globalLocation[1]) >= self.getConstDiff()):
            if self.globalLocation[0] < self.globalData.getConstTargetLocation()[0] and abs(self.globalLocation[0] - self.globalData.getConstTargetLocation()[0]) >= self.getConstDiff():
                logText('(decisionId = {}) Go FORWARD.'.format(self.decisionId))
                steps = int(self.getConstUnitBlockMeter() / self.pixelToMeter)
                hasObstacle = False

                for step in range(1, steps+1):
                    loc = (self.globalLocation[0] + step, self.globalLocation[1])
                    
                    if loc == 0:
                        hasObstacle = True
                        break

                if hasObstacle:
                    if self.globalLocation[1] < self.globalData.getConstTargetLocation()[1]:
                        self.drone_rotateLeft()
                    else:
                        self.drone_rotateRight()
                else:
                    self.drone_forward()

            elif self.globalLocation[0] > self.globalData.getConstTargetLocation()[0] and abs(self.globalLocation[0] - self.globalData.getConstTargetLocation()[0]) >= self.getConstDiff():
                logText('(decisionId = {}) Go BACK.'.format(self.decisionId))
                self.drone_rotateBack()
            elif self.globalLocation[1] < self.globalData.getConstTargetLocation()[1] and abs(self.globalLocation[1] - self.globalData.getConstTargetLocation()[1]) > self.getConstDiff():
                logText('(decisionId = {}) Go LEFT.'.format(self.decisionId))
                self.drone_rotateLeft()
            elif self.globalLocation[1] > self.globalData.getConstTargetLocation()[1] and abs(self.globalLocation[1] - self.globalData.getConstTargetLocation()[1]) > self.getConstDiff():
                logText('(decisionId = {}) Go RIGHT.'.format(self.decisionId))
                self.drone_rotateRight()
            else:
                logText('(decisionId = {}) Failed to make decision.'.format(self.decisionId))

            self.decisionId += 1
            logText('Global Location: (x = {}, y = {}) px'.format(self.globalLocation[0], self.globalLocation[1]))


        # Reach target.
        logText('[GOAL] Reached target.')
        self.globalData.isTerminated = True

