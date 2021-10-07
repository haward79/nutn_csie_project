
from threading import Thread
from queue import Queue
import numpy as np
import cv2
from drawing import drawSquare
from textLog import logText


class Map():

    def __init__(self, globalData):

        self.globalData = globalData
        self.gid = globalData.getConstInitGid()
        self.isLog = globalData.getConstIsLog()
        self.outputPath = globalData.getConstOutputPath()
        self.detectionsQueue = globalData.detectionsQueue

        self.pixelToMeter = 0.5
        self.mapWidth = 0  # Unit: pixel
        self.mapHeight = 0  # Unit: pixel
        self.mapOrigin = (0, 0)  # Unit: pixel
        self.globalLocation = (0, 0)  # Unit: pixel
        self.orientation = 0  # Clock wise (0, 1, 2, 3)
        self.map = np.zeros((1, 1, 1), np.uint8)
        self.writeMapQueue = Queue()

        self.peopleLocations = []  # Unit: global coord
        self.writePeopleLocationsQueue = Queue()

        if self.isLog:
            Thread(target=self.writeMap).start()
            Thread(target=self.writePeopleLocation).start()


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

                        if self.orientation == 0:
                            location = (currentLocation[0] + pl_depth_pixel, currentLocation[1])
                        elif self.orientation == 1:
                            location = (currentLocation[0], currentLocation[1] - pl_depth_pixel)
                        elif self.orientation == 2:
                            location = (currentLocation[0] - pl_depth_pixel, currentLocation[1])
                        else:
                            location = (currentLocation[0], currentLocation[1] + pl_depth_pixel)

                        if self.isValidLocationInGlobal(location):
                            peopleLocations.append(location)
                            logText('Person @(x = {}, y = {}) is a valid GLOBAL point.'.format(location[0], location[1]))
                        else:
                            logText('Person @(x = {}, y = {}) is NOT a valid GLOBAL point.'.format(location[0], location[1]))

                self.peopleLocations = peopleLocations
                self.writePeopleLocationsQueue.put(peopleLocations)

                # Draw voronoi diagram.

                id += 1

