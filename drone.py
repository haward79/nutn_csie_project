
from enum import Enum
from time import sleep
from textLog import logText


class DroneAction(Enum):

    STOP = 0
    FORWARD = 1
    ROTATE_LEFT = 2
    ROTATE_RIGHT = 3


class Drone():

    def __init__(self, globalData) -> None:
        
        self.globalData = globalData
        self.action = DroneAction.STOP
        self.logAction()


    def setAction(self, action: DroneAction, step: int = 1):

        if step > 0:
            self.action = action
            self.logAction()
            
            self.globalData.communication.receiveData('\nFlying action changed to {} .'.format(self.action) + '\nPress ENTER after action is done ......')


    def logAction(self):

        logText('Flying action changed to {} .'.format(self.action))

