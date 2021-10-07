
from enum import Enum
from time import sleep
from textLog import logText


class DroneAction(Enum):

    STOP = 0
    FORWARD = 1
    BACKWARD = -1
    LEFT = 2
    RIGHT = 3
    ROTATE_LEFT = 4
    ROTATE_RIGHT = 5


class Drone():

    def __init__(self) -> None:
        
        self.action = DroneAction.STOP
        self.logAction()


    def setAction(self, action, step: int = 1):

        if step > 0 and action >= -1 and action <= 5:
            self.action = action
            self.logAction()
            sleep(0.5)
        
        self.action = DroneAction.STOP
        self.logAction()


    def logAction(self):

        logText('Flying mode changed to "{}" .'.format(self.action))

