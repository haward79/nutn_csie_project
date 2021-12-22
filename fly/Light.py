
from time import sleep
import RPi.GPIO as GPIO

class LightColor():

    RED = 0
    GREEN = 1
    BLUE = 2
    GREEN_BLUE = 3
    BLUE_GREEN = 3
    PINK = 4
    YELLOW = 5

    def __init__(self):

        pass


class Light():

    RED_PIN = 15
    GREEN_PIN = 13
    BLUE_PIN = 11

    def __init__(self):

        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(self.RED_PIN, GPIO.OUT)
        GPIO.setup(self.GREEN_PIN, GPIO.OUT)
        GPIO.setup(self.BLUE_PIN, GPIO.OUT)

        self.turnOffAll()


    def __del__(self):
        
        GPIO.cleanup()


    def turnOffAll(self):

        GPIO.output(self.RED_PIN, False)
        GPIO.output(self.GREEN_PIN, False)
        GPIO.output(self.BLUE_PIN, False)

    
    def __illuminate(self, pins: list, duration: float):

        self.turnOffAll()

        for pin in pins:
            GPIO.output(pin, True)

        if duration >= 0:
            sleep(duration)
            self.turnOffAll()


    def illuminate(self, color: LightColor, duration: float = -1):

        pins = []

        if color == LightColor.RED:
            pins.append(self.RED_PIN)
        
        elif color == LightColor.GREEN:
            pins.append(self.GREEN_PIN)

        elif color == LightColor.BLUE:
            pins.append(self.BLUE_PIN)

        elif color == LightColor.GREEN_BLUE or color == LightColor.BLUE_GREEN:
            pins.append(self.GREEN_PIN)
            pins.append(self.BLUE_PIN)

        elif color == LightColor.PINK:
            pins.append(self.RED_PIN)
            pins.append(self.BLUE_PIN)

        elif color == LightColor.YELLOW:
            pins.append(self.RED_PIN)
            pins.append(self.GREEN_PIN)

        self.__illuminate(pins, duration)


    def blink(self, color: LightColor, times: int, duration: float = 0.3, interval: float = 0.1):

        if times >= 0 and duration >= 0 and interval >= 0:
            for i in range(times):
                if i != 0:
                    sleep(interval)

                self.illuminate(color, duration)

