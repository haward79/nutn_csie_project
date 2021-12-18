
from time import sleep
from Light import Light, LightColor
from Copter import Copter


light = Light()
copter = Copter()
copter.log_status()

for i in range(1):
    copter.arm()
    copter.changeFlightMode('GUIDED_NOGPS')

    light.blink(LightColor.GREEN, 2)
    copter.takeOff(1)
    
    light.blink(LightColor.BLUE, 2)
    copter.land()

    #copter.log('Sleeping ......')
    #sleep(10)

light.blink(LightColor.GREEN_BLUE, 3)

