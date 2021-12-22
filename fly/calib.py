
from time import sleep
from Light import Light, LightColor
from Copter import AttitudeType, Copter


light = Light()
copter = Copter()
copter.log_status()

copter.arm()
copter.changeFlightMode('GUIDED_NOGPS')

copter.set_attitude(AttitudeType.STOP, 5, 0.6)

light.blink(LightColor.GREEN, 2)
copter.set_attitude(AttitudeType.FORWARD, 3, 0.5)

light.blink(LightColor.BLUE, 2)
copter.set_attitude(AttitudeType.BACKWARD, 3, 0.5)

light.blink(LightColor.GREEN_BLUE, 2)
copter.land()

