
from time import sleep
from Copter import Copter
from Copter import AttitudeType


copter = Copter()
copter.log_status()

copter.changeFlightMode('STABILIZE')
copter.arm()

copter.changeFlightMode('GUIDED_NOGPS')
copter.takeOff(1)

copter.set_attitude(AttitudeType.FORWARD, 3, 0.6)

copter.changeFlightMode('LAND')

