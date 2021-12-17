
from time import sleep
from Sound import playSound, SoundType
from Copter import AttitudeType, Copter


copter = Copter()
copter.log_status()

copter.arm()

copter.changeFlightMode('GUIDED_NOGPS')
copter.set_attitude(AttitudeType.STOP, 5, 0.6)
playSound(SoundType.BEEP_SHORT, 2)
#copter.set_attitude(AttitudeType.FORWARD, 5, 0.5)
copter.land()

