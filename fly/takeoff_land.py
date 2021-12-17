
from time import sleep
from Sound import playSound, SoundType
from Copter import Copter


copter = Copter()
copter.log_status()

for i in range(1):
    playSound(SoundType.BEEP_SHORT, 2)
    copter.arm()

    copter.changeFlightMode('GUIDED_NOGPS')
    copter.takeOff(1)
    
    playSound(SoundType.BEEP_SHORT)
    copter.land()

    #copter.log('Sleeping ......')
    #sleep(10)

playSound(SoundType.BEEP_SHORT, 3)

