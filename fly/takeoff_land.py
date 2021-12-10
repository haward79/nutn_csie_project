
from time import sleep
from Sound import playSound, SoundType
from Copter import Copter


copter = Copter()
copter.log_status()

for i in range(1):
    playSound(SoundType.BEEP, 2)
    
    copter.changeFlightMode('STABILIZE')
    copter.arm()

    copter.changeFlightMode('GUIDED_NOGPS')
    copter.takeOff(1.5)
    
    playSound(SoundType.BEEP)
    copter.land()

    #copter.log('Sleeping ......')
    #sleep(10)

playSound(SoundType.BEEP, 3)

