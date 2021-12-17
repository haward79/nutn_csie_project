
from time import sleep
import subprocess


class SoundType():

    PIANO_C = 'sound/pianoC.mp3'
    BEEP = 'sound/beep.wav'
    BEEP_SHORT = 'sound/beep_short.wav'

    def __init__(self):

        pass


def playSound(soundName: SoundType, times: int = 1, interval: float = 0) -> None:

    if times >= 1 and interval >= 0:
        for i in range(times):
            subprocess.check_call(['mpv', soundName])
            sleep(interval)

