#! /usr/bin/env python

from __future__ import print_function

import time
from pygame import mixer

AUDIO_FILE='/home/dharmin/Music/waltz_trimmed.wav'

class MusicPlayer(object):
    def __init__(self, file_name=None):
        mixer.init()
        if file_name is None:
            self.sound_obj = None
        else:
            self.sound_obj = mixer.Sound(file_name)

    def start_playing(self):
        if self.sound_obj is not None:
            self.sound_obj.play()

    def stop_playing(self):
        if self.sound_obj is not None:
            self.sound_obj.stop()

if __name__ == "__main__":
    MP = MusicPlayer(AUDIO_FILE)
    MP.start_playing()
    time.sleep(3)
    MP.stop_playing()
