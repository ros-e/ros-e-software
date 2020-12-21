#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

Hello World for playing audio files.
Just a simple example with "playsound"

Make sure to select the correct device (e.g. ReSpeaker) as audio output device.

Article about sound playing in python: https://realpython.com/playing-and-recording-sound-python/#playing-audio-files
"""

from playsound import playsound
print "Test sound"

playsound('testSoundBell.wav')