#!/usr/bin/env python3

from respeakerInterface import RespeakerInterface # Interface to communicate with the reSpeaker module
import time

# Create reSpeaker instance
respeaker = RespeakerInterface()

if respeaker:

  while True:

    try:

      # Check if ReSpeaker detects loud sounds
      if (respeaker.is_voice()):
        # Print direction of sound
        print(respeaker.direction)
      time.sleep(0.2)

    except KeyboardInterrupt:
      break
