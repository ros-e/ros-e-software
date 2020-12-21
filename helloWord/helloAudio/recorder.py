#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Finds the Microphone as audio input device and records some seconds of audio to a file.

@author Johannes Sommerfeldt
"""

import sys
import pyaudio
import wave
import time

### Constants
DEVICE_NAME = "ReSpeaker 4 Mic Array"
SAMPLE_WIDTH = 2
BUFFER_SIZE = 1024
FILENAME = "recorded.wav"
DEFUALT_REC_DURATION = 10   # how many seconds to record if not specified in command line args

#####

# get how long to record
recording_duration = float(sys.argv[1]) if len(sys.argv) > 1 else DEFUALT_REC_DURATION

pyaudio_instance = pyaudio.PyAudio()

### find the device info of the ReSpeaker Mic
device_index = None

for i in range(pyaudio_instance.get_device_count()):
    device_info = pyaudio_instance.get_device_info_by_index(i)

    name = device_info['name'].encode('utf-8')
    if name.find(DEVICE_NAME) >= 0:
        # Get the sample rate and channel count from the device info so no constants have to be changed when changing the firmware.
        sample_rate = int(device_info['defaultSampleRate'])
        number_of_channels = device_info['maxInputChannels']

        device_index = i
        print('Found {} with {} max input channels and {} default sample rate at index {}'
                .format(name, number_of_channels, sample_rate, device_index))
        break

if device_index is None:
    raise ValueError('Cannot find any input device with the name {}'.format(DEVICE_NAME))

# open stream
stream = pyaudio_instance.open(
        format=pyaudio.get_format_from_width(SAMPLE_WIDTH), 
        channels=number_of_channels, 
        rate=sample_rate, 
        input=True, 
        frames_per_buffer=BUFFER_SIZE,
        input_device_index=device_index)

frames = []

print("recording...")
start_time = time.time()
while time.time() < start_time + recording_duration:
  data = stream.read(BUFFER_SIZE)
  frames.append(data)

stream.stop_stream()
stream.close()
pyaudio_instance.terminate()

print("done recording")

wf = wave.open(FILENAME, 'wb')
wf.setnchannels(number_of_channels)
wf.setsampwidth(SAMPLE_WIDTH)
wf.setframerate(sample_rate)
wf.writeframes(b''.join(frames))
wf.close()

print("audio saved to file.")



