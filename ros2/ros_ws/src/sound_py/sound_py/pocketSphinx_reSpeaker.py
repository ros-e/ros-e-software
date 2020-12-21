#!/usr/bin/python

"""

Altered reSpeaker.py by Monika, adding pocketshpinx

This file was renamed and the reSpeaker.py without pocketsphinx was reverted by Johannes, because this file caused errors when starting the node.
If the pocketsphinx contents are become relevant again, they should be implemented in microphone.py unless reSpeaker hardware functions are needed
since microphone.py handles normal picrohpne input streams


ROS node for the ReSpeaker Microphone Array 2

Node publishes:
'sound/is_speeching', Bool  --> boolean if module detects speeching
'sound/direction', Int32    --> Direction of sound source, value between -180 and 180

'sound/speech_text_recognized', String   --> detected spoken text 
"""

import os
import sys

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Bool, ColorRGBA # Default types for ros publishing
# from audio_common_msgs.msg import AudioData --> Audio Data

import time

# Math and calculation
import angles
import math
import numpy as np

from pocketsphinx import LiveSpeech
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from threading import Thread, Event

PY_VERSION = int(sys.version[:1])

# for PocketSphinx
BASEDIR = os.path.join(os.environ.get("HOME"), 'PocketSphinx')
if PY_VERSION == 2:
    print('BASEDIR: ', BASEDIR)
else:
    print('BASEDIR:', BASEDIR)
#TODO get language from redis
#LANGUAGE = 'de-de'
LANGUAGE = 'en-us'
MODELDIR = os.path.join(BASEDIR, 'model', LANGUAGE)
if PY_VERSION == 2:
    print('MODELDIR: ', MODELDIR)
else:
    print('MODELDIR:', MODELDIR)
DATADIR = os.path.join(BASEDIR, 'data')

from respeakerInterface import RespeakerInterface

class RespeakerNode(Node):
  
  def __init__(self):
    #self.quit_event = Event()
    super().__init__('reSpeaker_node')

    rclpy.get_default_context().on_shutdown(self.onShutdown)

    ### init ReSpeaker Object 
    self.respeaker = RespeakerInterface()

    ### init last published values
    self.prev_IsVoice = None
    self.prev_Doa = None

    ### define and announce publisher topics for ROS 
    self.pubVad = self.create_publisher(Bool, 'sound/is_speeching', 10)
    self.pubDoa = self.create_publisher(Int32, 'sound/direction', 10)
    #self.pubAudio = self.create_publisher(AudioData, "sound/audio", 10)
    #self.pubSpeechAudio = self.create_publisher(AudioData, "sound/speech_audio", 10)
    self.pubSpeechText = self.create_publisher(String, 'sound/speech_text_recognized', 10)

    #self.thread = Thread(target=self.liveSpeechTask, args=(self.quit_event,))
    #self.thread.start()
    self.liveSpeechTask()

    ### define and announce subsribed topics
    self.subLed = self.create_subscription(ColorRGBA, "visual/respeaker/ledRing", self.onStatusLed, 10)

    # update rate for the reSpeaker values
    self.updateRate = 10

    # Timer for ROS events and callbacks
    self.mainTimer = self.create_timer(1.0 / self.updateRate, self.onTimer)
    self.ledTimer = None

    self.get_logger().info("ReSpeaker Node started")

  def onShutdown(self):
    try:
        self.quit_event.set()
    except:
      pass
    finally:
      self.quit_event = None

    try:
        self.thread.join()
    except:
      pass
    finally:
      self.thread = None

    try:
      self.respeaker.close()
    except:
      pass
    finally:
      self.respeaker = None
    # try:
    #   self.respeaker_audio.stop()
    # except:
    #   pass
    # finally:
    #   self.respeaker_audio = None

  def liveSpeechTask(self):
    #TODO
    pass
    # Create a decoder with certain model
#    config = Decoder.default_config()
#    config.set_string('-hmm', os.path.join(MODELDIR, LANGUAGE))
#    config.set_string('-lm', os.path.join(MODELDIR, LANGUAGE + '.lm.bin'))
#    config.set_string('-dict', os.path.join(MODELDIR, 'cmudict-' + LANGUAGE + '.dict'))
#    decoder = Decoder(config)

#    p = pyaudio.PyAudio()
#    stream = p.open(
#        format=pyaudio.paInt16,
#        channels=1,
#        rate=16000,
#        input=True,
#        frames_per_buffer=1024
#    )
#    stream.start_stream()

#    in_speech_bf = False
#    decoder.start_utt()

#    while True:
#        buf = stream.read(1024)
#        if buf:
#            decoder.process_raw(buf, False, False)
#            if decoder.get_in_speech() != in_speech_bf:
#                in_speech_bf = decoder.get_in_speech()
#                if not in_speech_bf:
#                    decoder.end_utt()
#                    speech_text = decoder.hyp().hypstr
#                    print('\033[32mResult: ', speech_text, '\033[0m')
#                    self.pubSpeechText.publish(String(data=speech_text))
#                    decoder.start_utt()
#        else:
#            break

#    decoder.end_utt()


#    try:
#        speech = LiveSpeech(
#            verbose=True,
#            logfn=os.path.join(DATADIR, 'pocketsphinx.log'),
#            sampling_rate=16000,
#            buffer_size=2048,
#            no_search=False,
#            full_utt=False,
#            hmm=os.path.join(MODELDIR, LANGUAGE),
#            lm=os.path.join(MODELDIR, LANGUAGE + '.lm.bin'),
#            dic=os.path.join(MODELDIR, 'model', 'cmudict-' + LANGUAGE + '.dict')
#        )

#        speech.decode()
#        speech_text = speech.hypothesis()
#        print('\033[34m', speech_text, '\033[0m')
#        self.pubSpeechText.publish(String(data=speech_text))

#        for phrase in speech:
#            print('\033[32m', phrase, '\033[0m')
#    except KeyboardInterrupt:
#        print('Quit')
        #quit_event.set()

  # Timer function polling respeaker values
  def onTimer(self, event):

    stamp = event.current_real or self.get_clock().now()

    # get boolean for voice activation
    isVoice = self.respeaker.is_voice()

    ### get direction of arrival degrees
    # add 90 degree offset to fit 180 to the middle of module
    # substract 180 to fit doa between -180 and 180
    doaRad = math.radians(((self.respeaker.direction + 90) % 360) - 180.0)
    doa = math.degrees(doaRad)

    # vad
    if isVoice != self.prev_IsVoice:
      self.pubVad.publish(Bool(data=isVoice))
      self.prev_IsVoice = isVoice
    
    # doa
    if doa != self.prev_Doa:
    #if True:
      self.pubDoa.publish(Int32(data=doa))
      self.prev_Doa = doa
      self.get_logger().info(doa)

  def onStatusLed(self, msg):
    self.respeaker.set_led_color(r=msg.r, g=msg.g, b=msg.b, a=msg.a)
    if self.ledTimer and self.ledTimer.is_alive():
      self.ledTimer.shutdown()

    self.ledTimer = self.create_timer(3.0,
                                       lambda e: self.respeaker.set_led_trace(),
                                       oneshot=True)



def main(args=None):

  rclpy.init(args=args) # 'reSpeaker_node'

  node = RespeakerNode()

  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()