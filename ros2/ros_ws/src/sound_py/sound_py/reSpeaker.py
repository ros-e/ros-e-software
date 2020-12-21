#!/usr/bin/env python

"""
ROS node for the ReSpeaker Microphone Array 2

Node publishes:
'sound/is_speeching', Bool  --> boolean if module detects speeching
'sound/direction', Int32    --> Direction of sound source, value between -180 and 180

"""

import os
import sys
import struct

# USB for ReSpeaker
import usb.core   
import usb.util

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Bool, ColorRGBA
# from audio_common_msgs.msg import AudioData --> Audio Data

import time

# Math and calculation
import angles
import math
import numpy as np


from sound_py.respeakerInterface import RespeakerInterface

# Pixel Ring
try:
  from pixel_ring import usb_pixel_ring_v2
except IOError as e:
  print(e)
  raise RuntimeError("Check the device is connected and recognized")


class RespeakerNode(Node):
  
  def __init__(self):
    rclpy.get_default_context().on_shutdown(self.onShutdown)
    super().__init__('reSpeaker_node')

    ### init ReSpeaker Object 
    self.respeaker = RespeakerInterface()
    self.get_logger().info("ReSpeaker parameters:\n" + self.respeaker.read_all())

    ### init last published values
    self.prev_IsVoice = None
    self.prev_Doa = None

    ### define and announce publisher topics for ROS 
    self.pubVad = self.create_publisher(Bool, 'sound/is_speeching', 10)
    self.pubDoa = self.create_publisher(Int32, 'sound/direction', 10)
    #self.pubAudio = self.create_publisher(AudioData, "sound/audio", 10)
    #self.pubSpeechAudio = self.create_publisher(AudioData, "sound/speech_audio", 10)

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
      self.respeaker.close()
      self.mainTimer.shutdown()
    except:
      pass
    finally:
      self.respeaker = None
      self.mainTimer = None
    # try:
    #   self.respeaker_audio.stop()
    # except:
    #   pass
    # finally:
    #   self.respeaker_audio = None

  # Timer function polling respeaker values
  def onTimer(self):

    #stamp = event.current_real or self.get_clock().now()

    # get boolean for voice activation
    isVoice = bool(self.respeaker.is_voice())

    ### get direction of arrival degrees
    # add 90 degree offset to fit 180 to the middle of module
    # substract 180 to fit doa between -180 and 180
    doaRad = math.radians(((self.respeaker.direction + 90) % 360) - 180.0)
    doa = int(math.degrees(doaRad))

    
    # vad
    if isVoice != self.prev_IsVoice:
      self.pubVad.publish(Bool(data=isVoice))
      self.prev_IsVoice = isVoice
    
    # doa
    if doa != self.prev_Doa:
    #if True:
      self.pubDoa.publish(Int32(data=doa))
      self.prev_Doa = doa
      self.get_logger().info("DOA: {}".format(doa))

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