#!/usr/bin/env python2

"""
Python2 HelloWorld for I2C Control of the Audio Amplifier

Adapted from 
https://learn.adafruit.com/adafruit-20w-stereo-audio-amplifier-class-d-max9744/digital-control 

"""

import smbus # System management bus ==> I2C compatible
import time

"""
Class to control the Max9744 Audio Amplifier
"""
class Max9744(object):

  I2C_ADDRESS = 0x4B
  
  def __init__(self, i2cBus):
    
    if (i2cBus): 
      self.bus = i2cBus
    else: 
      self.bus = smbus.SMBus(0)

    ### Address of audio amplifier
    self.addr = Max9744.I2C_ADDRESS

    ### Volume --> value between 0 (mute) and 63 (max Volume)
    self.volume = 30

  def setVolume(self, volume = None):
    if (volume):
      self.volume = volume
      if (self.volume > 63): self.volume = 63
      elif (self.volume < 0): self.volume = 0

    self.bus.write_byte(self.addr, self.volume)
  
  def increaseVolume(self):
    self.volume = self.volume + 1
    if (self.volume > 63): self.volume = 63
    
    self.setVolume()

  def decreaseVolume(self):
    self.volume = self.volume - 1
    if (self.volume < 0): self.volume = 0
    
    self.setVolume()


""" 
Create smbus instance and open it.
The jetson nano has two I2C busses.

I2C Bus 0:
SDA --> Pin 27
SCL --> Pin 28

I2C Bus 1:
SDA --> Pin 3
SCL --> Pin 5
"""
bus = smbus.SMBus(1)

ampControl = Max9744(i2cBus=bus)

### Increase, decrease or set volume 
ampControl.decreaseVolume()
ampControl.increaseVolume()
ampControl.setVolume(20)




