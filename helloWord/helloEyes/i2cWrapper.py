#!/usr/bin/env python

"""
Class to wrap I2C commands.
At the moment this class opens its own i2c bus, later that can be changed to an ROS Topic.
"""

import smbus # System management bus ==> I2C compatible
import time

class I2C(object):
  def __init__(self, bus, address):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.maxData = 31
    self.delay = 0

  def writeCmdByte(self, cmd, byte):
    self.bus.write_byte_data(self.address, cmd, byte)    

  def writeCmdBlockData(self, cmd, data):
    l = len(data)

    if (l <= self.maxData):
      self.bus.write_i2c_block_data(self.address, cmd, data[0:])
            
    else:
      
      start = 0
      while start < l:
        if start + 1 == l:
          self.bus.write_byte_data(self.address, cmd, data[start])
        else:
          self.bus.write_i2c_block_data(self.address, cmd, data[(start):(start + self.maxData)])
          
        start += self.maxData