#!/usr/bin/env python

"""
I2C ROS 2 Node.

This node subscribes ROS-Messages to send data over I2C

I2C: 
http://www.netzmafia.de/skripten/hardware/RasPi/RasPi_I2C.html 
and
https://raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2 


"""

import os
import sys

import smbus # System management bus ==> I2C compatible
import time

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 

import threading
sem = threading.Semaphore()

""" 
Some addresses of the system. 
I2C Addresses --> 7 Bit Length 
"""

JETSON_I2CADDRESS = 0x00

ARDUINO_I2CADDR = 0x08  

AUDIO_MAX9744_I2CADDR = 0x4B # https://learn.adafruit.com/adafruit-20w-stereo-audio-amplifier-class-d-max9744/digital-control

OLED_DISPLAY_1_I2CADDR = 0x3C # https://www.adafruit.com/product/938 
OLED_DISPLAY_2_I2CADDR = 0x3D


""" 
Create smbus instance and open the instance.
The jetson nano has two I2C busses.

I2C Bus 0:
SDA --> Pin 27
SCL --> Pin 28

I2C Bus 1:
SDA --> Pin 3
SCL --> Pin 5

If there are any I2C devices attached, you can scan that bus from the command line
$ i2cdetect -y -r 0 
$ i2cdetect -y -r 1
"""


class I2CNode(Node):

  def __init__(self, bus = 1):
    super().__init__('i2cBridge_node')

    self.get_logger().info("Try starting I2C Bridge Node")

    rclpy.get_default_context().on_shutdown(self.onShutdown)

    # Open I2C Bus
    try:
      self.bus = smbus.SMBus(bus)
      self.get_logger().info("Successfully opened SMbus({})".format(bus))
    except Exception as e:
      self.get_logger().info("{}".format(e))
      self.bus = BusSim(bus, self)
    
    # Subscribe to I2C Bridge Topics

    self.subI2C8    = self.create_subscription(I2Cwrite8, "system/i2c/write8", self.onWrite8, 10)
    self.subI2C16   = self.create_subscription(I2Cwrite16, "system/i2c/write16", self.onWrite16, 10)
    self.subI2CArr  = self.create_subscription(I2CwriteArray, "system/i2c/writeArray", self.onWriteArray, 10)

    ################################
    ### LOG INFO ###################
    self.get_logger().info("Subscribed: system/i2c/write8      |  Msg: system/I2Cwrite8")
    self.get_logger().info("Subscribed: system/i2c/write16     |  Msg: system/I2Cwrite16")
    self.get_logger().info("Subscribed: system/i2c/writeArray  |  Msg: system/I2CwriteArray")
    

    ################################

  def onShutdown(self):
    self.bus.close()

  def onWrite16(self, msg):
    info = "I2C write16:    addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), str(msg.data))
    self.get_logger().info(info)

    try:
      self.bus.write_word_data(msg.address, msg.command, msg.data)
    except Exception as e:
      self.get_logger().error(str(e))

    # sem.acquire()
    # try:
    #   self.bus.write_word_data(msg.address, msg.command, msg.data)
    # except expression as identifier:
    #   pass
    # finally:
    #   time.sleep(0.005)
    #   sem.release()  
    
    
  
  def onWrite8(self, msg):
    info = "I2C write8:     addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), str(msg.data))
    self.get_logger().info(info)
    
    try:
      self.bus.write_byte_data(msg.address, msg.command, msg.data)
    except Exception as e:
      self.get_logger().error(str(e))

    

    # sem.acquire()
    # try:
    #   self.bus.write_byte_data(msg.address, msg.command, msg.data)
    # except expression as identifier:
    #   pass
    # finally:
    #   time.sleep(0.005)
    #   sem.release()  

    
    #time.sleep(0.005)

  def onWriteArray(self, msg):
    
    # self.get_logger().info(type(msg.data))
    
    data = []

    for d in msg.data:
      data.append(int(d))

    for i in range(2):
      try:
        self.bus.write_block_data(msg.address, msg.command, data)
        info = "I2C writeArray: addr: {} cmd: {} data: {}".format(str(msg.address), str(msg.command), list(msg.data))
        self.get_logger().info(info)
        break
      except Exception as e:
        self.get_logger().error(str(e) + " ... Trying again")
        time.sleep(0.05)
        
    

    # sem.acquire()
    # try:
    #   self.bus.write_block_data(msg.address, msg.command, data)
    # except expression as identifier:
    #   pass
    # finally:
    #   time.sleep(0.005)
    #   sem.release()  

    
    #time.sleep(0.005)


def main(args=None):
    
  # Announce node
  # rclpy.init(args=args) # 'i2cBridge_node'
  rclpy.init(args=args)

  node = I2CNode()

  # Run forever
  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()


class BusSim(object):
  def __init__(self, num, rosNode):

    self.num = num
    self.node = rosNode
    self.log = self.node.get_logger()

    self.log.info("####### ERROR OPENING SMbus #######")
    self.log.info("Starting SMbus Simulator ({})".format(self.num))

  def write_block_data(self, addr, cmd, data):
    self.log.info("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))

  def write_byte_data(self, addr, cmd, data):
    self.log.info("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))

  def write_word_data(self, addr, cmd, data):
    self.log.info("Write @ {} CMD: {} Data: {}".format(addr, cmd, data))


if __name__ == '__main__':
  main()

"""
### Address of the slave device
addr = 0x08

### Example values
cmd = 0x55        # Command or register
byteVal = 0x1A    # byte value
wordVal = 0xABCD  # 2 byte value
listVal = [5, 10, 15, 50]  # value list

### Write operations
bus.write_byte(addr, byteVal)
bus.write_byte_data(addr, cmd, byteVal)
bus.write_word_data(addr, cmd, wordVal)
bus.write_block_data(addr, cmd, listVal)

### Read operations
readByte = bus.read_byte(addr)
readByte = bus.read_byte_data(addr, cmd)
readWord = bus.read_word_data(addr, cmd)
readList = bus.read_block_data(addr, cmd)
"""

""" List of smbus commands

Send only the read / write bit 
long write_quick(int addr)

Read a single byte from a device, without specifying a device register. 
long read_byte(int addr)

Send a single byte to a device (without command or register)
long write_byte(int addr, char val)

Read a single byte from a device (cmd is the command or register declaration)
long read_byte_data(int addr, char cmd)

Send a single byte to a device (cmd is the command or register declaration)
long write_byte_data(int addr, char cmd, char val)
 
Read a 16 Bit word from a device (cmd is the command or register declaration)
long read_word_data(int addr, char cmd)

Send a 16 Bit word from to a device (cmd is the command or register declaration)
long write_word_data(int addr, char cmd, int val)

Read a block of data from a device (cmd is the command or register declaration)
long[] read_block_data(int addr, char cmd)

Send a block of data to a device (cmd is the command or register declaration).
The data block should be maximum 31 Byte.
The function adds a length byte before the data bytes.
write_block_data(int addr, char cmd, long vals[])

Process Call transaction
long process_call(int addr, char cmd, int val)

Block Process Call transaction
long[] block_process_call(int addr, char cmd, long vals[])

Read a block of raw data from a device (cmd is the command or register declaration)
long[] read_i2c_block_data(int addr, char cmd)

Send a block of raw data to a device (cmd is the command or register declaration).
write_i2c_block_data(int addr,char cmd, long vals[])

"""


