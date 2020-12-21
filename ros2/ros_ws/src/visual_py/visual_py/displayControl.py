#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import sys
import threading
import time

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, UInt8, Int16, ColorRGBA
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 
from visual.msg import DisplayConnectedPixel, Point, DisplayPixel

# class Commands():
#   MOTOR_SET_STIFFNESS = 0x10
#   MOTOR_TURN_SET_ABSOLUTE = 0x11
#   MOTOR_TURN_SET_RELATIVE = 0x12
#   MOTOR_PITCH_SET_ABSOLUTE = 0x13
#   MOTOR_PITCH_SET_RELATIVE = 0x14

#   LED_SET_RGB = 0x51

# See: https://icampusnet.th-wildau.de/gitlab/ros-e/tischroboter-software-sbc/wikis/Arduino-I2C-Kommandos
# DISPLAY_CLEAR        = 0x70
# DISPLAY_SET_PIXEL    = 0x71
# DISPLAY_SET_EYE      = 0x72
# DISPLAY_EYE_DATA     = 0x73
# DISPLAY_SET_IRIS     = 0x74
# DISPLAY_IRIS_DATA    = 0x75

i2cSem = threading.Semaphore()

class DisplayControl(Node):
  def __init__(self, left):

    super().__init__('displayControl_node')

    if left: 
      self.arduinoI2C = 0x0A
      self.side = "left"
    else: 
      self.arduinoI2C = 0x0B
      self.side = "right"

    # Messages for set eye pixel
    self.create_subscription(DisplayConnectedPixel, "visual/display/setEye", self.onSetEye, 10)
    self.create_subscription(DisplayConnectedPixel, "visual/display/" + self.side + "/setEye", self.onSetEye, 10)

    # Messages for set iris pixel
    self.create_subscription(DisplayConnectedPixel, "visual/display/setIris", self.onSetIris, 10)
    self.create_subscription(DisplayConnectedPixel, "visual/display/" + self.side + "/setIris", self.onSetIris, 10)

    # Messages for Display a single pixel on a Display
    self.create_subscription(DisplayPixel, "visual/display/setPixel", self.onSetPixel, 10)
    self.create_subscription(DisplayPixel, "visual/display/" + self.side + "/setPixel", self.onSetPixel, 10)

    # Messages for clearing the displays
    self.create_subscription(UInt8, "visual/display/clear", self.onClearDisplay, 10)
    self.create_subscription(UInt8, "visual/display/" + self.side + "/clear", self.onClearDisplay, 10)

    self.pubI2Cwrite8 = self.create_publisher(I2Cwrite8, "system/i2c/write8", 10)
    self.pubI2Cwrite16 = self.create_publisher(I2Cwrite16, "system/i2c/write16", 10)
    self.pubI2CwriteArray = self.create_publisher(I2CwriteArray, "system/i2c/writeArray", 10)

    ################################
    ### LOG INFO ###################
    self.get_logger().info("################### Init {} display node ################### ".format(self.side))
    self.get_logger().info("Subscribed: visual/display/setEye        |  Msg: visual/DisplayConnectedPixel")
    self.get_logger().info("Subscribed: visual/display/{}/setEye  |  Msg: visual/DisplayConnectedPixel".format(self.side))

    self.get_logger().info("Subscribed: visual/display/setIris       |  Msg: visual/DisplayConnectedPixel")
    self.get_logger().info("Subscribed: visual/display/{}/setIris |  Msg: visual/DisplayConnectedPixel".format(self.side))

    self.get_logger().info("Subscribed: visual/display/setPixel        |  Msg: visual/DisplayPixel")
    self.get_logger().info("Subscribed: visual/display/{}/setPixel  |  Msg: visual/DisplayPixel".format(self.side))

    self.get_logger().info("Subscribed: visual/display/clear         |  Msg: std_msgs/UInt8")
    self.get_logger().info("Subscribed: visual/display/{}/clear   |  Msg: std_msgs/UInt8".format(self.side))

    self.get_logger().info("--------------------------------------------------------------------------")
    self.get_logger().info("Publish on: system/i2c/write8      |  Msg: system/I2Cwrite8")
    self.get_logger().info("Publish on: system/i2c/write16     |  Msg: system/I2Cwrite16")
    self.get_logger().info("Publish on: system/i2c/writeArray  |  Msg: system/I2CwriteArray")

    self.get_logger().info("Started {} Display Control Node".format("left" if left else "right"))

  def transmitPixelData(self, msg, cmdControlData, cmdData):
    
    i2cSem.acquire()
    
    time.sleep(0.01)    

    # Transmit the control data
    o = I2CwriteArray()
    o.address = self.arduinoI2C
    o.command = cmdControlData
    o.data = [int(msg.curve), int(msg.animation), int(msg.value), int(len(msg.points))]

    self.pubI2CwriteArray.publish(o)
    time.sleep(0.01)

    # Transmit the pixel data
    o = I2CwriteArray()
    o.address = self.arduinoI2C
    o.command = cmdData

    data = []

    for point in msg.points:
      data.append(int(point.row))
      data.append(int(point.col))

    o.data = data
    self.pubI2CwriteArray.publish(o)

    i2cSem.release()



  def onSetEye(self, msg):
    self.get_logger().info("Set {} eye --> c: {} | v: {} | a: {} \n{}".format(self.side, msg.curve, msg.value, msg.animation, msg.points))

    self.transmitPixelData(msg, 0x72, 0x73)


  def onSetIris(self, msg):
    self.get_logger().info("Set {} iris --> c: {} | v: {} | a: {} \n{}".format(self.side, msg.curve, msg.value, msg.animation, msg.points))

    self.transmitPixelData(msg, 0x74, 0x75)



  def onSetPixel(self, msg):
    self.get_logger().info("Set new pixel on {} display: {}, {}, {}".format(self.side, msg.r, msg.c, msg.v))

    o = I2CwriteArray()
    o.address = self.arduinoI2C
    o.command = 0x71
    o.data = [int(msg.r), int(msg.c), int(msg.v)]

    self.pubI2CwriteArray.publish(o)


  def onClearDisplay(self, msg):
    self.get_logger().info("Clear {} display: {}".format(self.side, msg.data))

    o = I2Cwrite8()
    o.address = self.arduinoI2C
    o.command = 0x70
    o.data = msg.data

    self.pubI2Cwrite8.publish(o)
    time.sleep(0.01)

def main(args=None):

  rclpy.init(args=args) # 'displayControl_node'

  
  nodeLeft = DisplayControl(True)
  nodeRight = DisplayControl(False)


  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()