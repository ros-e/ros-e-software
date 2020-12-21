#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import sys

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, String, Int16, ColorRGBA
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 

class Commands():
  MOTOR_SET_STIFFNESS = 0x10
  MOTOR_TURN_SET_ABSOLUTE = 0x11
  MOTOR_TURN_SET_RELATIVE = 0x12
  MOTOR_PITCH_SET_ABSOLUTE = 0x13
  MOTOR_PITCH_SET_RELATIVE = 0x14

  LED_SET_RGB = 0x51

class LedControl(Node):
  def __init__(self):
    super().__init__('ledControl_node')

    self.arduinoI2C = 0x09

    self.create_subscription(ColorRGBA, "visual/led/rgb", self.onSetLedRGB, 10)

    self.pubI2Cwrite16 = self.create_publisher(I2Cwrite16, "system/i2c/write16", 10)
    self.pubI2CwriteArray = self.create_publisher(I2CwriteArray, "system/i2c/writeArray", 10)

    ################################
    ### LOG INFO ###################
    self.get_logger().info("Subscribed: visual/led/rgb   |  Msg: std_msgs/ColorRGBA")
    self.get_logger().info("-----------------------------------------------------------")
    self.get_logger().info("Publish on: system/i2c/writeArray  |  Msg: system/I2CwriteArray")
    self.get_logger().info("Publish on: system/i2c/write16     |  Msg: system/I2Cwrite16")

    self.get_logger().info("Started LED Control Node")

  def onSetLedRGB(self, msg):
    self.get_logger().info("Set new RGB value for LED: {}, {}, {}, {}".format(msg.r, msg.g, msg.b, msg.a))

    o = I2CwriteArray()
    o.address = self.arduinoI2C
    o.command = Commands.LED_SET_RGB
    o.data = [int(msg.r), int(msg.g), int(msg.b), int(msg.a)]

    self.pubI2CwriteArray.publish(o)


def main(args=None):

  rclpy.init(args=args) # 'ledControl_node'

  node = LedControl()

  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()