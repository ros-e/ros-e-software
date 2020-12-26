#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
ROS 2 Node to control all LEDs of ROSE.
"""


import os
import sys

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, String, Int16, Bool
from visual.msg import LED, LEDs
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 

import redis

import json

class ArduinoAddresses():
  MOUTH = 0x20

class Commands():
  LED_STRIP_TURN_ALL_OFF = 0x01
  LED_STRIP_RESET_TO_COLOR = 0x10
  LED_STRIP_UPDATE_TO_COLOR = 0x11


class LedControl():
  """
  Object representing a single LED control of the robot.
  Each new physical Arduino (or some other I2C) should get its own LedControl-Object in this script.
  """

  def __init__(self, parentNode,
      name, numLEDs,
      redisKey_currentLEDs,
      rosTopic_turnAllOff, rosTopic_resetToColor, rosTopic_updateToColor,
      i2cAddress, i2cArrayPublisher
    ) -> None:
    """Constructor for LedControl Object

        Args:
            parentNode (Node): The ROS2-Node over which subscriptions and publisher are created. Be aware to have only ONE single node instance for every started ROS-node.
            name (String): Name of the LedControl

    """
    super().__init__()
    
    self.parentNode = parentNode
    self.name = name

    ### Create python LED objects
    self.leds = {}
    for i in range(numLEDs):
      l = LED()
      l.id = i
      self.leds[i] = l 


    ### Create redis objects
    self.r = redis.Redis(host="localhost", port=6379, db=0) # Redis object to store and get key-values
    self.p = self.r.pubsub(ignore_subscribe_messages=True)  # PubSub to publish redis messages

    ### Redis publish topics for current led status
    self.redisKey_currentLEDs = redisKey_currentLEDs

    self.i2cAddress = i2cAddress
    self.i2cArrayPublisher = i2cArrayPublisher

    ### Ros subscriber topics
    self.rosTopic_turnAllOff = rosTopic_turnAllOff
    self.rosTopic_resetToColor = rosTopic_resetToColor
    self.rosTopic_updateToColor = rosTopic_updateToColor

    ### Motor specific topics subscriber and publisher
    if (self.rosTopic_turnAllOff is not None): self.parentNode.create_subscription(Bool, self.rosTopic_turnAllOff, self.onTurnAllOf, 10)
    if (self.rosTopic_resetToColor is not None): self.parentNode.create_subscription(LEDs, self.rosTopic_resetToColor, self.onResetToColor, 10)
    if (self.rosTopic_updateToColor is not None): self.parentNode.create_subscription(LEDs, self.rosTopic_updateToColor, self.onUpdateToColor, 10)


    self.logger = self.parentNode.get_logger()


    self.logger.info("Subscribed: {:20}  |  Msg: std_msgs/Bool".format(self.rosTopic_turnAllOff))
    self.logger.info("Subscribed: {:20}  |  Msg: visual/LEDs".format(self.rosTopic_resetToColor))
    self.logger.info("Subscribed: {:20}  |  Msg: visual/LEDs".format(self.rosTopic_updateToColor))
    
    self.logger.info("Publish on: system/i2c/writeArray  |  Msg: system/I2CwriteArray")    

  def onShutdown(self):
    pass

  def publishLEDsOnRedis(self):

    jsonData = { "leds": []}

    for k in self.leds:
      l = self.leds[k]
      dat = { "id": l.id, "r": l.r, "g": l.g, "b": l.b }
      jsonData["leds"].append(dat)
      
    j = json.dumps(jsonData)

    if self.redisKey_currentLEDs is not None:
      self.r.set(self.redisKey_currentLEDs, j)    
      self.r.publish(self.redisKey_currentLEDs, j)    


  def pubLedMessage(self, cmd, data):

    self.logger.info("[{}] --> got CMD {} and data {} ".format(self.name, cmd, data))
    
    o = I2CwriteArray()
    o.address = self.i2cAddress
    o.command = cmd
    o.data = data

    self.i2cArrayPublisher.publish(o)

  def onTurnAllOf(self, msg):
    if msg.data:
      self.pubLedMessage(Commands.LED_STRIP_TURN_ALL_OFF, [])    

  def onResetToColor(self, msg):

    # Reset all stored LED objects
    for k in self.leds:
      self.leds[k].r = 0
      self.leds[k].g = 0
      self.leds[k].b = 0

    # Update all LED objects from message
    for led in msg.leds:
      self.leds[led.id].r = led.r
      self.leds[led.id].g = led.g
      self.leds[led.id].b = led.b


    # Optimize pakages of LEDs with same color
    colors = {}
    for k in self.leds:
      r = self.leds[k].r
      g = self.leds[k].g
      b = self.leds[k].b
      c = (r << 16) + (g << 8) + b
      
      if c not in colors: 
        colors[c] = []
      colors[c].append(k)
        
    # For every color send a message to change the related LEDs
    for c in colors:
      r = (c >> 16) & 0xFF
      g = (c >> 8) & 0xFF
      b = c & 0xFF
      d = [0, 0, r, g, b]

      for l in colors[c]:
        if l <= 7: d[1] |= (1 << l)
        else: d[0] |= (1 << (l - 8))

      self.pubLedMessage(Commands.LED_STRIP_UPDATE_TO_COLOR, d)

    self.publishLEDsOnRedis()


  def onUpdateToColor(self, msg):

    colors = {}

    # Update all LED objects from message and store all color combinations for optimization
    for led in msg.leds:
      r = led.r
      g = led.g
      b = led.b
      self.leds[led.id].r = r
      self.leds[led.id].g = g
      self.leds[led.id].b = b

      c = (r << 16) + (g << 8) + b
      
      if c not in colors: 
        colors[c] = []
      colors[c].append(led.id)
        
    # For every color send a message to change the related LEDs
    for c in colors:
      r = (c >> 16) & 0xFF
      g = (c >> 8) & 0xFF
      b = c & 0xFF
      d = [0, 0, r, g, b]

      for l in colors[c]:
        if l <= 7: d[1] |= (1 << l)
        else: d[0] |= (1 << (l - 8))

      self.pubLedMessage(Commands.LED_STRIP_UPDATE_TO_COLOR, d)

    self.publishLEDsOnRedis()


class LedNode(Node):
  def __init__(self):
    super().__init__('ledControl_node')


    ### Publisher for I2C Connection
    self.pubI2Cwrite8 = self.create_publisher(I2Cwrite8, "system/i2c/write8", 10)
    self.pubI2CwriteArray = self.create_publisher(I2CwriteArray, "system/i2c/writeArray", 10)

    self.mouthControl = LedControl(parentNode=self, name="MouthLedControl",
      i2cAddress=ArduinoAddresses.MOUTH, numLEDs=16,
      redisKey_currentLEDs="mouth/leds",
      rosTopic_turnAllOff="mouth/leds/turnAllOff",
      rosTopic_resetToColor="mouth/leds/resetToColor",
      rosTopic_updateToColor="mouth/leds/updateToColor",
      i2cArrayPublisher=self.pubI2CwriteArray,
    )

  def onShutdown(self):
    self.mouthControl.onShutdown()

  #   self.create_subscription(ColorRGBA, "visual/led/rgb", self.onSetLedRGB, 10)

  #   self.pubI2Cwrite16 = self.create_publisher(I2Cwrite16, "system/i2c/write16", 10)
  #   self.pubI2CwriteArray = self.create_publisher(I2CwriteArray, "system/i2c/writeArray", 10)

  #   ################################
  #   ### LOG INFO ###################
  #   self.get_logger().info("Subscribed: visual/led/rgb   |  Msg: std_msgs/ColorRGBA")
  #   self.get_logger().info("-----------------------------------------------------------")
  #   self.get_logger().info("Publish on: system/i2c/writeArray  |  Msg: system/I2CwriteArray")
  #   self.get_logger().info("Publish on: system/i2c/write16     |  Msg: system/I2Cwrite16")

  #   self.get_logger().info("Started LED Control Node")

  # def onSetLedRGB(self, msg):
  #   self.get_logger().info("Set new RGB value for LED: {}, {}, {}, {}".format(msg.r, msg.g, msg.b, msg.a))

  #   o = I2CwriteArray()
  #   o.address = self.arduinoI2C
  #   o.command = Commands.LED_SET_RGB
  #   o.data = [int(msg.r), int(msg.g), int(msg.b), int(msg.a)]

  #   self.pubI2CwriteArray.publish(o)


def main(args=None):

  rclpy.init(args=args) # 'ledControl_node'

  node = LedNode()

  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()