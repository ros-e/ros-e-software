#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os
import sys

import json

import redis

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, UInt8, Int16
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 
from visual.msg import DisplayConnectedPixel, Point


class Colors():
  WHITE = 1
  BLACK = 0

#TODO last eye und last iris in redis speichern und in der Api beim get als letzten Wert zurückggeben



class PolygonControl(Node):
  """ 
  For each Polygon (e.g. Iris and Eye are 2 different polygons) create a PolygonControl object 
  """

  def __init__(self, name, side, setByNameTopic, displayControlTopic, lastTopic, color):

    super().__init__('eyeControl_node_{}'.format(name))

    self.r = redis.Redis(host="localhost", port=6379, db=0)
    self.p = self.r.pubsub(ignore_subscribe_messages=True)

    self.redisLastTopic = lastTopic

    self.color = color
    self.name = name
    self.side = side

    ### Polygon set topics subscriber
    self.create_subscription(String, setByNameTopic, self.onSetByName, 10)

    self.displayPublisher = self.create_publisher(DisplayConnectedPixel, displayControlTopic, 10)

    
    self.get_logger().info('Started Eye Control Nodes{}'.format(name))

  def getPointsFromName(self, name):
    jsonString = self.r.get("eyes/{}/{}".format(name, self.name))
    jsonData = []
    if (jsonString != None):
      try:
        jsonData = json.loads(jsonString)
      except Exception as e:
        pass
    
    return jsonData
    

  
  def onSetByName(self, msg):

    pointData = self.getPointsFromName(msg.data)
    if pointData == None: pointData = []

    self.get_logger().info("Set ney eye {} with name {}".format(self.name, msg.data))
    self.get_logger().info(msg)
    
    # if len(pointData) == 0:
    #   return
    
    d = DisplayConnectedPixel()

    # TODO: neues DisplayConnectedPixel Forma mit x,y,c in jedem Punkt
    
    self.get_logger().info(pointData)

    points = []

    for pd in pointData:
      p = Point()
      p.row = pd['y']
      p.col = pd['x']
      #p.x
      #p.y
      #p.c
      points.append(p)

    d.points = points
    d.curve = 0     # linear
    d.value = self.color     # white
    d.animation = 0

    self.displayPublisher.publish(d)
    self.get_logger().info("Published new {} with name {}".format(self.name, msg.data))
    self.r.set(self.redisLastTopic, msg.data)




class EyesControl(object):
  def __init__(self):


    leftEye = PolygonControl(name="leftEye",side="left", 
      setByNameTopic="eyes/left/setEye/byName", displayControlTopic="visual/display/left/setEye",
      lastTopic="eyes/left/lastEye",
      color=Colors.WHITE)

    rightEye = PolygonControl(name="rightEye",side="right", 
      setByNameTopic="eyes/right/setEye/byName", displayControlTopic="visual/display/right/setEye",
      lastTopic="eyes/right/lastEye",
      color=Colors.WHITE)

    leftIris = PolygonControl(name="leftIris",side="left", 
      setByNameTopic="eyes/left/setIris/byName", displayControlTopic="visual/display/left/setIris",
      lastTopic="eyes/left/lastIris",
      color=Colors.BLACK)

    rightIris = PolygonControl(name="rightIris",side="right", 
      setByNameTopic="eyes/right/setIris/byName", displayControlTopic="visual/display/right/setIris",
      lastTopic="eyes/right/lastIris",
      color=Colors.BLACK)
      


def main(args=None):

  rclpy.init(args=args) # 'eyeControl_node'
  
  # TODO: Hier müssen iwie alle Knoten gespint werden
  node = EyesControl()
  

  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()