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
from visual.msg import Polygon, PolygonPoint # DisplayConnectedPixel, Point


class Colors():
  WHITE = 1
  BLACK = 0

#TODO last eye und last iris in redis speichern und in der Api beim get als letzten Wert zurückggeben



class PolygonControl():
  """ 
  For each Polygon (e.g. Iris and Eye are 2 different polygons) create a PolygonControl object 
  """

  def __init__(self, parentNode, name, side, setByNameTopic, displayControlTopic, lastTopic, color, defaultPolygon = None):

    self.name = 'eyeControl_node_{}'.format(name)
    self.parentNode = parentNode
    self.logger = self.parentNode.get_logger()

    self.r = redis.Redis(host="localhost", port=6379, db=0)
    self.p = self.r.pubsub(ignore_subscribe_messages=True)

    self.redisLastTopic = lastTopic

    self.color = color
    self.name = name
    self.side = side

    ### Polygon set topics subscriber
    self.parentNode.create_subscription(String, setByNameTopic, self.onSetByName, 10)

    self.displayPublisher = self.parentNode.create_publisher(Polygon, displayControlTopic, 10)

    if defaultPolygon is not None:
      self.publishPolygon(defaultPolygon)


    
    self.logger.info('Started Eye Control Nodes{}'.format(name))

  def getPointsFromName(self, name):
    jsonString = self.r.get("eyes/{}/{}".format(name, self.name))
    jsonData = []
    if (jsonString != None):
      try:
        jsonData = json.loads(jsonString)
      except Exception as e:
        pass
    
    return jsonData
    
  def publishPolygon(self, polygon):
    self.displayPublisher.publish(polygon)
  
  def onSetByName(self, msg):
    pass

  #   pointData = self.getPointsFromName(msg.data)
  #   if pointData == None: pointData = []

  #   self.get_logger().info("Set ney eye {} with name {}".format(self.name, msg.data))
  #   self.get_logger().info(msg)
    
  #   # if len(pointData) == 0:
  #   #   return
    
  #   d = DisplayConnectedPixel()

  #   # TODO: neues DisplayConnectedPixel Forma mit x,y,c in jedem Punkt
    
  #   self.get_logger().info(pointData)

  #   points = []

  #   for pd in pointData:
  #     p = Point()
  #     p.row = pd['y']
  #     p.col = pd['x']
  #     #p.x
  #     #p.y
  #     #p.c
  #     points.append(p)

  #   d.points = points
  #   d.curve = 0     # linear
  #   d.value = self.color     # white
  #   d.animation = 0

  #   self.displayPublisher.publish(d)
  #   self.get_logger().info("Published new {} with name {}".format(self.name, msg.data))
  #   self.r.set(self.redisLastTopic, msg.data)




class EyesControl(Node):
  def __init__(self):
    super().__init__('eyeControl_node')

    defaultEyePoints = [
      PolygonPoint(x= 10, y= 10, c= -1, cx= 30, cy= 5, view_influence= 50),
      PolygonPoint(x= 50, y= 10, c= 0, cx= 0, cy= 0, view_influence= 50),
      PolygonPoint(x= 50, y= 80, c= -1, cx= 30, cy= 90, view_influence= 50),
      PolygonPoint(x= 10, y= 80, c= 0, cx= 0, cy= 0, view_influence= 50)
    ]

    defaultIrisPoints = [
      PolygonPoint(x= 20, y= 30, c= 0, cx= 0, cy= 0, view_influence= 100),
      PolygonPoint(x= 40, y= 30, c= 0, cx= 0, cy= 0, view_influence= 100),
      PolygonPoint(x= 40, y= 60, c= 0, cx= 0, cy= 0, view_influence= 100),
      PolygonPoint(x= 20, y= 60, c= 0, cx= 0, cy= 0, view_influence= 100),
    ]

    defaultIris = Polygon()
    defaultIris.points = defaultIrisPoints

    defaultEye = Polygon()
    defaultEye.points = defaultEyePoints

    leftEye = PolygonControl(parentNode = self, name="leftEye",side="left", defaultPolygon=defaultEye,
      setByNameTopic="eyes/left/setEye/byName", displayControlTopic="visual/display/left/setEye",
      lastTopic="eyes/left/lastEye",
      color=Colors.WHITE)

    rightEye = PolygonControl(parentNode = self, name="rightEye",side="right", defaultPolygon=defaultEye, 
      setByNameTopic="eyes/right/setEye/byName", displayControlTopic="visual/display/right/setEye",
      lastTopic="eyes/right/lastEye",
      color=Colors.WHITE)

    leftIris = PolygonControl(parentNode = self, name="leftIris",side="left", defaultPolygon=defaultIris, 
      setByNameTopic="eyes/left/setIris/byName", displayControlTopic="visual/display/left/setIris",
      lastTopic="eyes/left/lastIris",
      color=Colors.BLACK)

    rightIris = PolygonControl(parentNode = self, name="rightIris",side="right", defaultPolygon=defaultIris,
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