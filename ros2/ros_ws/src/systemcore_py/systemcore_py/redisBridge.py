#!/usr/bin/env python

"""
ROS Node that bridges values between ROS and Redis Database
Used to send values to other frameworks and programms of different languages, e.g. to a Node.js Electron Desktop App
"""

import redis
import json

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int32, Int16, Int8, Float32, Float64, UInt32, UInt16, UInt8, ColorRGBA
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray, RedisMessage
from visual.msg import DisplayPixel, DisplayConnectedPixel, Point
from head.msg import MotorPosition

from sound.msg import Spike


import copy
import time
from systemcore_py.messages import MessageType  



class RedisBridge(Node):

  def __init__(self):

    super().__init__('redisBridgeNode')

    ### Create redis objects
    self.r = redis.Redis(host="localhost", port=6379, db=0)
    self.p = self.r.pubsub(ignore_subscribe_messages=True)

    
    
    #####################################################################################################################
    ### Init ROS & Redis Subscriber and Publisher #######################################################################
    #####################################################################################################################

    ### Generic bridge Redis -> ROS
    self.publisherDict = {}
    self.p.subscribe(**{'message/ros/generic':  (lambda msg: self.onGenericRosMessage(msg['data']))})
    # self.p.subscribe(**{'message/ros/generic':  (lambda msg: self.onGenericRosMessage(msg))})

    ### Generic bridge ROS -> Redis
    self.subGenericRosRedis = self.create_subscription(RedisMessage, 'message/redis/generic', (lambda msg: self.redisPublishGeneric(msg.key, msg.json)), 10)

    ### Sound 
    self.subIsSpeeching = self.create_subscription(Bool, 'sound/is_speeching', (lambda msg: self.redisPublishBool("sound/is_speeching", msg.data)), 10)
    self.subDirection = self.create_subscription(Int32, 'sound/direction', (lambda msg: self.redisPublishInt("sound/direction", msg.data)), 10)

    ### Head
    self.pub_head_motorTurn_setPWM = self.create_publisher(Int16, "head/motorturn/setPWM", 10)
    self.pub_head_motorPitch_setPWM = self.create_publisher(Int16, "head/motorpitch/setPWM", 10)
    self.p.subscribe(**{'head/motorturn/setPWM':  (lambda msg: self.pub_head_motorTurn_setPWM.publish(Int16(data = int(msg['data']))))})
    self.p.subscribe(**{'head/motorpitch/setPWM': (lambda msg: self.pub_head_motorPitch_setPWM.publish(Int16(data = int(msg['data']))))})


    ### Visual

    self.pub_visual_led_rgb = self.create_publisher(ColorRGBA, "visual/led/rgb", 10)
    self.pub_visual_display_pixel = self.create_publisher(DisplayPixel, "visual/display/setPixel", 10)

    # self.p.subscribe(**{'visual/led/rgb': (lambda msg: self.get_logger().info(msg)) })
    self.p.subscribe(**{'visual/led/rgb':  (lambda msg: self.pub_visual_led_rgb.publish(self.rosMessage_ColorRGBA(msg["data"])))})
    self.p.subscribe(**{'visual/display/setPixel':  (lambda msg: self.pub_visual_display_pixel.publish(self.rosMessage_DisplayPixel(msg["data"])))})


    ################################
    ### LOG INFO ###################

    self.get_logger().info("Redis: head/motorturn/setPWM   --> ROS: head/motorturn/setPWM")
    self.get_logger().info("Redis: head/motorpitch/setPWM  --> ROS: head/motorpitch/setPWM")
    self.get_logger().info("Redis: visual/led/rgb          --> ROS: visual/led/rgb")
    

    self.get_logger().info("ROS: sound/mic/isSpeeching   --> Redis: sound/mic/isSpeeching")
    self.get_logger().info("ROS: sound/mic/direction     --> Redis: sound/mic/direction")

    ################################

    #####################################################################################################################

    # self.pub = self.create_publisher(MotorPosition, "head/turn/setAngle", 10)

    ### Start subscriber listen thread
    # start_new_thread(self.redisCallbackListener, (self.p,))
    thread = self.p.run_in_thread(sleep_time=0.001)
    
    self.get_logger().info("Started Redis Bridge Node")
  
    MessageType.node = self

  def onGenericRosMessage(self, jsonData): 
    """ Method to process generic ROS Messages published by the Redis Pub/Sub  """

    # print(jsonData)

    try:

      # The message should contain JSON information
      data = json.loads(jsonData)

      # get the topic of the message, the messageType and the value
      topic = data['topic']
      messageType = data['type']
      value = data  

      # If there is no related rclpy.publisher for the given topic, create and store it for later  
      if topic not in self.publisherDict:
        self.publisherDict[topic] = MessageType.getPublisher(self, topic, messageType)
        print(self.publisherDict)
        time.sleep(0.5)
      
      # Create the ROS-Object and publish it
      o = MessageType.createRosObject(messageType, value)
      self.publisherDict[topic].publish(o)
      print("Object published under " + topic)

    except Exception as e:
      # raise e
      self.get_logger().error(str(e))


  # def redisCallbackListener(self, pubsub):
  #   for message in pubsub.listen():
  #     # print(message)
  #     # self.get_logger().info("Got redis message")
  #     pass

  ###### Callback methods for different types #######

  # ## Method to get generic ROS messages and publish them on Redis
  # def redisPublishGeneric(self, msg):
  #   self.r.publish(msg.key, msg.json)

  ## Method to get generic ROS messages and publish them on Redis
  def redisPublishGeneric(self, key, message):
    # self.get_logger().info(("Got generic message. Publish @ " + key + " : " + message))
    self.r.publish(key, message)

  def redisPublishBool(self, topic, message):
    self.r.publish(topic, (0 if message else 1))
    self.get_logger().info(("Published under " + topic + ": " + ("true" if message else "false")))
  
  def redisPublishInt(self, topic, message):
    self.r.publish(topic, message)
    self.get_logger().info(("Published under " + topic + ": " + str(message)))
  
  def redisPublishFloat(self, topic, message):
    self.r.publish(topic, message)
    self.get_logger().info(("Published under " + topic + ": " + str(message)))


  def test(self, data):
    print(data)

  def rosMessage_ColorRGBA(self, data):
    m = ColorRGBA()
    dat = json.loads(data)
    m.r = dat["r"]
    m.g = dat["g"]
    m.b = dat["b"]
    m.a = dat["a"]
    return m

  def rosMessage_DisplayPixel(self, data):
    m = DisplayPixel()
    dat = json.loads(data)
    m.r = dat["r"]
    m.c = dat["c"]
    m.v = dat["v"]
    return m






def main(args=None):

  ### Announce node
  rclpy.init(args=args) # 'redisBridgeNode'

  node = RedisBridge()

  ### Run forever
  rclpy.spin(node)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()



