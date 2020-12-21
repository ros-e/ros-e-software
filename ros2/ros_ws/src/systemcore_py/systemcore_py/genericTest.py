#!/usr/bin/env python

"""
ROS Node that bridges values between ROS and Redis Database
Used to send values to other frameworks and programms of different languages, e.g. to a Node.js Electron Desktop App
"""

# import redis
import json

from thread import start_new_thread

class RGB(object):
  def __init__(self):
    self.r = 0
    self.g = 0
    self.b = 0
    self.a = 0

  def __setitem__(self, k, v):
    self.k = v

def createRosMessage(msg, data):
    m = msg()
    jsonObject = json.loads(data)
    for k in jsonObject:
      print(jsonObject[k])
      m[k] = jsonObject[k]

    return m

obj = json.loads('{"r": 5, "g": 10, "b": 22, "a": 55}')

x = createRosMessage(RGB, obj)
print([x.r, x.g, x.b, x.a] )



