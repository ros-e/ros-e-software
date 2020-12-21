#!/usr/bin/env python

# see http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# and http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

from Emotion import Emotion

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, String, UInt8, Int16, ColorRGBA
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray 
from visual.msg import DisplayConnectedPixel, Point


"""
"""
class talker(Node):
    def __init__(self):
        super().__init__('')
        pubEyeRight = self.create_publisher(DisplayConnectedPixel, 'visual/display/right/setEye', 10)
        pubEyeLeft = self.create_publisher(DisplayConnectedPixel, 'visual/display/left/setEye', 10)

        pubIrisRight = self.create_publisher(DisplayConnectedPixel, 'visual/display/right/setIris', 10)
        pubIrisLeft = self.create_publisher(DisplayConnectedPixel, 'visual/display/left/setIris', 10)

        displayClear = self.create_publisher(UInt8, 'visual/display/clear', 10)

        rclpy.init(args=args) # 'eyeNode'

        # Clear display
        # displayClear.publish(UInt8(data=0))

        # eye
        self.get_logger().debug('eye: {}'.format(Emotion.ANGRY_LEFT.value))
        d = DisplayConnectedPixel()
        
        points = []
        for tup in Emotion.ANGRY_LEFT.value:
            p = Point()
            p.row = tup[1]
            p.col = tup[0]
            points.append(p)
        
        d.points = points
        d.curve = 0     # linear
        d.value = 1     # white
        d.animation = 0

        pubEyeLeft.publish(d)

        # iris
        self.get_logger().debug('iris: {}'.format(Emotion.NORMAL_LEFT_IRIS_RIGHT_UP.value))
        d = DisplayConnectedPixel()

        points = []
        for tup in Emotion.NORMAL_LEFT_IRIS_RIGHT_UP.value:
            p = Point()
            p.row = tup[1]
            p.col = tup[0]
            points.append(p)
        
        d.points = points
        d.curve = 0     # linear
        d.value = 0     # black
        d.animation = 0

        pubIrisLeft.publish(d)

        d = DisplayConnectedPixel()
        
        points = []
        for tup in Emotion.ANGRY_RIGHT.value:
            p = Point()
            p.row = tup[1]
            p.col = tup[0]
            points.append(p)
        
        d.points = points
        d.curve = 0     # linear
        d.value = 1     # white
        d.animation = 0

        pubEyeRight.publish(d)

        # iris
        self.get_logger().debug('iris: {}'.format(Emotion.NORMAL_RIGHT_IRIS_RIGHT_UP.value))
        d = DisplayConnectedPixel()

        points = []
        for tup in Emotion.NORMAL_RIGHT_IRIS_RIGHT_UP.value:
            p = Point()
            p.row = tup[1]
            p.col = tup[0]
            points.append(p)
        
        d.points = points
        d.curve = 0     # linear
        d.value = 0     # black
        d.animation = 0

        pubIrisRight.publish(d)



asyncPublisher = None


def asyncTalker():
    global asyncPublisher
    asyncPublisher = self.create_publisher(String, 'chatter', 10)
    rclpy.init(args=args) # 'talker'

    updateRate = 10   # 10Hz

    # Create an async timer with callback method
    mainTimer = self.create_timer(1.0 / updateRate, asyncCallback)

    # Wait for ROS shutdown and process timer in the background
    rclpy.spin(node)
    node.onShutdown()
    node.destroy_node()
    rclpy.shutdown()


def asyncCallback():
    global asyncPublisher
    hello_str = "hello world %s" % self.get_clock().now()
    asyncPublisher.publish(hello_str)


def main(args=None):
    talker()
    # or
    # asyncTalker()

if __name__ == '__main__':
  main()