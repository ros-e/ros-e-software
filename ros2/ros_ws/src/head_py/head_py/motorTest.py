#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
"""

import os
import sys

import redis

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Int16, Bool
from head.msg import MotorPosition
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray

import time
from threading import Timer

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')

        self.motorTurnPub = self.create_publisher(MotorPosition, "head/turn/setAngle", 10)
        self.motorPitchPub = self.create_publisher(MotorPosition, "head/pitch/setAngle", 10)
        self.testPub = self.create_publisher(String, "test/motor/sub", 10)


        time.sleep(0.5)


        # s = String()
        # s.data = "Test 1"

        # self.testPub.publish(s)

        to = MotorPosition()
        to.angle = 20
        to.duration = 1000
        to.speed = 0.0

        
        po = MotorPosition()
        po.angle = -10
        po.duration = 1000
        po.speed = 0.0


        self.motorPitchPub.publish(po)
        self.get_logger().info("Published pitch.")

        self.motorTurnPub.publish(to)
        self.get_logger().info("Published turn.")

        # time.sleep(2)


        # s = String()
        # s.data = "Test 2"

        # self.testPub.publish(s)

    def onShutdown(self):
      pass


def main(args=None):

  rclpy.init(args=args)

  # Init all motors
  node = MotorTester()

  # Spin forever
  # rclpy.spin(node)
  time.sleep(0.5)
  node.onShutdown()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()